/*
  XIAO nRF52840 Sense — IMU + PDM audio BLE streamer
  ----------------------------------------------------
  Streams two live data feeds over BLE GATT notifications:
    1. IMU  — 6-axis accel + gyro, ~20 Hz, 16 bytes/packet
    2. Audio — PDM mic, downsampled to 8 kHz, compressed 4:1 with
       IMA ADPCM (a standard, very low-complexity codec well suited
       to a Cortex-M4 with no hardware audio codec)

  Board:     Seeed XIAO nRF52840 Sense
  Tools menu: Seeed nRF52 mbed-enabled Boards > Seeed XIAO nRF52840 Sense

  Required libraries (Library Manager):
    - ArduinoBLE
    - Seeed Arduino LSM6DS3   (IMU)
    - PDM                     (bundled with the nRF52 mbed core)

  ---------------------------------------------------------------
  BANDWIDTH NOTE (read this if audio sounds choppy or doesn't arrive):
  The default BLE payload is only 20 bytes per notification. The audio
  packet size below (122 bytes) needs the phone to negotiate a larger
  ATT MTU (~125+), which Chrome/Android and the nRF52840 SoftDevice
  usually do automatically on connect. If audio doesn't come through,
  check the negotiated MTU with the nRF Connect app and reduce
  AUDIO_CHUNK_SAMPLES below (e.g. to 32) so packets fit in 20 bytes —
  you'll get lower throughput but a guaranteed-to-work baseline.
  ---------------------------------------------------------------
*/

#include <ArduinoBLE.h>
#include <PDM.h>
#include <Wire.h>
#include "LSM6DS3.h"

// ---------------- Config ----------------
#define AUDIO_SAMPLE_RATE    8000   // Hz, after 2:1 decimation from 16 kHz PDM
#define AUDIO_CHUNK_SAMPLES  240    // raw samples encoded per BLE packet (must be even)
#define AUDIO_CHUNK_BYTES    (AUDIO_CHUNK_SAMPLES / 2)   // 4-bit ADPCM = 2 samples/byte
#define IMU_INTERVAL_MS      50     // 20 Hz IMU updates

// ---------------- BLE UUIDs ----------------
#define SERVICE_UUID    "19b10000-e8f2-537e-4f6c-d104768a1214"
#define IMU_CHAR_UUID   "19b10001-e8f2-537e-4f6c-d104768a1214"
#define AUDIO_CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEService sensorService(SERVICE_UUID);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLERead | BLENotify, 16);
BLECharacteristic audioChar(AUDIO_CHAR_UUID, BLERead | BLENotify, AUDIO_CHUNK_BYTES + 2);

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// ---------------- PDM capture ----------------
short pdmBuffer[512];

#define PCM_RING_SIZE 4096
short pcmRing[PCM_RING_SIZE];
volatile uint32_t pcmWriteIdx = 0;
uint32_t pcmReadIdx = 0;

// ---------------- IMA ADPCM encoder ----------------
struct AdpcmState {
  int16_t valprev;
  int8_t index;
};
AdpcmState adpcmState = {0, 0};

static const int stepSizeTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,
  80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,
  494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,
  2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,
  8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,
  27086,29794,32767
};
static const int indexTable[16] = {
  -1,-1,-1,-1,2,4,6,8,
  -1,-1,-1,-1,2,4,6,8
};

uint8_t adpcmEncodeSample(int16_t sample, AdpcmState *st) {
  int diff = sample - st->valprev;
  int step = stepSizeTable[st->index];
  int sign = 0;
  if (diff < 0) { sign = 8; diff = -diff; }
  int delta = 0;
  int vpdiff = step >> 3;
  if (diff >= step) { delta = 4; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { delta |= 2; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { delta |= 1; vpdiff += step; }
  if (sign) st->valprev -= vpdiff; else st->valprev += vpdiff;
  if (st->valprev > 32767) st->valprev = 32767;
  else if (st->valprev < -32768) st->valprev = -32768;
  st->index += indexTable[sign | delta];
  if (st->index < 0) st->index = 0;
  if (st->index > 88) st->index = 88;
  return (uint8_t)(sign | delta);
}

// ---------------- PDM callback ----------------
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(pdmBuffer, bytesAvailable);
  int samples = bytesAvailable / 2;
  // Native PDM rate is 16 kHz; simple 2:1 decimation to 8 kHz.
  for (int i = 0; i < samples; i += 2) {
    pcmRing[pcmWriteIdx] = pdmBuffer[i];
    pcmWriteIdx = (pcmWriteIdx + 1) % PCM_RING_SIZE;
  }
}

uint16_t audioSeq = 0;
uint8_t audioPacket[AUDIO_CHUNK_BYTES + 2];

void trySendAudio() {
  uint32_t available = (pcmWriteIdx + PCM_RING_SIZE - pcmReadIdx) % PCM_RING_SIZE;
  if (available < AUDIO_CHUNK_SAMPLES) return;

  audioPacket[0] = (uint8_t)(audioSeq & 0xFF);
  audioPacket[1] = (uint8_t)(audioSeq >> 8);
  for (int i = 0; i < AUDIO_CHUNK_BYTES; i++) {
    int16_t s1 = pcmRing[pcmReadIdx]; pcmReadIdx = (pcmReadIdx + 1) % PCM_RING_SIZE;
    int16_t s2 = pcmRing[pcmReadIdx]; pcmReadIdx = (pcmReadIdx + 1) % PCM_RING_SIZE;
    uint8_t n1 = adpcmEncodeSample(s1, &adpcmState);
    uint8_t n2 = adpcmEncodeSample(s2, &adpcmState);
    audioPacket[2 + i] = (n1 & 0x0F) | (n2 << 4);
  }
  audioChar.writeValue(audioPacket, sizeof(audioPacket));
  audioSeq++;
}

// ---------------- IMU ----------------
uint32_t lastIMUms = 0;

void sendIMU() {
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  uint8_t packet[16];
  uint32_t t = millis();
  memcpy(packet, &t, 4);
  // accel scaled x1000 (g), gyro scaled x10 (dps) to preserve precision as int16
  int16_t vals[6] = {
    (int16_t)(ax * 1000.0f), (int16_t)(ay * 1000.0f), (int16_t)(az * 1000.0f),
    (int16_t)(gx * 10.0f),   (int16_t)(gy * 10.0f),   (int16_t)(gz * 10.0f)
  };
  memcpy(packet + 4, vals, 12);
  imuChar.writeValue(packet, sizeof(packet));
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (myIMU.begin() != 0) {
    Serial.println("IMU init failed");
  } else {
    Serial.println("IMU ready");
  }

  PDM.onReceive(onPDMdata);
  PDM.setGain(30);
  if (!PDM.begin(1, 16000)) {
    Serial.println("PDM init failed");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  BLE.setLocalName("XIAO-Sense-Stream");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(imuChar);
  sensorService.addCharacteristic(audioChar);
  BLE.addService(sensorService);

  // Request a fast connection interval (units of 1.25 ms: 6=7.5ms, 12=15ms).
  // This is a request only — the central may negotiate something slower.
  BLE.setConnectionInterval(0x0006, 0x000C);

  BLE.advertise();
  Serial.println("Advertising as XIAO-Sense-Stream");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    adpcmState.valprev = 0;
    adpcmState.index = 0;
    pcmReadIdx = pcmWriteIdx;

    while (central.connected()) {
      trySendAudio();
      if (millis() - lastIMUms >= IMU_INTERVAL_MS) {
        lastIMUms = millis();
        sendIMU();
      }
    }
    Serial.println("Disconnected");
  }
}
