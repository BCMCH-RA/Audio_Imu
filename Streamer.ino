/*
  XIAO nRF52840 Sense — IMU + PDM audio BLE streamer
  ----------------------------------------------------
  Streams two live data feeds over BLE GATT notifications:
    1. IMU  — 6-axis accel + gyro, ~200 Hz, 16 bytes/packet
    2. Audio — PDM mic at native 16 kHz, compressed 4:1 with IMA
       ADPCM (a standard, very low-complexity codec well suited to
       a Cortex-M4 with no hardware audio codec)

  Board:     Seeed XIAO nRF52840 Sense
  Tools menu: Seeed nRF52 mbed-enabled Boards > Seeed XIAO nRF52840 Sense

  Required libraries (Library Manager):
    - ArduinoBLE
    - Seeed Arduino LSM6DS3   (IMU) — make sure Arduino_LSM6DS3 (the
      Nano 33 BLE Sense one) is NOT also installed, they collide.
    - PDM                     (bundled with the nRF52 mbed core)

  ---------------------------------------------------------------
  BANDWIDTH NOTE:
  Testing showed this link has a fixed ceiling of roughly 205-210
  GATT notifications per SECOND, total, shared between both
  characteristics — it's a packet-COUNT limit, not a bytes/sec limit
  (audio's own count + IMU's own count stayed nearly constant no
  matter which one we favored). So instead of one BLE packet per IMU
  sample (200/sec), IMU samples are now batched: the sensor is still
  read at the full IMU_ODR_HZ rate, but IMU_BATCH_SIZE samples are
  packed into a single notification. That turns 200 notifications/sec
  into 200/IMU_BATCH_SIZE, leaving the rest of the ~205-210 budget for
  audio. The Serial Monitor prints actual achieved rates once a
  second — use that to re-tune IMU_BATCH_SIZE or AUDIO_CHUNK_SAMPLES
  for your specific phone's link.

  Audio quality: a DC-blocking filter now runs before ADPCM encoding
  (removes a slow offset that otherwise reads as grainy noise), and
  mic gain was dropped from 30 to 20 (AUDIO_GAIN) since 30 was likely
  driving the signal into clipping, which ADPCM encodes as noise too.
  Tune AUDIO_GAIN to taste — too low brings back a hiss-y noise floor.
  ---------------------------------------------------------------
*/

#include <ArduinoBLE.h>
#include <PDM.h>
#include <Wire.h>
#include "LSM6DS3.h"

// ---------------- Config ----------------
#define AUDIO_SAMPLE_RATE    16000  // Hz, native PDM rate (no decimation)
#define AUDIO_CHUNK_SAMPLES  320    // raw samples encoded per BLE packet (must be even)
#define AUDIO_CHUNK_BYTES    (AUDIO_CHUNK_SAMPLES / 2)   // 4-bit ADPCM = 2 samples/byte
#define IMU_INTERVAL_MS      5      // 200 Hz IMU sampling (sensor reads, not notify calls)
#define IMU_ODR_HZ           208    // sensor's own output data rate (closest standard value >=200Hz)
#define AUDIO_GAIN           20     // PDM mic gain (0-255-ish). Was 30 — too hot, likely clipping. Tune to taste.
#define IMU_BATCH_SIZE       8      // samples packed per BLE notification (200Hz/8 = 25 notifies/sec)
#define IMU_SAMPLE_BYTES     13     // 1 (rel. offset ms) + 6 x int16
#define IMU_PACKET_BYTES     (5 + IMU_BATCH_SIZE * IMU_SAMPLE_BYTES)  // 4 (base ts) + 1 (count) + samples

// ---------------- BLE UUIDs ----------------
#define SERVICE_UUID    "19b10000-e8f2-537e-4f6c-d104768a1214"
#define IMU_CHAR_UUID   "19b10001-e8f2-537e-4f6c-d104768a1214"
#define AUDIO_CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEService sensorService(SERVICE_UUID);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLERead | BLENotify, IMU_PACKET_BYTES);
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
// Simple one-pole DC blocker (y[n] = x[n] - x[n-1] + R*y[n-1]). PDM mics
// carry a slowly-wandering DC offset; left in, it biases the ADPCM
// predictor's differences and comes out as a grainy background noise.
// Stripping it before encoding noticeably cleans up the decoded audio.
static float dcPrevX = 0.0f, dcPrevY = 0.0f;
const float DC_R = 0.995f;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(pdmBuffer, bytesAvailable);
  int samples = bytesAvailable / 2;
  for (int i = 0; i < samples; i++) {
    float x = (float)pdmBuffer[i];
    float y = x - dcPrevX + DC_R * dcPrevY;
    dcPrevX = x;
    dcPrevY = y;
    if (y > 32767.0f) y = 32767.0f;
    else if (y < -32768.0f) y = -32768.0f;

    pcmRing[pcmWriteIdx] = (int16_t)y;
    pcmWriteIdx = (pcmWriteIdx + 1) % PCM_RING_SIZE;
  }
}

uint16_t audioSeq = 0;
uint8_t audioPacket[AUDIO_CHUNK_BYTES + 2];

bool trySendAudio() {
  uint32_t available = (pcmWriteIdx + PCM_RING_SIZE - pcmReadIdx) % PCM_RING_SIZE;
  if (available < AUDIO_CHUNK_SAMPLES) return false;

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
  return true;
}

// ---------------- IMU ----------------
uint32_t lastIMUms = 0;
uint32_t audioPacketCount = 0, imuPacketCount = 0, imuSampleCount = 0, lastStatsMs = 0;

uint8_t imuPacket[IMU_PACKET_BYTES];
uint8_t imuBatchCount = 0;
uint32_t imuBatchBaseMs = 0;

// Reads the sensor and appends one sample into the current batch packet.
// Only actually sends a BLE notification once IMU_BATCH_SIZE samples have
// accumulated — returns true when it does.
bool sampleIMU() {
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  uint32_t now = millis();
  if (imuBatchCount == 0) {
    imuBatchBaseMs = now;
    memcpy(imuPacket, &imuBatchBaseMs, 4);
    imuPacket[4] = IMU_BATCH_SIZE;
  }

  uint8_t relOffset = (uint8_t)(now - imuBatchBaseMs);  // batch spans well under 255ms
  int16_t vals[6] = {
    (int16_t)(ax * 1000.0f), (int16_t)(ay * 1000.0f), (int16_t)(az * 1000.0f),
    (int16_t)(gx * 10.0f),   (int16_t)(gy * 10.0f),   (int16_t)(gz * 10.0f)
  };

  uint8_t *dst = imuPacket + 5 + (imuBatchCount * IMU_SAMPLE_BYTES);
  dst[0] = relOffset;
  memcpy(dst + 1, vals, 12);
  imuBatchCount++;

  if (imuBatchCount >= IMU_BATCH_SIZE) {
    imuChar.writeValue(imuPacket, IMU_PACKET_BYTES);
    imuBatchCount = 0;
    return true;
  }
  return false;
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Raise the sensor's own sample rate to match our 200 Hz polling —
  // otherwise readFloatAccelX() etc. just return the same stale value
  // multiple times between real sensor updates.
  myIMU.settings.accelSampleRate = IMU_ODR_HZ;
  myIMU.settings.gyroSampleRate = IMU_ODR_HZ;

  if (myIMU.begin() != 0) {
    Serial.println("IMU init failed");
  } else {
    Serial.println("IMU ready");
  }

  PDM.onReceive(onPDMdata);
  PDM.setGain(AUDIO_GAIN);
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
    lastIMUms = millis();
    lastStatsMs = millis();
    audioPacketCount = 0;
    imuPacketCount = 0;
    imuSampleCount = 0;
    imuBatchCount = 0;

    while (central.connected()) {
      // IMU is checked and sampled FIRST every pass, so a slow audio send
      // never pushes a due IMU sample past its 5 ms deadline.
      if (millis() - lastIMUms >= IMU_INTERVAL_MS) {
        lastIMUms += IMU_INTERVAL_MS;   // catch-up scheduling, no drift buildup
        imuSampleCount++;
        if (sampleIMU()) imuPacketCount++;
      }
      if (trySendAudio()) audioPacketCount++;

      // Once a second, print the ACTUAL achieved rates. imuSampleCount is
      // the true sensor sample rate (target 200); imuPacketCount is how
      // many BLE notifications that took (target 25, at IMU_BATCH_SIZE=8).
      if (millis() - lastStatsMs >= 1000) {
        Serial.print("IMU samples: "); Serial.print(imuSampleCount); Serial.print(" Hz   ");
        Serial.print("IMU pkts: "); Serial.print(imuPacketCount); Serial.print("/s   ");
        Serial.print("Audio pkts: "); Serial.print(audioPacketCount); Serial.println("/s");
        imuSampleCount = 0;
        imuPacketCount = 0;
        audioPacketCount = 0;
        lastStatsMs += 1000;
      }
    }
    Serial.println("Disconnected");
  }
}
