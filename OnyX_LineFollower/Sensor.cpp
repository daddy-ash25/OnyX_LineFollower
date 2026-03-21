#include "Sensor.h"
#include <algorithm>

// ===== PINS =====
#define s0 41
#define s1 42
#define s2 2
#define s3 1
#define sensorPin 19

const uint8_t muxPins[4] = { s0, s1, s2, s3 };
int lastMuxChannel = -1;

// ===== GLOBAL STRUCT =====
SensorCalibration sensorCal = {
  {0}, {4095}, {0},
  2048,
  0.1,
  6000
};

// ===== INIT =====
void initSensors() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

// ===== MUX SELECT =====
void selectMuxChannel(uint8_t channel) {
  if (channel == lastMuxChannel) return;

  uint8_t changedBits = lastMuxChannel ^ channel;

  for (uint8_t bit = 0; bit < 4; bit++) {
    if (changedBits & (1 << bit)) {
      digitalWrite(muxPins[bit], (channel >> bit) & 1);
    }
  }

  lastMuxChannel = channel;
}

// ===== RAW READ =====
int sensorRead(int i) {
  selectMuxChannel(i);
  return analogRead(sensorPin);
}

// ===== CALIBRATED READ =====
int readCalibrated(int i) {
  int tempHolder = sensorRead(i);
  tempHolder = map(tempHolder,
                   sensorCal.minCollector[i],
                   sensorCal.maxCollector[i],
                   0, 1000);

  return std::clamp(tempHolder, 0, 1000);
}

// ===== UPDATE =====
void updateIRValues() {
  for (int i = 0; i < sensorCount; i++) {
    sensorCal.calibratedValues[i] = readCalibrated(i);
  }
}

void updateIRValuesWhite() {
  for (int i = 0; i < sensorCount; i++) {
    sensorCal.calibratedValues[i] = 1000 - readCalibrated(i);
  }
}

// ===== CALIBRATION =====
void sensorCalibrate() {
  for (int i = 0; i < sensorCount; i++) {
    int temp = sensorRead(i);

    if (temp > sensorCal.midPoint) {
      sensorCal.maxCollector[i] =
        (1 - sensorCal.alpha) * sensorCal.maxCollector[i] +
        sensorCal.alpha * temp;
    } else {
      sensorCal.minCollector[i] =
        (1 - sensorCal.alpha) * sensorCal.minCollector[i] +
        sensorCal.alpha * temp;
    }
  }
}

// void calibrationPage() {
//   for (int i = 0; i < sensorCount; i++) {
//     sensorCal.maxCollector[i] = 4095;
//     sensorCal.minCollector[i] = 0;
//   }

//   unsigned long startTime = millis();

//   while ((millis() - startTime) < sensorCal.calibrationDuration) {
//     sensorCalibrate();
//   }

//   float avgMax = 0, avgMin = 0;
//   int validMaxCount = 0, validMinCount = 0;

//   for (int i = 0; i < sensorCount; i++) {
//     if (sensorCal.maxCollector[i] < 4094) {
//       avgMax += sensorCal.maxCollector[i];
//       validMaxCount++;
//     }
//     if (sensorCal.minCollector[i] > 1) {
//       avgMin += sensorCal.minCollector[i];
//       validMinCount++;
//     }
//   }

//   if (validMaxCount > 0) avgMax /= validMaxCount;
//   if (validMinCount > 0) avgMin /= validMinCount;

//   for (int i = 0; i < sensorCount; i++) {
//     if (sensorCal.maxCollector[i] >= 4094)
//       sensorCal.maxCollector[i] = avgMax;

//     if (sensorCal.minCollector[i] <= 1)
//       sensorCal.minCollector[i] = avgMin;
//   }
// }