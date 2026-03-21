#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// ===== CONSTANTS =====
#define sensorCount 16

// ===== STRUCT =====
struct SensorCalibration {
  float minCollector[sensorCount];
  float maxCollector[sensorCount];
  float calibratedValues[sensorCount];
  int midPoint;
  float alpha;
  uint16_t calibrationDuration;
};

// ===== GLOBAL =====
extern SensorCalibration sensorCal;

// ===== FUNCTIONS =====
void initSensors();
int sensorRead(int i);
int readCalibrated(int i);

void updateIRValues();
void updateIRValuesWhite();

void sensorCalibrate();
// void calibrationPage();

#endif