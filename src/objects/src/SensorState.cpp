//
// Created by Florian VanKampen on 2019-06-17.
//

#include "SensorState.h"

SensorState::SensorState(bool isEmpty)
{
  batteryVoltage = 0;

  // Flow sensor
  flowX = 0;
  flowY = 0;
  flowQ = 0;
  flowPosX = 0;
  flowPosY = 0;
  flowPosQ = 0;
  flowPosSamples = 0.0;
  flowPosMissedSamples = 0;
  flowTimeStamp = 0;

  // Gyro sensor
  gyroXY = 0;
  gyroYZ = 0;
  gyroZX = 0;
  gyroTimeStamp = 0;

  // Accelerometer
  accelX = 0;
  accelY = 0;
  accelZ = 0;
  accelTimeStamp = 0;

  // Lidars
  rangeForward = 0.0;
  rangeUp = 0.0;
  rangeLeft = 0.0;
  rangeRight = 0.0;
  rangeDown = 0.0;
};
SensorState refSensor(true);

SensorState::SensorState()
{
  batteryVoltage = refSensor.batteryVoltage;

  // Flow sensor
  flowX = refSensor.flowX;
  flowY = refSensor.flowY;
  flowQ = refSensor.flowQ;
  flowTimeStamp = refSensor.flowTimeStamp;
  flowPosX = refSensor.flowPosX;
  flowPosY = refSensor.flowPosY;
  flowPosQ = refSensor.flowPosQ;
  flowPosSamples = refSensor.flowPosSamples;
  flowPosMissedSamples = refSensor.flowPosMissedSamples;

  // Gyro sensor
  gyroXY = refSensor.gyroXY;
  gyroYZ = refSensor.gyroYZ;
  gyroZX = refSensor.gyroZX;
  gyroTimeStamp = refSensor.gyroTimeStamp;

  // Accelerometer
  accelX = refSensor.accelX;
  accelY = refSensor.accelY;
  accelZ = refSensor.accelZ;
  accelTimeStamp = refSensor.accelTimeStamp;

  // Lidars
  rangeForward = refSensor.rangeForward;
  rangeUp =  refSensor.rangeUp;
  rangeLeft =  refSensor.rangeLeft;
  rangeRight =  refSensor.rangeRight;
  rangeDown =  refSensor.rangeDown;
};
