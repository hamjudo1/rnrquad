//
// Created by Florian VanKampen on 2019-06-17.
//

#include "Lidar.h"
#include "SensorState.h"

SensorState::SensorState(bool isEmpty)
{
  batteryVoltage = 0;

  // Flow sensor
  flowX = 0;
  flowY = 0;
  flowQ = 0;
  flowTimeStamp = 0;

  // Gyro sensor
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
  gyroTimeStamp = 0;

  // Accelerometer
  accelX = 0;
  accelY = 0;
  accelZ = 0;
  accelTimeStamp = 0;

  // Lidars
  lidarFront = Lidar();
  lidarTop = Lidar();
  lidarLeft = Lidar();
  lidarRight = Lidar();
  lidarBottom = Lidar();
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

  // Gyro sensor
  gyroX = refSensor.gyroX;
  gyroY = refSensor.gyroY;
  gyroZ = refSensor.gyroZ;
  gyroTimeStamp = refSensor.gyroTimeStamp;

  // Accelerometer
  accelX = refSensor.accelX;
  accelY = refSensor.accelY;
  accelZ = refSensor.accelZ;
  accelTimeStamp = refSensor.accelTimeStamp;

  // Lidars
  lidarFront = refSensor.lidarFront;
  lidarTop =  refSensor.lidarTop;
  lidarLeft =  refSensor.lidarLeft;
  lidarRight =  refSensor.lidarRight;
  lidarBottom =  refSensor.lidarBottom;
};
