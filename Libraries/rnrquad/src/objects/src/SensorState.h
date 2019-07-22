//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_SENSORSTATE_H
#define RNRQUAD_SENSORSTATE_H

class SensorState
{
public:
  SensorState();
  SensorState(bool isEmpty);


  float batteryVoltage;

  // Flow sensor
  float flowX;
  float flowY;
  float flowQ;
  float flowTimeStamp;

  // Gyro sensor
  float gyroX;
  float gyroY;
  float gyroZ;
  float gyroTimeStamp;

  // Accelerometer
  float accelX;
  float accelY;
  float accelZ;
  float accelTimeStamp;

  // Lidars
  Lidar lidarFront;
  Lidar lidarTop;
  Lidar lidarLeft;
  Lidar lidarRight;
  Lidar lidarBottom;
};
extern SensorState refSensor;
#endif
