//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_SENSORSTATE_H
#define RNRQUAD_SENSORSTATE_H

class SensorState {
public:
  SensorState();

  float batteryVoltage;

  // Flow sensor
  float flowX;
  float flowY;
  float flowQ;

  // Gyro sensor
  float gyroX;
  float gyroY;
  float gyroZ;

  // Accelerometer
  float accelX;
  float accelY;
  float accelZ;

  // Lidars
  Lidar lidarFront;
  Lidar lidarTop;
  Lidar lidarLeft;
  Lidar lidarRight;
  Lidar lidarBottom;
};

#endif
