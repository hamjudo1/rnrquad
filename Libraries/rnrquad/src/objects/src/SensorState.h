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
  float flowPosX;
  float flowPosY;
  float flowPosQ;
  float flowPosStartTime;
  float flowTimeStamp;
  float flowPosSamples;

  // Gyro sensor
  float gyroXY;
  float gyroYZ;
  float gyroZX;
  float gyroTimeStamp;

  // Accelerometer
  float accelX;
  float accelY;
  float accelZ;
  float accelTimeStamp;

  // Lidars
  float rangeForward;
  float rangeUp;
  float rangeLeft;
  float rangeRight;
  float rangeDown;
};
extern SensorState refSensor;
#endif
