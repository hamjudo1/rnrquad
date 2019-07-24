//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

enum Status
{
  forward,
  turning_left,
};
unsigned long previousTimestamp;
float counter;
Status status = forward;

PID dPID, lrPID, fbPID;

void setup()
{
  baseSetup();

  dPID.begin(.3, 1, 0, .5);
  lrID.begin(0, 1, 0, .5);
  fbID.begin(0, 1, 0, .5);}

void loop()
{
  baseLoop();
  ControllerState inputControllerState = getControllerState();
  ControllerState outputControllerState = ControllerState();
  SensorState sensorState = getSensorState();

  switch (status)
  {
    case forward:
      controllerState = goForward(controllerState);

    case turning_left:
      controllerState = turnLeft(controllerState);
  }
  int altitude = sensorState.rangeDown;

  outputControllerState.throttle = dPID.tstep(altitude);
  outputControllerState.rightStickXPosition = lrPID.tstep(sensorState.flowY);
  outputControllerState.rightStickYPosition = fbPID.tstep(sensorState.flowX);

  setControllerState(outputControllerState);
}

ControllerState turnLeft(ControllerState controllerState)
{
  controllerState.leftStickXPosition = -.5;
  degreesTraveled = sensorState.gyroXYY * (millis() - previousTimestamp);
  counter -= degreesTraveled;

  if (counter <= 0)
  {
    status = forward;
    counter = 1000; // 1000ms
  }
}

ControllerState goForward(ControllerState controllerState
{
  outputControllerState.rightStickYPosition = .5;
  counter -= (millis() - previousTimestamp);
  if (counter <= 0)
  {
    status = turning_left;
    counter = 90; // 90 degrees
  }
}