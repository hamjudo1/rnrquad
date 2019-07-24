//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

PID dPID, lrPID, fbPID;

void setup()
{
  baseSetup();

  dPID.begin(.3, 1, 0, .5);
  lrID.begin(0, 1, 0, .5);
  fbID.begin(0, 1, 0, .5);
}

void loop()
{
  baseLoop();

  ControllerState inputControllerState = getControllerState();
  ControllerState outputControllerState = ControllerState();
  SensorState sensorState = getSensorState();
  int altitude = sensorState.rangeDown;

  outputControllerState.throttle = dPID.tstep(altitude);
  outputControllerState.rightStickXPosition = lrPID.tstep(sensorState.flowY);
  outputControllerState.rightStickYPosition = fbPID.tstep(sensorState.flowX);

  setControllerState(outputControllerState);
}