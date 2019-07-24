//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

PID dPID;

void setup()
{
  baseSetup();

  dPID.begin(.3, 1, 0, .5);
}

void loop()
{
  baseLoop();

  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  if (controllerState.throttle > .5)
  {
    controllerState.throttle = dPID.tstep(sensorState.rangeDown);
  }

  setControllerState(controllerState);
}