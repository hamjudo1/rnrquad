#include <rnrquad.h>

float t = 0.5;

void setup()
{
  disableRangeFinders();
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState ();
  if (flightTime() < 2.0) {
    controllerState.throttle = t;
  } else {
    controllerState.throttle = 0;
  }
  setControllerState(controllerState);
}
