#include <rnrquad.h>

float t = 0.5;
float s = 0;
float f = 0;
void setup()
{
  disableRangeFinders();
  baseSetup();
  addSym(&t, "t", "throttle", "3");
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState ();
  if (flightTime() < 2.0) {
    t = 0.5 + controllerState.trim0 * 0.02;
    controllerState.throttle = t;
  } else {
    controllerState.throttle = 0;
  }
  setControllerState(controllerState);

}
