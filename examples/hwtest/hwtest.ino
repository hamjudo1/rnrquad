#include <rnrquad.h>

void setup()
{
  disableRangeFinders();
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState (); 

  setControllerState(controllerState);
}
