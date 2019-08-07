#include <rnrquad.h>

void setup()
{
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState (); 

  setControllerState(controllerState);
}
