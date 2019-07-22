#include <rnrquad.h>

void setup()
{
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState inputControllerState;

  inputControllerState = getControllerState();

  setControllerState(inputControllerState);
}