#include <rnrquad.h>

void setup()
{
  disableRangeFinders();//
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState (); 

  setControllerState(controllerState);
  static bool notRun = true;
  if ( notRun ) {
    if ( seconds() > 10.0 ) {
      notRun = false;
      enableRangeFinders();
    }
  }
}
