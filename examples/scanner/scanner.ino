//
// Created by mark kingham on 2019-06-17.
//

#include <rnrquad.h>

float KP = 0.25;
float KD = 3;
float KI = 0.05;
float target = 0.6;
float minHoverThrottle = .60;
//.15
//1.9
//0.2
//.569
PID dPID;

float throttle = 0;

void setup()
{
  baseSetup();

  dPID.begin(target, KP, KI, KD, minHoverThrottle);
  addSym( &(throttle), "t", "throttle", "3");
}

void loop()
{
  baseLoop();

  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  //  dPID.KP = KP + .3 * controllerState.trim0;
  //  dPID.target = target + .1 * controllerState.trim1;
  throttle = dPID.tstep(sensorState.rangeDown);
  if (controllerState.throttle > .1 )
  {
    controllerState.throttle = constrain(throttle, .3 , .8);
    if (sensorState.rangeForward < .6 )
    {
      controllerState.rightStickYPosition = -0.5;
    }

    if (sensorState.rangeRight < .6 )
    {
      controllerState.rightStickXPosition = -0.5;
    }

    if (sensorState.rangeLeft < .6 )
    {
      controllerState.rightStickXPosition = +0.5;
    }
  } else {
    controllerState.throttle = 0;
  }
  setControllerState(controllerState);
}
