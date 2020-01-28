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
    if (sensorState.rangeDown < 0.3 ) {  // Too low. side looking lidar sees the ground.
      controllerState.rightStickYPosition = 0;
      controllerState.rightStickXPosition = 0;
    } else {
      if (sensorState.rangeForward < .3 ) {
        controllerState.rightStickYPosition = -(0.3 - sensorState.rangeForward);
      } else if (sensorState.rangeForward < 0.7 ) {
        controllerState.rightStickYPosition = 0;
      } else if (sensorState.rangeForward < 1.0 ) {
        controllerState.rightStickYPosition = ( sensorState.rangeForward - 0.7);
      } else {
        controllerState.rightStickYPosition = 0.3;
      }

      if ( sensorState.rangeRight + sensorState.rangeLeft < 1.5 ) {
        float delta = sensorState.rangeRight - sensorState.rangeLeft;
        controllerState.rightStickXPosition = constrain(delta / 2.0, -0.25, 0.25);
      } else {
        if (sensorState.rangeRight < .5 )
        {
          controllerState.rightStickXPosition = -0.25;
        } else if (sensorState.rangeLeft < .5 ) {
          controllerState.rightStickXPosition = +0.25;
        } else {
          controllerState.rightStickXPosition = 0.0;
        }
      }
    }
    if ( controllerState.leftStickXPosition > -0.1 && controllerState.leftStickXPosition < 0.1 ) {
      controllerState.leftStickXPosition = 0.0;
    }
  } else {
    controllerState.throttle = 0;
  }
  setControllerState(controllerState);
}
