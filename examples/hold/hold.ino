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
PID lrPID; // left - right PID
PID faPID; // forward - aft PID
float lrfaKP = 0.01;
float lrfaKD = 1;
float lrfaKI = 0;

float throttle = 0;
float lr = 0.0, fa = 0.0;
void setup()
{
  baseSetup();

  lrPID.begin(0.0, lrfaKP, lrfaKD, lrfaKI, 0.0);
  lrPID.altitudeMode = false;
  faPID.begin(0.0, lrfaKP, lrfaKD, lrfaKI, 0.0);
  faPID.altitudeMode = false;
  dPID.begin(target, KP, KI, KD, minHoverThrottle);
  addSym( &(throttle), "t", "throttle", "3");
  addSym( &lr, "lr", "left-right PID result", "3");
  addSym( &fa, "fa", "forward-aft PID result", "3");
}

void loop()
{
  baseLoop();

  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  //  dPID.KP = KP + .3 * controllerState.trim0;
  //  dPID.target = target + .1 * controllerState.trim1;
  throttle = dPID.tstep(sensorState.rangeDown);
  lr = lrPID.tstep(sensorState.flowAveX);
  fa = -faPID.tstep(sensorState.flowAveY);
  if (controllerState.throttle > .1 )
  {
    controllerState.throttle = constrain(throttle, .3 , .8);
    if (sensorState.rangeDown < 0.3 ) {  // Too low. side looking lidar sees the ground.
      controllerState.rightStickYPosition = 0;
      controllerState.rightStickXPosition = 0;
    } else {
      if (sensorState.rangeForward < .3 ) {
        controllerState.rightStickYPosition = -(0.3 - sensorState.rangeForward);
      } else {
        controllerState.rightStickYPosition = 0;
      } 

      if (sensorState.rangeRight < .3)
      {
        controllerState.rightStickXPosition = -0.25;
      } else if (sensorState.rangeLeft < .3 ) {
        controllerState.rightStickXPosition = +0.25;
      } else {
        controllerState.rightStickXPosition = 0.0;
      }

      if ( 0.0 == controllerState.rightStickXPosition && 0.0 == controllerState.rightStickYPosition ) {
        controllerState.rightStickXPosition = constrain(lr, -0.1, 0.1);
        controllerState.rightStickYPosition = constrain(fa, -0.1, 0.1);
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
