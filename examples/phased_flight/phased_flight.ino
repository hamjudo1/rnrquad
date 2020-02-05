//
// Created by
//

#include <rnrquad.h>

float KP = 0.5;
float KD = 1;
float KI = 0.05;
float target = 0.0;
float minHoverThrottle = .55;
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
  dPID.altitudeMode = false;
  addSym( &(throttle), "t", "throttle", "3");
  addSym( &lr, "lr", "left-right PID result", "3");
  addSym( &fa, "fa", "forward-aft PID result", "3");
  addSym( &dPID.target, "tar", "target altitude", "3");
}
int phase = 0;
void motion(float flightTime) {
  float timePerPhase = 3.0;
  int phase = (int)(flightTime / timePerPhase);
  float fraction = (flightTime - (timePerPhase * (float)phase)) / timePerPhase;
  float height1 = 0.5;
  switch (phase) {
    case 0:
      lrPID.target = 0.0;
      faPID.target = 0.0;
      dPID.target = fraction * height1;
      break;
    case 1:
      lrPID.target = 0.0;
      faPID.target = 0.0;
      dPID.target = height1;
      break;
    case 2:
      lrPID.target = 0.2 * fraction;
      faPID.target = 0.0;
      dPID.target = height1;
      break;
    case 3:
      lrPID.target = 0.2 - (0.2 * fraction);
      faPID.target = 0.0;
      dPID.target = height1;
      break;
    case 4:
      lrPID.target = -0.2 * fraction;
      faPID.target = 0.0;
      dPID.target = height1;
      break;
    case 5:
      lrPID.target = -0.2 - (0.2 * fraction);
      faPID.target = 0.0;
      dPID.target = height1;
      break;
    case 6:
      lrPID.target = 0.0;
      faPID.target = 0.0;
      dPID.target = height1 - (fraction * height1);
      break;
    default:
      lrPID.target = 0.0;
      faPID.target = 0.0;
      dPID.target = 0.0;
      break;
  }
}
void loop()
{
  static float takeOffTime = 0.0;
  baseLoop();

  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  //  dPID.KP = KP + .3 * controllerState.trim0;
  //  dPID.target = target + .1 * controllerState.trim1;

  if (controllerState.throttle > .1 )
  {
    motion(seconds() - takeOffTime);
    throttle = dPID.tstep(sensorState.rangeDown);
    lr = lrPID.tstep(sensorState.flowAveX);
    fa = -faPID.tstep(sensorState.flowAveY);
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
        controllerState.rightStickXPosition = constrain(lr, -0.2, 0.2);
        controllerState.rightStickYPosition = constrain(fa, -0.2, 0.2);
      }
    }
    if ( controllerState.leftStickXPosition > -0.1 && controllerState.leftStickXPosition < 0.1 ) {
      controllerState.leftStickXPosition = 0.0;
    }
    if ( dPID.target < 0.01 ) {
      controllerState.throttle = 0;
    }
  } else {
    controllerState.throttle = 0;
    takeOffTime = seconds();
    dPID.reset();
    lrPID.reset();
    faPID.reset();
  }
  setControllerState(controllerState);
}
