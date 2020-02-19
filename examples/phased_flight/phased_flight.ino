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
float lrfaKP = 1.0;
float lrfaKD = 0.1;
float lrfaKI = 0.1;

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
  addSym( &lrPID.target, "lrtar", "lr target", "3");
  addSym( &faPID.target, "fatar", "fa target", "3");
}
int phase = 0;
void showFraction(float fraction) {
  for ( int i = 1; i < 5; i++) {
    if ( ((float)i) * 0.2 > fraction ) {
      magentaSet.constant(i, yellow);
    } else {
      magentaSet.constant(i, blue);
    }
  }
}
boolean takeOffPointSet = false;
float takeOffLR = 0.0;
float takeOffFA = 0.0;
const float tau = 3.2425926535 / 2.0;
// Given a value of 0 to 1 on a linear scale map it to
// 0 to 1, but start quickly and ramp down, at 1.0 it will not be changing.
// This is the first quadrant of a sine wave
float rampDown(float pos) {
  float angle = pos * tau; // Convert 0.0 to 1.0 to angle expressed in radians.
  float result = sin(angle);
  return result;
}
// Given a value of 0 to 1 on a linear scale map it to
// 0 to 1, but ramping up slowly, at 1.0 it will be moving fast.
// This is 1+ the fourth quadrant of the sine wave
float rampUp(float pos) { 
  float angle = pos * tau; // Convert 0.0 to 1.0 to angle expressed in radians.
  float result = 1.0 - cos(angle);
  return result;
}

float scurve(float pos) {
  if ( pos < 0.5 ) {
    return 0.5 * rampUp (pos * 2.0);
  } else {
    return 0.5 + 0.5 * rampDown ((pos - 0.5) * 2.0);
  }
}
void motion(float flightTime, SensorState sensorState) {
  float timePerPhase = 1.0;
  int phase = (int)(flightTime / timePerPhase);
  float fraction = (flightTime - (timePerPhase * (float)phase)) / timePerPhase;
  float height1 = 0.5;
  const float side = 0.5;
  if ( phase < 4 ) {
    showFraction(fraction);
  } else {
    magentaSet.reset();
  }
  switch (phase) {
    case 0:
      magentaSet.blink(0, red, blue);
      if ( sensorState.rangeDown < 0.1 ) {
        lrPID.reset();
        faPID.reset();
        refSensor.flowPosX = 0.0;
        refSensor.flowPosY = 0.0;
        takeOffLR = sensorState.flowPosX;
        takeOffFA = sensorState.flowPosY;
      }
      lrPID.target = takeOffLR;
      faPID.target = takeOffFA;
      dPID.target = scurve(fraction) * height1;
      break;
    case 1:
      magentaSet.blink(0, red, green);
      lrPID.target = takeOffLR;
      faPID.target = takeOffFA;
      dPID.target = height1;
      break;
    case 2:
      magentaSet.blink(0, yellow, magenta);
      lrPID.target = takeOffLR + side * scurve(fraction);
      faPID.target = takeOffFA;
      dPID.target = height1;
      break;
    case 3:
      magentaSet.blink(0, yellow, green);
      lrPID.target = takeOffLR + side;
      faPID.target = takeOffFA;
      dPID.target = height1 - (scurve(fraction) * height1);
      break;
    /*  case 3:
        magentaSet.blink(0, yellow, cyan);
        lrPID.target = takeOffLR + 0.5;
        faPID.target = takeOffFA + 0.5 * fraction;
        dPID.target = height1;
        break;
      case 4:
        magentaSet.blink(0, yellow, blue);
        lrPID.target = takeOffLR + 0.5 * (1.0 - fraction);
        faPID.target = takeOffFA + 0.5;
        dPID.target = height1;
        break;
      case 5:
        magentaSet.blink(0, yellow,red);
        lrPID.target = takeOffLR;
        faPID.target = takeOffFA + 0.5 * (1.0 - fraction);
        dPID.target = height1;
        break;
      case 6:
        magentaSet.blink(0, yellow, green);
        lrPID.target = takeOffLR;
        faPID.target = takeOffFA;
        dPID.target = height1 - (fraction * height1);
        break;
    */
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
    motion(seconds() - takeOffTime, sensorState);
    throttle = dPID.tstep(sensorState.rangeDown);
    lr = lrPID.tstep(sensorState.flowPosX);
    fa = faPID.tstep(sensorState.flowPosY);
    controllerState.throttle = constrain(throttle, .3 , .8);
    if (sensorState.rangeDown < 0.1 ) {  // Too low. side looking lidar sees the ground.
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
  if ( controllerState.trim0 > 0 ) {
    controllerState.throttle = 0;
  }
  setControllerState(controllerState);
}
