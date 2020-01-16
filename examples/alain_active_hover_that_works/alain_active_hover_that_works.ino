///
// Created by mark kingham on 2019-06-17.
//

#include <rnrquad.h>

float KP = 0.15;
float KD = 3;
float KI = 0.05;
float target = 1;
float minHoverThrottle = .569;
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
}

void loop()
{
  baseLoop();

  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  //  dPID.KP = KP + .3 * controllerState.trim0;
  //  dPID.target = target + .1 * controllerState.trim1;

if (controllerState.throttle < .1)
{
  controllerState.throttle = 0;
}
else if (controllerState.throttle < .9)
{
  controllerState.throttle = (minHoverThrottle - .05);
}
else
{
  controllerState.throttle = constrain(dPID.tstep(sensorState.rangeDown), .3, .8); 
}
  /**bool throttleWentHigh = false;
  if (controllerState.throttle < .0 ) {
    Serial.println("Throttle off");
  }
  else if (controllerState.throttle > .9 )
  {
    throttleWentHigh = true;
    controllerState.throttle = constrain(dPID.tstep(sensorState.rangeDown), .3, .8);
    Serial.println("Throttle high");
  }
  else if ( throttleWentHigh ) // but is not high
  {
    
    Serial.println("Throttle low, but not too low.");
    static bool firstTime = true;
    static float secondsSinceChange = 0;
    if ( firstTime ) {
       firstTime = false;
       secondsSinceChange = seconds();
    } 
    if ( (seconds() - secondsSinceChange) < 1.000) {
          
      controllerState.throttle = (minHoverThrottle -.05);
    } else {
    
      controllerState.throttle = 0;
    }
    
  }

  if (controllerState.throttle > .1 )
  {
    controllerState.throttle = constrain(dPID.tstep(sensorState.rangeDown), .3 , .8);
  }

  else
  {
    controllerState.throttle = 0;
  }**/


  if (sensorState.rangeForward < .5 )
  {
    controllerState.rightStickYPosition = -0.5;
  }

  if (sensorState.rangeRight < .5 )
  {
    controllerState.rightStickXPosition = -0.5;
  }

  if (sensorState.rangeLeft < .5 )
  {
    controllerState.rightStickXPosition = +0.5;
  }

  if (sensorState.rangeUp < .5 )
  {
    controllerState.throttle = (controllerState.throttle - .2);
  }
  
  setControllerState(controllerState);
}
