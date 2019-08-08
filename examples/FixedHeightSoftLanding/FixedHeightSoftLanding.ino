//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

float KP = 0.151;
float KD = 1.9;
float KI = 0.05;
float target = 1;
float minHoverThrottle = .58;
//.55 mHT for 3.7v
//.569 mHT for 3.83v
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

   
  bool throttleWentHigh = false;
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

  setControllerState(controllerState);
}
