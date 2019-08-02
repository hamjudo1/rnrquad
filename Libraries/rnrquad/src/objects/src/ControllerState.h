//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_CONTROLLERSTATE_H
#define RNRQUAD_CONTROLLERSTATE_H

class ControllerState
{
public:
  ControllerState();
  ControllerState(bool isEmpty);
  float throttle;
  float leftStickXPosition;
  float rightStickXPosition;
  float rightStickYPosition;
  float leftTrigger; // increments with each press
  float leftTriggerTime;
  // bool rightTrigger; // not sent
  float trim0;
  float trim1;
  float button7; // increments with each press
  float button7Time;
  float button8; // increments with each press
  float button8Time;
  // TODO buttons
};
extern ControllerState refControl;
#endif
