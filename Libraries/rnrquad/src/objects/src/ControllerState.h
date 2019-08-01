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
  float leftTrigger; // 0, 1 ,2
  // bool rightTrigger; // not sent
  float trim0;
  float trim1;
  float button7;
  float button8;
  // TODO buttons
};
extern ControllerState refControl;
#endif
