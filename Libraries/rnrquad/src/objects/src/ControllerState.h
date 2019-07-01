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
  bool leftTrigger;
  bool rightTrigger;
  float trim0;
  float trim1;
  // TODO buttons
};
extern ControllerState refControl;
#endif
