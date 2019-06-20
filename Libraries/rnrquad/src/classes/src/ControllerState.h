//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_CONTROLLERSTATE_H
#define RNRQUAD_CONTROLLERSTATE_H

class ControllerState {
public:
  ControllerState();

  float leftStickYPosition;
  float leftStickXPosition;
  float rightStickXPosition;
  float rightStickYPosition;
};

#endif