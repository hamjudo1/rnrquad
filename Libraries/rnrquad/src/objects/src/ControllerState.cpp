//
// Created by Florian VanKampen on 2019-06-17.
//
#include "state.h"
#include "ControllerState.h"

ControllerState::ControllerState(bool isEmpty)
{
  throttle = 0;
  leftStickXPosition = 0;
  rightStickXPosition = 0;
  rightStickYPosition = 0;
  trim0 = 0;
  trim1 = 0;
}
ControllerState refControl=ControllerState(true);

ControllerState::ControllerState()
{
  throttle = refControl.throttle;
  leftStickXPosition = refControl.leftStickXPosition;
  rightStickXPosition = refControl.rightStickXPosition;
  rightStickYPosition = refControl.rightStickYPosition;
  trim0 = refControl.trim0;
  trim1 = refControl.trim1;
  
}
