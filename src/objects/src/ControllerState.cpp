//
// Created by Florian VanKampen on 2019-06-17.
//
#include "state.h"
#include "ControllerState.h"

// Pointless double constructor so it's possible to make all ControllerState objects clones of the reference controller
ControllerState::ControllerState(bool isEmpty)
{
  throttle = 0;
  leftStickXPosition = 0;
  rightStickXPosition = 0;
  rightStickYPosition = 0;
  leftTrigger = 0;
  leftTriggerTime = 0;
  trim0 = 0;
  trim1 = 0;
  button7 = 0;
  button7Time = 0;
  button8 = 0;
  button8Time = 0;
}
ControllerState refControl=ControllerState(true);

ControllerState::ControllerState()
{
  throttle = refControl.throttle;
  leftStickXPosition = refControl.leftStickXPosition;
  rightStickXPosition = refControl.rightStickXPosition;
  rightStickYPosition = refControl.rightStickYPosition;
  leftTrigger = refControl.leftTrigger;
  leftTriggerTime = refControl.leftTriggerTime;
  button7 = refControl.button7;
  button7Time = refControl.button7Time;
  button8 = refControl.button8;
  button8Time = refControl.button8Time;
  trim0 = refControl.trim0;
  trim1 = refControl.trim1;
}
