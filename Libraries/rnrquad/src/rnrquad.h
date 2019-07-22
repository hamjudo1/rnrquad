#include "state.h"

void baseSetup();

void baseLoop();

SensorState getSensorState();

ControllerState getControllerState();

void setControllerState(ControllerState controllerState);

void setLED(int ledNumber, RgbLed led);