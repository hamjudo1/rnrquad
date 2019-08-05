#include <rnrquad.h>

const int avoidDistance = .5; // Meters to stay away from obstacles

void setup()
{
  baseSetup();
}

void loop()
{
  baseLoop();
  ControllerState controllerState = getControllerState();
  SensorState sensorState = getSensorState();

  if (sensorState.left < avoidDistance)
  {
    controllerState.rightStickXPosition = -(sensorState.rangeLeft / avoidDistance);
  }

  if (sensorState.right < avoidDistance)
  {
    controllerState.rightStickXPosition = (sensorState.rangeRight / avoidDistance);
  }

  if (sensorState.up < avoidDistance)
  {
    controllerState.throttle = (sensorState.rangeUp / avoidDistance);//TODO
  }

  if (sensorState.down < avoidDistance)
  {
    controllerState.throttle = -(sensorState.rangeDown / avoidDistance);//TODO
  }

  if (sensorState.forward < avoidDistance)
  {
    controllerState.rightStickYPosition = -(sensorState.rangeForward / avoidDistance);
  }

  setControllerState(controllerState);
}