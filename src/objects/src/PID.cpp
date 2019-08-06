#include "PID.h"
#include "state.h"


/**
 * Loop iteration
 *
 * @param value
 * @return float
 */
float PID::tstep(float value)
{
  float time = Utility::time();
  float error = constrain (target - value, -.3, .1); //Error should never be wildly high
  float proportional = error;
  float derivative = (error - previousError) / (time - previousTime);
  float integral =  (.9 * accumulatedError + .1 * error);


  derivative = constrain(derivative, -.2, .2);
  integral = constrain(integral, -.1, .1);

  previousTime = time;
  previousError = error;
  accumulatedError = integral;

  return C + (KP * proportional) + (KD * derivative) + (KI * integral);
}

/**
 * Initializes a PID object with given constants
 *
 * @param targetValue
 * @param proportionalConstant Proportional Constant
 * @param integralConstant Integral Constant
 * @param derivativeConstant Derivative Constant
 */
void PID::begin(float targetValue, float proportionalConstant, float integralConstant, float derivativeConstant, float constant)
{
  target = targetValue;
  KP = proportionalConstant;
  KI = integralConstant;
  KD = derivativeConstant;
  previousError = -target;
  previousTime = Utility::time();
  C = constant;
}