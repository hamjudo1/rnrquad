#include "PID.h"

/**
 * Loop iteration
 *
 * @param value
 * @return float
 */
float PID::tstep(float value)
{
  float error = target - value;
  float proportional = error;
  float derivative = value - (previous_value ? : value) / target;
  float integral =  (.8 * accumulated_error + .2 * error);

  previous_value = value;
  accumulated_error = integral;

  return (KP * proportional) + (KD * derivative) + (KI * integral);
}

/**
 * Initializes a PID object with given constants
 *
 * @param target_value
 * @param proportional_constant Proportional Constant
 * @param integral_constant Integral Constant
 * @param derivative_constant Derivative Constant
 */
void PID::begin(float target_value, float proportional_constant, float integral_constant, float derivative_constant)
{
  target = target_value;
  KP = proportional_constant;
  KI = integral_constant;
  KD = derivative_constant;
}