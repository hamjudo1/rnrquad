//#include "PID.h"
//
//
//void PID::tstep(float value)
//{
//  proportional = (target - value) / target;
//  derivative = value - (previous_value ? : value) / target;
//  integral = value - (previous_value ? : value) / target;
//
//}
//
///**
// * Initializes a PID object with given constants
// *
// * @param target_value
// * @param proportional_constant Proportional Constant
// * @param integral_constant Integral Constant
// * @param derivative_constant Derivative Constant
// */
//void PID::begin(float target_value, float proportional_constant, float integral_constant, float derivative_constant)
//{
//  target = target_value;
//  KP = proportional_constant;
//  KI = integral_constant;
//  KD = derivative_constant;
//}