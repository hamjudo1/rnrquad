//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

const int target_height = .3;
const int proportional_constant = 1;
const int derivative_constant = .5;
const int integral_constant = 0;

// Left - Right, Up - Down, Front - Back PID loops
PID lrPID, udPID, fbPID;

void setup()
{
  baseSetup();

  udPID.begin(target_height, proportional_constant, integral_constant, derivative_constant);
}

void loop()
{
  baseLoop();

  udPID.tstep()
}