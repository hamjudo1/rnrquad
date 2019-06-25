#include "PID.h"

void simplePID::tstep(float dt)
{
  PIDintegral += *PIDerror * dt; //multiply error by time interval since last reading
  PIDderivative = (*PIDerror - prevError) / dt; //divide rise over run - dY/dt
  prevError = *PIDerror;
  *PIDoutput = (*KP) * (*PIDerror) + (*KI) * PIDintegral + (*KD) * PIDderivative;
}

void simplePID::begin(char *prefix, float *pError, float *pOutput, float *pKP, float *pKI, float *pKD)
{
  PIDerror = pError;
  PIDoutput = pOutput;
  KP = pKP;
  KI = pKI;
  KD = pKD;
//  addSym(PIDerror, catString2(prefix, "_error"), "PID error", "3L");
//  addSym(PIDoutput, catString2(prefix, "_output"), "PID output", "3L");
//  addSym(&PIDintegral, catString2(prefix, "_integral"), "PID integral", "3L");
//  addSym(&PIDderivative, catString2(prefix, "_derivative"), "PID derivative", "3L");
//  addSym(KP, catString2(prefix, "_KP"), "PID KP", "3L");
//  addSym(KI, catString2(prefix, "_KI"), "PID KI", "3L");
//  addSym(KD, catString2(prefix, "_KD"), "PID KD", "3L");
  PIDintegral = 0.0;
  PIDderivative = 0.0;
  prevError = 0.0;
}