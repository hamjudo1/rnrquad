#ifndef RNRQUAD_OBJECTS_PID_H
#define RNRQUAD_OBJECTS_PID_H

class simplePID
{
public:
  float PIDintegral;
  float PIDderivative;
  float prevError;
  float *PIDoutput;
  float *PIDerror;
  float *KD;
  float *KI;
  float *KP;
  char *prefix;

  void begin(char *prefix, float *pError, float *pOutput, float *pKP, float *pKI, float *pKD);

  void tstep(float dt);

  void setLogging(bool On);
};

class PID
{
public:
  float target;
  float KP;
  float KI;
  float KD;
  float previousError;
  float C;

  void begin(float targetValue, float proportionalConstant, float integralConstant, float derivativeConstant, float constant);
  float tstep(float value);
  bool altitudeMode;
  void reset();
private:
  float accumulatedError;
  float previousTime;
};

#endif
