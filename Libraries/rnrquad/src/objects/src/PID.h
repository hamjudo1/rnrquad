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

//class PID
//{
//public:
//  float PIDintegral;
//  float PIDderivative;
//  float prevError;
//  float *PIDoutput;
//  float *PIDerror;
//  float *KD;
//  float *KI;
//  float *KP;
//  char *prefix;
//
//  void begin(char *prefix, float *pError, float *pOutput, float *pKP, float *pKI, float *pKD);
//
//  void tstep(float dt);
//
//  void setLogging(bool On);
//};