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
  float previous_value;
  float accumulated_error;

  void begin(float target_value, float proportional_constant, float integral_constant, float derivative_constant);
  float tstep(float value);
};