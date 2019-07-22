//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>

//Specify the links and initial tuning parameters
float throttle = 0.0;
float notokay = 0.0;
float enableLogging[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float enableLoggingNoComm = 0.0;
float lrTarget = 0;
float csum = 0;
float fbTarget;
float fbKI = 0.000;
float fbKP = 1.000;
float fbKD = 1.0;
float fbError = 0.0;
float fbOutput = 0.0;

float udKI = 0.0000;
float udKPuntrim = 1.0;
float udKP = udKPuntrim;
float udKD = 0.5;
float udError = 0.0;
float udOutput = 0.0;
float udTargetUntrim = 0.300;
float udTarget = udTargetUntrim;
float udBaseUntrim = 0.59;
float udBase = udBaseUntrim;

float lrKI = 0.0000;
float lrKP = 0.3;
float lrKD = 1.0;
float lrError = 0.0;
float lrOutput = 0.0;
float PIDderivative = 0.0;
float PIDintegral = 0.0;

float udPrevError = 0.0;
float udDerivative = 0.0;
float lrDerivative = 0.0;
float fbDerivative = 0.0;
float takeOffPower = 0.55;
float minThrottle = 0.3;
float maxThrottle = 0.9;
float dt = 0.0;
float maxFlights = 2.0;
int lastTState = -1;
float spin = 0.0;
float spinStart = 0;
float takeOffs = 0;
unsigned long timeOfLastExecution = 0;
int launchPhase = 0;

float throttleHigh = 0;
float flightState = 6.0;
float grams = 0.0;
float trim0 = 0.0;
float trim1 = 0.0;
float trim1offset = 0.0;
float xFlowC = 0.0, yFlowC = 0.0;
float undervoltCount = 0.0;
int noFlowNoise = 0;
extern int activeRangeFinderCnt;

extern float tokenVal;

// Left - Right, Up - Down, Front - Back PID loops
simplePID lrPID, udPID, fbPID;

void pidInit()
{
  lrPID.begin("lr", &lrError, &lrOutput, &lrKP, &lrKI, &lrKD);
  udPID.begin("ud", &udError, &udOutput, &udKP, &udKI, &udKD);
  fbPID.begin("fb", &fbError, &fbOutput, &fbKP, &fbKI, &fbKD);
}

bool okayToFly()
{
  static bool throttleDownSinceLastEvent = false;
  if (rx[3] < 0.1)
  {
    throttleDownSinceLastEvent = true;
    notokay = 1.0; // + event;
    return false;
  } else if (!throttleDownSinceLastEvent)
  {
    notokay = 2.0; // + event;
    return false;
  } else if (voltage < 3.1)
  {
    undervoltCount++;
    notokay = 3.0; // + event;
    throttleDownSinceLastEvent = false;
    return false;
  }
  notokay = 0;
  return true;
}

long lastTimeMicros = 0;

void udMotion(float dt, float newRx[4])
{
  float tf0, tf1;
  float dv = rangeVel[DOWNRANGE];
  float altitudeSeen = rangesInM[DOWNRANGE];
  if (altitudeSeen == 0)
  {
    altitudeSeen = 1.3;
  }
  if (trim1 > 0.5 && trim1 < 30)
  {
    tf1 = (15.5 - trim1) / 10.0;
  } else
  {
    tf1 = 0.0;
  }
  udTarget = constrain(udTargetUntrim + tf1, 0.0, 1.0);
  udError = udTarget - altitudeSeen;

  udError = constrain(udError, -0.30, 0.10); // more negative means too high.

  udDerivative = (udError - udPrevError) / dt;
  udDerivative = constrain(udDerivative, -0.20, 0.20);
  udPrevError = udError;
  udKP = udKPuntrim; // + (trim0 - 15.5);
  if (trim0 > 0.5 && trim0 < 30)
  {
    tf0 = (trim0 - 15.5) / 50.0;
  } else
  {
    tf0 = 0.0;
  }
  udBase = udBaseUntrim + tf0;
  newRx[3] = constrain(udBase + udKP * udError + udKD * udDerivative, minThrottle, maxThrottle);


}

float lastLrError;

void motion(float newRx[4])
{
  long nowMicros = micros();
  float dt = (nowMicros - lastTimeMicros) / 1000000.0;
  lastTimeMicros = nowMicros;
  udMotion(dt, newRx);
}

float tenths = 0;
int lastTenths = 0;
long lastTime = 0;
long stateChangeTime = 0;
unsigned long nextLogTime = 0;
bool sawAtLeastOnePacket = false;
long lastPacketTime = 0;
unsigned long testStart = 0;
float rateOfClimb = 0.0;

float prevAltitude = 0.0;

unsigned long prevLoop = 0;

void setup()
{
  baseSetup();

  addSym(&undervoltCount, "uvc", "under voltage Event count", "0N");
  addSym(&xFlowC, "xf", "optical flow in X (altitude compensated)", "3N");
  addSym(&yFlowC, "yf", "optical flow in Y (altitude compensated)", "3N");
  addSym(&throttle, "t", "throttle", "3N");
  addSym(&trim0, "trim0", "remote control trim #0", "1N");
  addSym(&trim1, "trim1", "remote control trim #1", "1N");
  addSym(&trim1offset, "trim1offset", "constant added to trim1", "1N");
  addSym(&takeOffPower, "takeOff", "Throttle at Takeoff (estimate)", "3");
  addSym(&minThrottle, "minThrottle", "Lowest throttle setting during flight", "2");
  addSym(&maxThrottle, "maxThrottle", "Highest throttle setting during flight", "2");
  addSym(&spin, "spin", "Positive is CCW", "2");

  addSym(&udBase, "udBase", "base thrust", "3");
  addSym(&udTarget, "udTarget", "target Altitude", "3");
  addSym(&udDerivative, "dv1", "change in altitude", "3");
  addSym(&rangeVel[DOWNRANGE], "dv2", "change in altitude alternate", "3");
  addSym(&udKP, "udKP", "ud proportional", "3");
  addSym(&udError, "udError", "ud error", "3");
  addSym(&lrKP, "lrKP", "lr proportional", "3");
  addSym(&lrError, "lrError", "lr error", "3");
  addSym(&notokay, "notokay", "reason given by okayToFly() for not flying", "3");

  for (int i = 0; i < 10; i++)
  {
    char *sym = (char *) calloc(strlen("el0") + 1, 1);
    char *desc = (char *) calloc(strlen("FLAG enable logging during state 0") + 1, 1);
    sprintf(sym, "el%d", i);
    sprintf(desc, "FLAG enable logging during state %d", i);
    addSym(&(enableLogging[i]), sym, desc, "F");
  }

  addSym(&enableLoggingNoComm, "eln", "FLAG enable logging when radio down", "F");
  addSym(&flightState, "flightState", "First phase of flight", "0");
  addSym(&maxFlights, "maxFlights", "Max flight segments", "0");
  addSym(&tState, "state", "state machine", "0L");
  addSym(&takeOffs, "takeOffs", "How many flights since boot", "0L");
  addSym(&throttleHigh, "throttleHigh", "Highest throttle value", "4");
  tState = 0;
  //turn the PID on
  pidInit();
  lrTarget = rangesInM[LEFTRANGE];
  fbTarget = rangesInM[FRONTRANGE];
}

void loop()
{
  baseLoop();

  float newRx[4];
  static float xFlow = 0.0, yFlow = 0.0, qFlow = 0.0;


  int sampCnt = getAveFlow(xFlow, yFlow, qFlow);
  unsigned long now = millis();
  unsigned long loopTime = now - prevLoop;
  prevLoop = now;
  if (sampCnt > 0)
  {
    xFlowC = xFlow * (rangesInM[DOWNRANGE] - 0.025);
    yFlowC = yFlow * (rangesInM[DOWNRANGE] - 0.025);
  } else
  {
    xFlowC = xFlowC * 0.8;
    yFlowC = yFlowC * 0.8;
  }

  for (int i = 0; i < 4; i++)
  {
    newRx[i] = rx[i];
  }
  motion(newRx);
  throttle = newRx[3];
  if (okayToFly())
  {


    if (fabs(rx[0]) < 0.1 && fabs(rx[1]) < 0.1)
    {
      // xFlow Positive is traveling backwards, yFlow Positive is moving Left

      if (rangesInM[DOWNRANGE] > 0.15)
      {
        if (sampCnt > 0)
        {
          newRx[1] = constrain(yFlowC * 0.05, -0.3, +0.3); // newRx Positive is forward thrust
          newRx[0] = constrain(-xFlowC * 0.05, -0.3, +0.3); // Negative is left
        }
      }

    }
  } else
  { // under voltage or something
    if (notokay == 3.0 || notokay == 2.0)
    { // active undervoltage
      if (millis() % 666 > 222)
      {
        Led::rgbColorSingleLed(2, 0.0, 0.0, 1.0);
      } else
      {
        Led::rgbColorSingleLed(2, 0.0, 1.0, 0.0);
      }
    }

    newRx[3] = 0;
  }

  if (notokay < 1.5)
  {
    if (millis() % 333 > 222)
    {
      Led::rgbColorSingleLed(2, 0.0, 0.0, 0.0);
    } else
    {

      if (rangesInM[DOWNRANGE] > 0.6)
      {
        Led::rgbColorSingleLed(2, 0.0, 0.0, 0.5);
      } else if (rangesInM[DOWNRANGE] > 0.1)
      {
        Led::rgbColorSingleLed(2, 0.0, 0.5, 0.0);
      } else
      {
        Led::rgbColorSingleLed(2, 0.5, 0.0, 0.0);
      }
    }
  }

  replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);
  if (millis() % 666 > 555)
  {
    Led::rgbColorSingleLed(3, 0.0, 0.0, 0.0);
  } else
  {

    if (noFlowNoise < 2)
    {
      Led::rgbColorSingleLed(3, 0.0, 0.5, 0.0);
    } else if (noFlowNoise < 10)
    {
      Led::rgbColorSingleLed(3, 0.5, 0.5, 0.0);
    } else
    {
      Led::rgbColorSingleLed(3, 0.7, 0.7, 0.7);
    }
  }
  XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
}