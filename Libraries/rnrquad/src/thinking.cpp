
#include "state.h"

//Specify the links and initial tuning parameters
float throttle = 0.0;
float notokay = 0.0;
float enableLogging[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
boolean ktMode = false;
float fbSpeed = 0.0;
float fbBase = 0.00;
float lrBase = 0.10;
float lrSpeed = 0;
float overrideGrams = 0.0;
float launchHeight;
float enableLoggingNoComm = 0.0;
float lrTarget = 0;
float csum = 0;
// float udTarget;
float fbTarget;
// fb: forward-backward, ud: up-down, lr: left-right
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

float PIDtarget = 0.0;
float udPrevError = 0.0;
float fbPrevError = 0.0;
float udDerivative = 0.0;
float lrPrevError = 0.0;
float lrDerivative = 0.0;
float fbDerivative = 0.0;
float takeOffPower = 0.55;
float takeOffPowerActual[5];
float minThrottle = 0.3;
// float maxThrottle = constrain(takeOffPower + 0.2, 0.0, 1.0);
float maxThrottle = 0.9;
float dt = 0.0;
float maxFlights = 2.0;
int lastTState = -1;
float spin = 0.0;
float spinStart = 0;
float takeOffs = 0;
unsigned long timeOfLastExecution = 0;
int launchPhase = 0;

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
  addSym(PIDerror, catString2(prefix, "_error"), "PID error", "3L");
  addSym(PIDoutput, catString2(prefix, "_output"), "PID output", "3L");
  addSym(&PIDintegral, catString2(prefix, "_integral"), "PID integral", "3L");
  addSym(&PIDderivative, catString2(prefix, "_output"), "PID derivative", "3L");
  addSym(KP, catString2(prefix, "_KP"), "PID KP", "3L");
  addSym(KI, catString2(prefix, "_KI"), "PID KI", "3L");
  addSym(KD, catString2(prefix, "_KD"), "PID KD", "3L");
  PIDintegral = 0.0;
  PIDderivative = 0.0;
  prevError = 0.0;
}

simplePID lrPID, udPID, fbPID;

void pidInit()
{
  lrPID.begin("lr", &lrError, &lrOutput, &lrKP, &lrKI, &lrKD);
  udPID.begin("ud", &udError, &udOutput, &udKP, &udKI, &udKD);
  fbPID.begin("fb", &fbError, &fbOutput, &fbKP, &fbKI, &fbKD);
}

void pidLoop()
{
  unsigned long now = millis();
  unsigned long dtInMilliSec = now - timeOfLastExecution;
  long timeOfLastExecution = now;
  dt = (float) (dtInMilliSec) / 1000.0; // Lets use seconds, to help maintain sanity.
  lrPID.tstep(dt);
  udPID.tstep(dt);
  fbPID.tstep(dt);
  /*  PIDintegral += PIDerror * dt; //multiply error by time interval since last reading
    PIDderivative = (PIDerror - prevError) / dt; //divide rise over run - dY/dt
    prevError = PIDerror;
    PIDoutput = KP * PIDerror + KI * PIDintegral + KD * PIDderivative; //multiply by gains */
}

/* void goToAltitude(float targetAlt) {
  PIDtarget = targetAlt;
  PIDerror = targetAlt - rangesInM[DOWNRANGE];
  PIDerror = constrain(PIDerror, -0.1, 0.1);
  if ( (PIDerror < 0.003) && (PIDerror > -0.003) ) {
    PIDerror = 0.0;
  }
  pidLoop();
  throttle = constrain(takeOffPowerActual[2] + PIDoutput, minThrottle, maxThrottle);
  } */
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

void setupThinking()
{
  for (int k = 0; k < 5; k++)
  {
    takeOffPowerActual[k] = 0;
  }
  addSym(&undervoltCount, "uvc", "under voltage Event count", "0N");
  addSym(&xFlowC, "xf", "optical flow in X (altitude compensated)", "3N");
  addSym(&yFlowC, "yf", "optical flow in Y (altitude compensated)", "3N");
  //addSym(&KP, "KP", "PID - Proportional constant", "5");
  // addSym(&KI, "KI", "PID - Integral constant", "5");
  // addSym(&KD, "KD", "PID - Differential constant", "5");
  // addSym(&PIDerror, "error", "PID - error", "5L");
  // addSym(&PIDintegral, "integral", "PID - integral", "5L");
  // addSym(&PIDderivative, "derivative", "PID - derivative", "8L");
  // addSym(&PIDoutput, "output", "PID - output", "5L");
  // addSym(&PIDtarget, "target", "PID - target", "3L");
  addSym(&throttle, "t", "throttle", "3N");
  // addSym(&grams, "g", "throttle", "3N" );
  addSym(&trim0, "trim0", "remote control trim #0", "1N");
  addSym(&trim1, "trim1", "remote control trim #1", "1N");
  addSym(&trim1offset, "trim1offset", "constant added to trim1", "1N");
  addSym(&takeOffPower, "takeOff", "Throttle at Takeoff (estimate)", "3");
  addSym(&minThrottle, "minThrottle", "Lowest throttle setting during flight", "2");
  addSym(&maxThrottle, "maxThrottle", "Highest throttle setting during flight", "2");
  addSym(&spin, "spin", "Positive is CCW", "2");

  // addSym(&spinStart, "spinStart", "time in millis when we start spinning", "0");
  // addSym(&takeOffPowerActual[0], "takeoff T 0", "Throttle when left ground with rapid accel", "3");
  // addSym(&takeOffPowerActual[1], "takeoff T 1", "Throttle when left ground with medium accel", "3");
  // addSym(&takeOffPowerActual[2], "takeoff T 2", "Throttle when left ground with slow accel", "3");
  // addSym(&lrSpeed, "lrt", "left throttle", "3");
  // addSym(&fbSpeed, "fbt", "forward throttle", "3");
  addSym(&udBase, "udBase", "base thrust", "3");
  addSym(&udTarget, "udTarget", "target Altitude", "3");
  addSym(&udDerivative, "dv1", "change in altitude", "3");
  addSym(&rangeVel[DOWNRANGE], "dv2", "change in altitude alternate", "3");
  addSym(&udKP, "udKP", "ud proportional", "3");
  addSym(&udError, "udError", "ud error", "3");
  addSym(&lrKP, "lrKP", "lr proportional", "3");
  addSym(&lrError, "lrError", "lr error", "3");
  addSym(&notokay, "notokay", "reason given by okayToFly() for not flying", "3");
  //addSym(&csum, "csum", "Packet checksum", 0);

  // addSym((float*)&inp, "input", "PID - Input", "0L");
  // addSym((float*)&outp, "output", "PID - Output", "4L");
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
  //addSym(&battVoltage, "b", "battVoltage", "2L" );
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

long onPlatformStartTime = 0;

void offPlatform()
{
  onPlatformStartTime = 0;
}

boolean onPlatform()
{
  long now = millis();
  // if ( rangesInM[DOWNRANGE] > 0.060 && rangesInM[DOWNRANGE] < 0.120 ) {
  if (rangesInM[DOWNRANGE] < 0.50)
  {
    if (onPlatformStartTime > 0)
    {
      if (onPlatformStartTime + 1000 < now)
      {
        return true;
      }
    } else
    {
      onPlatformStartTime = now;
    }
  } else
  {
    onPlatformStartTime = 0;
  }
  return false;
}

void translate()
{
  if (rangesInM[FRONTRANGE] > 0.3)
  {
    fbSpeed += 0.002;
  } else if (rangesInM[FRONTRANGE] < 0.2)
  {
    fbSpeed -= 0.002;
  }
  if (rangesInM[LEFTRANGE] > 0.3)
  {
    lrSpeed += 0.002;

  } else if (rangesInM[LEFTRANGE] < 0.2)
  {
    lrSpeed -= 0.002;
  }
}

extern float tokenVal;

void keyboardThrottle()
{
  if (tokenVal >= 0.0)
  {
    ktMode = true;
    overrideGrams = tokenVal;
    Serial.print("overriding Throttle to ");
    Serial.println(overrideGrams, 1);
  } else
  {
    Serial.println("Leaving throttle override mode");
    ktMode = false;
  }
}

float maxAltitudeSeen = 0.0;
float event = 0.0;
float estoprange = 1.0;

boolean okayToFly()
{
  static boolean throttleDownSinceLastEvent = false;
  if (rx[3] < 0.1)
  {
    throttleDownSinceLastEvent = true;
    maxAltitudeSeen = 0.0;
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
  /* if ( estoprange ) {
     for (int i = 1; i < 5; i++) {
       if ( i != DOWNRANGE ) {
         if ( rangesInM[i] < 0.10 && rangesInM[i] != 0.0 ) {  // spurious range zero sightings on right sensor.
           throttleDownSinceLastEvent = false;
           event = 0.1 * (float)i;
           return false;
         }
       }
     }
    } */
  event = 0.0;
  notokay = 0;
  return true;
}

float gramsToThrottle(float g)
{
  grams = g;
  if (!okayToFly())
  {
    return 0.0;
  } else
  {
    //return mapf(g, 27.0, 33.0, 0.45, 0.6);
    return g / 80.0; // Multiplied by 20 for each motor in the brainstem.
  }
}

//  PIDintegral += *PIDerror * dt; //multiply error by time interval since last reading
//  PIDderivative = (*PIDerror - prevError) / dt; //divide rise over run - dY/dt
//  prevError = *PIDerror;
//  *PIDoutput = (*KP) * (*PIDerror) + (*KI) * PIDintegral + (*KD) * PIDderivative;
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

void lrMotion(float dt)
{
  float lDist = constrain(rangesInM[LEFTRANGE], 0.0, 0.6);
  float rDist = constrain(rangesInM[RIGHTRANGE], 0.0, 0.6);
  lrError = (rDist - lDist);
  lrDerivative = (lrError - lrPrevError) / dt;
  lrPrevError = lrError;
  lrSpeed = lrError * lrKP + lrKD * lrDerivative;
  lrSpeed = constrain(lrSpeed, -0.4, 0.4);

}

void fbMotion(float dt)
{
  float fDist = constrain(rangesInM[FRONTRANGE], 0.0, 1.0);

  fbError = (fDist - 0.50);
  fbDerivative = (fbError - fbPrevError) / dt;
  fbPrevError = fbError;
  fbSpeed = fbError * fbKP + fbKD * fbDerivative;
  fbSpeed = constrain(fbSpeed, -0.4, 0.4);

}

void motion(float newRx[4])
{
  long nowMicros = micros();
  float dt = (nowMicros - lastTimeMicros) / 1000000.0;
  lastTimeMicros = nowMicros;
  udMotion(dt, newRx);
  // lrMotion(dt);
  // fbMotion(dt);
}

/*
  void planMotion(float udTarget, float lrTarget, float fbTarget) {
  udError = udTarget - rangesInM[DOWNRANGE];
  udError = constrain(udError, -0.1, 0.1);
  lrError = lrTarget - rangesInM[LEFTRANGE];
  fbError = fbTarget - rangesInM[FRONTRANGE];
  //if ( (udPIDerror < 0.003) && (udPIDerror > -0.003) ) {
  //  udPIDerror = 0.0;
  // }
  pidLoop();
  throttle = constrain(takeOffPowerActual[2] + udOutput, minThrottle, maxThrottle);
  }
*/
extern float colorAngle;

float tenths = 0;
int lastTenths = 0;
long lastTime = 0;
long stateChangeTime = 0;
unsigned long nextLogTime = 0;
boolean sawAtLeastOnePacket = false;
long lastPacketTime = 0;
unsigned long testStart = 0;
float rateOfClimb = 0.0;

float prevAltitude = 0.0;

float removeDeadzone(float val)
{
  /*if (val > 0.01) {
    val += 0.1;
    } else if ( val < -0.01) {
    val -= 0.1;
    } */
  return val;
}

unsigned long prevLoop = 0;

void pollThinking()
{
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


  //
  /*  if ( loopTime > 10 && loopTime < 100 ) {
      rateOfClimb = (dw - prevAltitude) / (0.001 * loopTime);
      if ( rateOfClimb > 0.1  ) {
        rgbSingleDot1(2, 0, 0, 1.0);
      } else if ( rateOfClimb < -0.1 ) {
        rgbSingleDot1(2, 1.0, 0, 0);
      } else {  //
        rgbSingleDot1(2, 0, 1, 0);
      }
    } */

  /* Serial.print(dw, 3);
    Serial.print(", ");
    Serial.println(rateOfClimb, 5); */
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
          newRx[1] = constrain(removeDeadzone(yFlowC * 0.05), -0.3, +0.3); // newRx Positive is forward thrust
          newRx[0] = constrain(removeDeadzone(-xFlowC * 0.05), -0.3, +0.3); // Negative is left
        }
      }

    }
    /*
      float factor = 1.0;
      if ( dv > 0.01 && dw > 0.1 ) {
      factor = 1.0 - (0.9 * (dw - 0.1)) ;
      factor = constrain(factor, 0, 1.0);
      }
      newRx[3] = rx[3] * factor;
      newRx[3] = constrain(rx[3] * factor, 0.1, 1.0);
    */

#if 0
    Serial.print(sampCnt);
    Serial.print(" Raw flow (");
    Serial.print(xFlow, 0);
    Serial.print(",");
    Serial.print(yFlow, 0);
    Serial.print(") newRx: (");
    Serial.print(newRx[1], 5);
    Serial.print(", ");
    Serial.print(newRx[0], 5);
    Serial.print("), q:");
    Serial.print(qFlow, 0);
    Serial.print("  ");
    Serial.print(dw);

    Serial.print("  throttle:");
    Serial.println(newRx[3], 5);
#endif
    //  Serial.println(millis());
    //  }

  } else
  { // under voltage or something
    if (notokay == 3.0 || notokay == 2.0)
    { // active undervoltage
      if (millis() % 666 > 222)
      {
        rgbSingleDot1(2, 0.0, 0.0, 1.0);
      } else
      {
        rgbSingleDot1(2, 0.0, 1.0, 0.0);
      }
    }

    newRx[3] = 0;
  }
  if (notokay < 1.5)
  {
    if (millis() % 333 > 222)
    {
      rgbSingleDot1(2, 0.0, 0.0, 0.0);
    } else
    {

      if (rangesInM[DOWNRANGE] > 0.6)
      {
        rgbSingleDot1(2, 0.0, 0.0, 0.5);
      } else if (rangesInM[DOWNRANGE] > 0.1)
      {
        rgbSingleDot1(2, 0.0, 0.5, 0.0);
      } else
      {
        rgbSingleDot1(2, 0.5, 0.0, 0.0);
      }
    }

  }
  replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);
  if (millis() % 666 > 555)
  {
    rgbSingleDot1(3, 0.0, 0.0, 0.0);
  } else
  {

    if (noFlowNoise < 2)
    {
      rgbSingleDot1(3, 0.0, 0.5, 0.0);
    } else if (noFlowNoise < 10)
    {
      rgbSingleDot1(3, 0.5, 0.5, 0.0);
    } else
    {
      rgbSingleDot1(3, 0.7, 0.7, 0.7);
    }
  }
  XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
}

#if 0
void pollThinkingOld() {
  float newRx[4];
  unsigned long now = millis();

  long timeInState = now - stateChangeTime;
  motion();

  if ( unreadPacket ) {
    lastPacketTime = now;
    unreadPacket = false;
    if ( ! sawAtLeastOnePacket ) {
      launchHeight = 0.01 + rangeHistTotal[DOWNRANGE] / HISTDEPTH;
      // launchHeight = rangesInM[DOWNRANGE] + 0.01; // Add a centimeter to launch height so we start out going up.
      sawAtLeastOnePacket = true;
      tState = 1;
      // throttle = minThrottle;
      throttle = 0.0;
    }
  }
  for (int i = 0; i < 4; i++) {
    newRx[i] = rx[i];
  }
  if ( rx[3] < 0.1 ) {
    if ( showVerbose ) {
      Serial.println("Remote throttle off!! Land now");
    }
    tState = 9;
    throttle = 0.0;
  }

  if ( sawAtLeastOnePacket && (now - lastPacketTime) > 100 )   {
    if ( showVerbose ) {
      Serial.println("Lost contact with remote!");
    }
    tState = 9;
    throttle = 0.0;
  }
  if ( sawAtLeastOnePacket ) {
    if ( byPassManipulation ) {
      if ( showVerbose ) {
        Serial.println("Bypass mode.");
      }
    } else {
      if ( showVerbose ) {
        Serial.print(tState, 0);
        Serial.print(", ");
        Serial.print(throttle, 4);
        Serial.print(" ");
        Serial.print(now - lastPacketTime);
        Serial.print(": ");
        Serial.println(rangesInM[DOWNRANGE]);
      }
      lastTime = now;
      int newVal;
      long tTime = now - testStart;
      long fifths = (tTime / 1000) % 11;
      //  throttle = rx[3];
      motion();

      // grams = (int)(throttle * 80.0);
      // throttle = grams / 80.0;

      showSmallInt(grams);
      /*
        switch ((int) tState ) {
        case 0:
          tenths = fifths * 0.1;
          if ( throttle != tenths ) {
            throttle = tenths;
            tState = 1;
          }
          break;
        case 1:
          newVal = (int)(throttle * 10 + 0.5);
          colorSingleDot(2, hsvColor(throttle * 330., 1.0, 1.0));
          if ( newVal & 1 ) {
            colorSingleDot(3, 0x000);
            colorSingleDot(4, 0x202020);
          } else {
            colorSingleDot(4, 0x000);
            colorSingleDot(3, 0x202020);
          }
          tState = 0;
          break;
        case 2:
          break;
        case 3:
          break;
        case 4:
          break;
        case 5:
          break;
        case 6:
          break;
        case 7:
          break;
        case 8:
          break;
        case 9:
          throttle = 0.0;
          if ( (now - lastPacketTime) < 100 && rx[3] < 0.1 ) {
            tState = 1;
            testStart = now;
          }
          break;
        default:
          break;
        }
        }
      */
      /*     if ( tState == 0 ) { // wait to initialize.
             throttle = takeOffPower - 0.05;
             if ( onPlatform() ) {

               tState = flightState;
               throttle = takeOffPower;
               takeOffs += 1;
             } else {
               if ( showVerbose ) {
                 if ( timeInState < 1000 ) {
                   Serial.println(" Not time for flight");
                 } else {
                   Serial.println(" Too high for takeoff");
                 }
               }
             }
           } else if ( tState == 1 ) { // Pre flight checklist
             // Check battery voltage

             // Check for valid connection from remote
             throttle = rx[3];
             // Check that downward rangefinder is working
             if ( rangesInM[DOWNRANGE] > 0.100 && rangesInM[DOWNRANGE]  < 0.200 ) {
               tState = 2;
             } else {
               if ( showVerbose ) {
                 Serial.print(" Wait for good altitude ");
               }
             }
           } else if ( tState == 2 ) {
             //translate();
             planMotion(udTarget, lrTarget, fbTarget);
             if ( timeInState < 300 ) {
               throttle = takeOffPowerActual[launchPhase - 1] - 0.2;
             } else if ( timeInState < 600 ) {
               throttle = takeOffPowerActual[launchPhase - 1] - 0.1;
             } else if ( rangesInM[DOWNRANGE] < launchHeight ) {
               if ( launchPhase > 2 ) {
                 tState = 6;
               } else {
                 tState = 5;
               }
             }
           } else if ( tState == 3 ) {
             throttle = throttle - 0.05;
             if ( throttle < 0.10 ) {
               throttle = 0.0;
               tState = 4;
             }

           } else if (tState == 4 ) {
             if ( rx[3] < 0.1 ) {
               throttle = 0.0;
             } else if (takeOffs >= maxFlights ) {
               tState = 9;
               throttle = 0.0;
             } else if (  onPlatform () ) {
               tState = 5;
               launchPhase = 0;
             } else {
               throttle = minThrottle;
             }
           } else if (tState == 5 ) { // Determine takeoff thrust.
             if ( rangesInM[DOWNRANGE] > launchHeight + 0.02 ) {
               tState = 2;
               takeOffPowerActual[launchPhase] = throttle;
               throttle = throttle - 0.1;
               launchPhase++;
             } else {
               if ( launchPhase == 0 ) {
                 throttle += 0.05;
               } else if ( launchPhase == 1 ) {
                 throttle += 0.002;
               } else if ( launchPhase == 2 ) {
                 throttle += 0.001;
               }
               if ( throttle > throttleHigh ) {
                 throttleHigh = throttle;
               }
             }
           } else if (tState == 6 ) { // climb tenth of a meter in 2 seconds switch to state 7 (hold altitude)
             float stateTime = timeInState / 1000.0;
             udTarget = launchHeight + stateTime * 0.050; // climb at a 50mm per second.
             planMotion(udTarget, lrTarget, fbTarget);
             if ( rangesInM[DOWNRANGE] > (0.02 + launchHeight) ) {
               spin = 0.4; // Start spinning.
               spinStart = (float) now;
             }
             if ( udTarget > (0.200 + launchHeight) ) {
               udTarget = 0.200 + launchHeight;
               tState = 7;
             }
           } else if (tState == 7 ) {
             // goToAltitude(launchHeight + 0.200);
             planMotion(udTarget, lrTarget, fbTarget);
             if ( timeInState > 3000 ) {
               tState = 8;
             }
           } else if (tState == 8 ) {
             float stateTime = timeInState / 1000.0;
             float udTarget = launchHeight + 0.100 - stateTime * 0.050;
             planMotion(udTarget, lrTarget, fbTarget);
             if ( rangesInM[DOWNRANGE] < (launchHeight - 0.03 )) {
               tState = 4;
             }
           } else if (tState == 9 ) {
             throttle = 0;
           } */
    }

    //  if ( throttle != 0 ) {
    //    throttle = constrain(throttle, minThrottle, maxThrottle);
    //  }
    newRx[3] = throttle;
    newRx[2] = spin; // Twist CCW
    newRx[1] = fbSpeed + fbBase; // Positive is forward
    newRx[0] = lrSpeed + lrBase; // Negative is left
    colorAngle = throttle * 360.0;

    // replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut] );
    // updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);
    // csum = (int)XN297L_payloadOut[!XN297L_goodPayloadOut][14];
    // XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
    if ( tState != lastTState ) {
      if ( tState == 4 ) {
        offPlatform();
      }
      stateChangeTime = now;
      lastTState = tState;
      logSyms();

    } else if ( enableLogging[(int)tState] ) {
      logSyms();

    }
  } else {
    if ( enableLoggingNoComm ) {
      if ( now > nextLogTime ) {
        logSyms();
        nextLogTime = now + 50;
      }
    }
    if ( showVerbose ) {
      Serial.println("no packet yet");
    }
  }
}
#endif



