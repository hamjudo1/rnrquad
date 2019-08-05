//
// Created by Florian VanKampen on 2019-06-17.
//
#include <rnrquad.h>
// 0.5 at 3.5 volts and above
// 0.6 works around 3.4 volts
// 0.65 at 3.3 volts
// 0.69 at 3.15
// 0.70 at cutoff 3.10 volts
//
//Specify the links and initial tuning parameters
float throttle = 0.0;

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

float udIntegral = 0.0;

float udKI = 0.01;

float udKPuntrim = 1.0;
float udKP = udKPuntrim;
float udKD = 0.5;
float udError = 0.0;
float udOutput = 0.0;
float udTargetUntrim = 0.300;
float udTarget = udTargetUntrim;
float udBaseUntrim = 0.5;
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
simplePID lrPID, dPID, fbPID;


void pidInit()
{
  lrPID.begin("lr", &lrError, &lrOutput, &lrKP, &lrKI, &lrKD);
  // dPID.begin("ud", &udError, &udOutput, &udKP, &udKI, &udKD);
  fbPID.begin("fb", &fbError, &fbOutput, &fbKP, &fbKI, &fbKD);
}

long lastTimeMicros = 0;
float targetAltitude = 0.0;
void udMotion(float dt, float newRx[4])
{
  float tf0, tf1;
  float dv = rangeVel[DOWNRANGE];
  // SensorState sensors = new SensorState();
  float altitudeSeen = refSensor.rangeDown;
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
  // udTarget = constrain(udTargetUntrim + refControl.trim1 * 0.01, 0.0, 1.0);
  udTarget = constrain(targetAltitude, 0.0, 1.0);
  udError = udTarget - altitudeSeen;

  udError = constrain(udError, -0.10, 0.10); // more negative means too high.

  udDerivative = (udError - udPrevError) / dt;
  udDerivative = constrain(udDerivative, -0.20, 0.20);
  if ( okayToFly() &&  flightTime() > 0.1 ) {
    udIntegral += udError * dt;
  } else {
    udIntegral = 0.0;
  }
  udPrevError = udError;
  udKP = udKPuntrim; // + (trim0 - 15.5);
  if (trim0 > 0.5 && trim0 < 30)
  {
    tf0 = (trim0 - 15.5) / 50.0;
  } else
  {
    tf0 = 0.0;
  }
  udBase = udBaseUntrim + refControl.trim0 * 0.01;
  newRx[3] = constrain(udBase + udKP * udError + udKD * udDerivative + udKI * udIntegral, minThrottle, maxThrottle);
}

float lastLrError;

void motion(float newRx[4])
{
  long nowMicros = micros();
  float dt = (nowMicros - lastTimeMicros) / 1000000.0;
  lastTimeMicros = nowMicros;
  udMotion(dt, newRx);
}
extern float vcorrection;

extern float colorAngle;

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
int dir = 0;
SFEVL53L1X distanceSensor;
void setupNew() {
  Serial.begin(115200);
  Wire.begin();
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    delay(800);
  }

  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() == true)
  {
    Serial.println("Sensor online!");
  } else {

    Serial.println("Sensor ?unhappy!");
  }
}

void setup()
{

  baseSetup();
  setupNew();
  addSym(&vcorrection, "vco", "low voltage correction", "3N");
  addSym(&xFlowC, "xf", "optical flow in X (altitude compensated)", "3N");
  addSym(&yFlowC, "yf", "optical flow in Y (altitude compensated)", "3N");
  addSym(&throttle, "t", "throttle", "3N");
  addSym(&targetAltitude, "ta", "targetAltitude", "3N");
  addSym(&udKI, "udki", "updown integral constant", "3N");
  addSym(&udIntegral, "udi", "updown integral", "3N");
  // addSym(&trim0, "trim0", "remote control trim #0", "1N");
  // addSym(&trim1, "trim1", "remote control trim #1", "1N");
  // addSym(&trim1offset, "trim1offset", "constant added to trim1", "1N");
  // addSym(&takeOffPower, "takeOff", "Throttle at Takeoff (estimate)", "3");
  // addSym(&minThrottle, "minThrottle", "Lowest throttle setting during flight", "2");
  // addSym(&maxThrottle, "maxThrottle", "Highest throttle setting during flight", "2");
  // addSym(&spin, "spin", "Positive is CCW", "2");

  addSym(&udBase, "udBase", "up/down base thrust (with trim)", "3");
  addSym(&udBaseUntrim, "udBu", "up down base thrust untrimmed", "3");
  addSym(&udTarget, "udTarget", "target Altitude", "3");
  addSym(&udDerivative, "dv1", "change in altitude", "3");
  addSym(&rangeVel[DOWNRANGE], "dv2", "change in altitude alternate", "3");
  addSym(&udKP, "udKP", "ud proportional", "3");
  addSym(&udError, "udError", "ud error", "3");
  addSym(&lrKP, "lrKP", "lr proportional", "3");
  addSym(&lrError, "lrError", "lr error", "3");
  addSym(&A, "A", "A", "3N");

  tState = 0;
  //turn the PID on
  pidInit();
  // lrTarget = refSensor.rangeLeft;
  //fbTarget = refSensor.rangeForward;
}
void flightLogic() {
  float ft = flightTime();
  if ( ft < 0.1 ) {
    targetAltitude = 0.0; // Propellors to minimum operational speed, not flying but close.
    dir = 0;
  } else if ( ft < 2.6 ) {
    targetAltitude = (ft - 0.1) * 0.2;
    dir = 0;
  } else {
    targetAltitude = 0.5;
    dir = 0;
    // dir = 1 + (int(ft) % 4);
  }
}

void loop() {
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement

  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  refSensor.rangeForward = (float)distance * 0.001;

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();
  delay(2000);

}
void loopOld()
{
  baseLoop();

  // SensorState sensors = refSensor;

  float newRx[4];
  flightLogic();
  xFlowC = refSensor.flowX;
  yFlowC = refSensor.flowY;
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
      if (refSensor.flowQ > 70 )
      {
        newRx[1] = constrain(yFlowC * 0.03, -0.3, +0.3); // newRx Positive is forward thrust
        newRx[0] = constrain(-xFlowC * 0.03, -0.3, +0.3); // Negative is left
      }
    } else {
      rx[0] = rx[0] / 8.0;
      rx[1] = rx[1] / 8.0;
    }
    rx[2] = rx[2] / 4.0;
  } else {
    newRx[3] = 0.0;
  }

  replaceRx(newRx, XN297L_payloadIn[XN297L_goodPayloadIn], XN297L_payloadOut[!XN297L_goodPayloadOut]);
  updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);

  XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
}
