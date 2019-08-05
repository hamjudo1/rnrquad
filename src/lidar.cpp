#include "state.h"

// SFEVL53L1X loxes[RFINDERS] = {SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X()};
//SFEVL53L1X front;
//SFEVL53L1X downR;
//SFEVL53L1X right;
//SFEVL53L1X left;
//SFEVL53L1X up;

// J5 1 CPU pin 12 Port PA07 Arduino Pin 9.
// J5 2 CPU pin 11 Port PA06 Arduino Pin 8.
// J5 3 CPU pin 10 Port PA05 Arduino Pin 18.
// J5 4 CPU pin 9 Port PA04 Arduino Pin 17.
// J5 5 CPU pin 8 Port PB09 Arduino Pin 16.
const int J5X1 = 9;
const int J5X2 = 8;
const int J5X3 = 18;
const int J5X4 = 17;
const int J5X5 = 16;

// J5 connects the range finders and NEO pixels.
// J5X1 Yellow to “UP”, Arduino 9
// J5X2 White to “LEFT”, Arduino 8
// J5X3 Blue to “DOWN”, Arduino 18
// J5X4 Green to “FRONT”, Arduino 17
// J5X5 Blue to Neopixel array, Arduino 16
// xshut on "right" range finder is not connected.

rangeConfigElem_t rangeConfig[] = {
    {-1, true, "front", 0},
    {7,  true, "down",  0},
    {0,  true, "right", 0},
    {19, true, "left",  0},
    {16, true, "up",    0},
};
bool I2C_Would_Hang(int SDAPin, int SCLPin) {
  bool wouldHang = false;
  pinMode(SDAPin,INPUT);
  pinMode(SCLPin,INPUT);
  if ( digitalRead(SDAPin) != 1 ) {
    Serial.println("SDA pin is low");
    wouldHang = true;
  }
  if ( digitalRead(SCLPin) != 1 ) {
    Serial.println("SCL pin is low");
    wouldHang = true;
  }
  return wouldHang;
}
int activeRangeFinderCnt = 0;
int activeRF[5];
const char *allNames[5];

bool initRangeFinderWRetries(int i)
{
  int retryCount = 0;
  float *errorP;
  while (retryCount < 5)
  {
    Serial.print(__FILE__);Serial.println(__LINE__);
    bool success = false;
    // bool success = downR.begin();
    // bool success = loxes[activeRangeFinderCnt].begin( );
    Serial.print(__FILE__);Serial.println(__LINE__);
    // downR.setI2CAddress(1 + (i * 2));
    // loxes[activeRangeFinderCnt].setI2CAddress(1 + (i * 2));
    Serial.print(__FILE__);Serial.println(__LINE__);
    Serial.print(allNames[i]);
    Serial.print(" ");
    Serial.print(i);
    if (success)
    {
      Serial.println(" range finder started normally");

      return true;
    }
    retryCount++;
    if (retryCount == 1)
    {
      errorP = &(errorList[errorListIndex++]);
      addSym(errorP, "RETRIES", "Range Finder startup glitch", "1");
    }
    *errorP = i + 0.1 * (float) retryCount;
    Serial.print(" range finder start attempt ");
    Serial.print(retryCount);
    Serial.println(" Failed");
  }
  Serial.print(allNames[i]);
  Serial.println(" would not start giving up after many tries.");
  addSym(errorP, "FATAL", "RangeFinder would not start after repeated retries.", "0");
  return false;
}

  // moved to independent function, because even dead range finders get names.
void rangeFinderSyms() {
  for (int i = 0; i < RFINDERS; i++)
  {
    allNames[i] = rangeConfig[i].Name;
    addSym(&(rangesInM[i]), allNames[i], "range finder", "3N");
  }
}
bool testSettingXshut(int pinNo, int val, const char* name) {
  bool failed = false;
  pinMode(pinNo, OUTPUT);
  digitalWrite(pinNo, val);
  if ( I2C_Would_Hang(20,21) ) {
    failed = true;
    Serial.print("I2C Would Hang xshut pulled to ");
    Serial.print(val);
    Serial.print("low on ");
    Serial.println(name);
  }
  pinMode(pinNo, INPUT);
  return failed;
}
void disableAllRangeFinders() {
  i2cBusSafe = false;
  for (int i=0; i < RFINDERS; i++ ) {
    rangeConfig[i].enabled = false;
  }
}

// range finders one by one to see if any of them interfere with the I2C bus.
void testRangeFindersI2Csafe() {
  i2cBusSafe = true;
  if ( I2C_Would_Hang(20,21) ) {
    Serial.println("I2C bus is permanently broken even before xshut manipulation");
    disableAllRangeFinders();
    return;
  }
  
  for (int i=0; i < RFINDERS; i++ ) {
    int pinNo = rangeConfig[i].j5Index;
    if (pinNo >= 0 ) {                 // Always set the xshut pin to low output, even for disabled range finders. Or they block the bus.
      bool lowFailed = testSettingXshut(pinNo,LOW,allNames[i]);
      bool highFailed = testSettingXshut(pinNo,HIGH,allNames[i]);
      if ( lowFailed || highFailed ) {
        rangeConfig[i].enabled = false;
      }
    }
  }
  if ( I2C_Would_Hang(20,21) ) {
    Serial.println("I2C bus is permanently broken ");
    i2cBusSafe = false;
    disableAllRangeFinders();
  }
}
void setupRangeFinders()
{
  int i;
  rangeFinderSyms();
  if ( I2C_Would_Hang(20,21) ) {
    i2cBusSafe = false;
    return;
  } else {
    i2cBusSafe = true;
  }
  Serial.print(__FILE__);Serial.println(__LINE__);
     
  // Setup I2C
  Wire.begin();
  Serial.print(__FILE__);Serial.println(__LINE__);
  int aPinNo;
  for (i = 0; i < RFINDERS; i++)
  {
    rangesInM[i] = 0.0;
    lastR[i] = 0;
    for (int k = 0; k < HISTDEPTH; k++)
    {
      rangeHist[i][k] = 0.0;
    }
    rangeHistTotal[i] = 0.0;
    aPinNo = rangeConfig[i].j5Index;
    if (aPinNo >= 0)
    {                 // Always set the xshut pin to low output, even for disabled range finders. Or they block the bus.
      pinMode(aPinNo, OUTPUT);
      digitalWrite(aPinNo, LOW);
    }
  }
  for (i = 0; i < RFINDERS; i++)
  {
    allNames[i] = rangeConfig[i].Name;
    if (rangeConfig[i].enabled)
    {
      /*
      Serial.print("quadRange active count ");
      Serial.print(activeRangeFinderCnt);
      Serial.print(" i ");
      Serial.println(i);
      */
      aPinNo = rangeConfig[i].j5Index;
      if (aPinNo >= 0)
      {
      
        Serial.print("Setting Arduino Pin ");
        Serial.print(aPinNo);
        Serial.print(" HIGH for ");
        Serial.println(allNames[i]);
     
        digitalWrite(aPinNo, HIGH);
      }
      else
      {

        Serial.println("Initializing the light sensor with a pullup resistor and pulldown removed.");

      }
      if (initRangeFinderWRetries(i))
      {
	Serial.print(__FILE__);Serial.println(__LINE__);
        // loxes[activeRangeFinderCnt].startRanging();
        // downR.startRanging();
        activeRF[activeRangeFinderCnt] = i;
        activeRangeFinderCnt++;
      }

    }
  }
}

void pollRangeFinders()
{
  int count = 0;
  unsigned long now = millis();
  if ( ! i2cBusSafe ) {
    return;
  }
  for (int i = 0; i < activeRangeFinderCnt; i++)
  {
    int loxIndex = activeRF[i];
    // if ( loxes[i].checkForDataReady() ) {
    // if ( downR.checkForDataReady() ) {
    if ( false ) {
      // rangesInM[loxIndex] = (float)loxes[i].getDistance() * 0.001;
      // rangesInM[loxIndex] = (float)downR.getDistance() * 0.001;
      // loxes[i].stopRanging();
      // downR.stopRanging();
      // loxes[i].startRanging();
      // downR.stopRanging();
      switch (loxIndex) {
        case FRONTRANGE:
	   refSensor.rangeForward = rangesInM[loxIndex];
	   break;
	case DOWNRANGE:
	   refSensor.rangeDown = rangesInM[loxIndex];
	   break;
	case RIGHTRANGE:
	   refSensor.rangeRight = rangesInM[loxIndex];
	   break;
	case LEFTRANGE:
	   refSensor.rangeLeft = rangesInM[loxIndex];
	   break;
	 case TOPRANGE:
	   refSensor.rangeUp = rangesInM[loxIndex];
	   break;
	}
      lastR[loxIndex] = (lastR[loxIndex] + 1) % HISTDEPTH;
      // We don't know when it completes finding a range, since we are polling,
      // but we know when we start. So log the start time. Sometime after it completes and
      // we get around to polling the range finder, move the logged start time so it can
      // go with the measurement.(but do it in an order where we don't need a temp variable).
      rangesTS[loxIndex][P1] = rangesTS[loxIndex][P0]; // updateRangeInMeters() returned the measurement that was already in progress.
      rangesTS[loxIndex][P0] = micros(); // That call started the next measurement.
      rangeHistTotal[loxIndex] = rangeHistTotal[loxIndex] - rangeHist[loxIndex][lastR[loxIndex]] + rangesInM[loxIndex];
      rangeHist[loxIndex][lastR[loxIndex]] = rangesInM[loxIndex];
      rangeHistTS[loxIndex][lastR[loxIndex]] = rangesTS[loxIndex][P1];
      int indplus4 = (lastR[loxIndex] + 4) % HISTDEPTH;
      long deltaTimeMicros = rangeHistTS[loxIndex][lastR[loxIndex]] - rangeHistTS[loxIndex][indplus4];
      float deltaTime =
          (rangeHistTS[loxIndex][lastR[loxIndex]] - rangeHistTS[loxIndex][indplus4]) / 1000000.0; // in seconds
      float deltaDist = rangeHist[loxIndex][lastR[loxIndex]] - rangeHist[loxIndex][indplus4];


      if (deltaTime > 0.001 && deltaTime < 5.0)
      {

        rangeVel[loxIndex] = -deltaDist / deltaTime;
      } else
      {
        rangeVel[loxIndex] = -100.0; // Our signal for invalid data is about to impact.
      }


    } else
    {
      if ((micros() - rangeHistTS[loxIndex][lastR[loxIndex]]) > 100000)
      {
        rangeVel[loxIndex] = -100.0; // really fast towards impact
      }
    }
  }
  if (showRanges)
  {
    if (count > 0)
    {
      Serial.print(" time:");
      Serial.println(now);
    } else
    {
      Serial.print("*");
    }
  }
}
