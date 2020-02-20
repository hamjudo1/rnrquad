#include <Arduino.h>
#include "state.h"
int rangeFinderCycle = 20;

// SFEVL53L1X loxes[RFINDERS] = {SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X(),SFEVL53L1X()};
//SFEVL53L1X front;
//SFEVL53L1X downR;
//SFEVL53L1X right;
//SFEVL53L1X left;
//SFEVL53L1X up;

SFEVL53L1X forwardR;
SFEVL53L1X downR;
SFEVL53L1X rightR;
SFEVL53L1X leftR;
SFEVL53L1X upR;
float ERF[5] = {0, 0, 0, 0, 0};


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
unsigned long nextReading = 0;

// tools to allow blocking the configuration and use of the
// rangefinders, as the code has an awful tendency to hang the
// CPU if there are hardware problems.
bool rangeFindersDisabled = false;
bool setupRangeFindersRun = false;
void enableRangeFinders() {
  rangeFindersDisabled = false;
  if ( ! setupRangeFindersRun ) {
    setupRangeFinders();
  }
}
void disableRangeFinders() {
  rangeFindersDisabled = true;
  addCmd(enableRangeFinders, "RFC", "enable Rangefinders ", NULL);
}
float roiSize = 16.0;

void roi() {
  int roiX = (int)roiSize;
  int roiY = (int)roiSize;
  Serial.print("Setting region of interest on right range finder to ");
  Serial.println(roiX);
  rightR.setROI(roiX,roiY); // Note roiY is ignored!!!
  leftR.setROI(roiX,roiY); // Note roiY is ignored!!!
  upR.setROI(roiX,roiY); // Note roiY is ignored!!!
  downR.setROI(roiX,roiY); // Note roiY is ignored!!!
  forwardR.setROI(roiX,roiY); // Note roiY is ignored!!!
}
void setupRoi() {
  addSym(&roiSize,"roisize","region of interest size, 4 to 16","0");
  addCmd(roi,"roi","set region of interest to roisize", NULL);
}
void initRangeFinder(SFEVL53L1X finder, int finderIndex) {
  if ( rangeConfig[finderIndex].enabled ) {
    int pinNo = rangeConfig[finderIndex].j5Index;
    if (pinNo >= 0 ) {
      pinMode(pinNo, OUTPUT);
      digitalWrite(pinNo, 1);
    }
    finder.begin();
    finder.setI2CAddress(10 + (finderIndex * 2));
    (void)finder.startRanging(); // This returns a value that is bogus.

    delay(100);
    int dist = finder.getDistance();
    Serial.print("First reading in mm ");
    Serial.println(dist);
    finder.stopRanging();
    finder.startRanging();
    Serial.print("Init rangeFinder complete ");


  } else {
    Serial.print("Rangefinder already disabled. ");
  }
  Serial.println(rangeConfig[finderIndex].Name);
}
extern void testRangeFindersI2Csafe();
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
    bool success = false;
    // bool success = downR.begin();
    // bool success = loxes[activeRangeFinderCnt].begin( );
    // downR.setI2CAddress(1 + (i * 2));
    // loxes[activeRangeFinderCnt].setI2CAddress(1 + (i * 2));
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
  static bool rangeFinderSymsRun = false;
  if ( ! rangeFinderSymsRun ) {
    for (int i = 0; i < RFINDERS; i++)
    {
      allNames[i] = rangeConfig[i].Name;
      addSym(&(rangesInM[i]), allNames[i], "range finder", "3N");
    }
    for (int finderIndex = 0; finderIndex < 5; finderIndex++) {
      ERF[finderIndex] = (float)rangeConfig[finderIndex].enabled;
      addSym(&ERF[finderIndex], catString2(rangeConfig[finderIndex].Name, "_en"), "enable rangefinder", "F");
    }
    rangeFinderSymsRun = true;
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
void pollRangeFinders() {
  if( rangeFindersDisabled ) {
    if ( refControl.button7 != 0 ) {
      enableRangeFinders();
    }
    return;
  }
  int distance;
  int dDown, dLeft, dRight, dUp;;
  if ( millis() > nextReading ) {
    nextReading = millis() + rangeFinderCycle;
    if ( rangeConfig[0].enabled ) {
      distance = forwardR.getDistance(); //Get the result of the measurement from the sensor
      refSensor.rangeForward = (float)distance * 0.001;
      rangesInM[0] = refSensor.rangeForward;
      forwardR.stopRanging();
      forwardR.startRanging();
    }
    if ( rangeConfig[1].enabled ) {
      dDown = downR.getDistance(); //Get the result of the measurement from the sensor
      refSensor.rangeDown = (float)dDown * 0.001;
      rangesInM[1] = refSensor.rangeDown;
      downR.stopRanging();
      downR.startRanging();
    }
    if ( rangeConfig[2].enabled ) {
      dRight = rightR.getDistance(); //Get the result of the measurement from the sensor
      refSensor.rangeRight = (float)dRight * 0.001;
      rangesInM[2] = refSensor.rangeRight;
      rightR.stopRanging();
      rightR.startRanging();
    }
    if ( rangeConfig[3].enabled ) {
      dLeft = leftR.getDistance(); //Get the result of the measurement from the sensor
      refSensor.rangeLeft = (float)dLeft * 0.001;
      rangesInM[3] = refSensor.rangeLeft;
      leftR.stopRanging();
      leftR.startRanging();
    }

    if ( rangeConfig[4].enabled ) {
      dUp = upR.getDistance(); //Get the result of the measurement from the sensor
      refSensor.rangeUp = (float)dUp * 0.001;
      rangesInM[4] = refSensor.rangeUp;
      upR.stopRanging();
      upR.startRanging();
    }
  }
}
void setupRangeFinders() {
  int finderIndex;
  for(int i =0; i<5; i++ ) {
    Led::rgbColorSingleLed(i,0,0.25,0);
  }
  rangeFinderSyms();
  if( rangeFindersDisabled ) {
    return;
  }
  testRangeFindersI2Csafe();
  for (finderIndex = 0; finderIndex < 5; finderIndex++) {
    rangeConfig[finderIndex].enabled = (ERF[finderIndex] != 0.0);
    Serial.print(rangeConfig[finderIndex].enabled);
    Serial.print(" ");
    Serial.println(rangeConfig[finderIndex].Name);
  }
  // i2cBusSafe = false;
  // Serial.println("Disabling i2c bus");
  if ( ! i2cBusSafe ) {
    Serial.println("i2cBus is not safe??");
    return;
  }
  Serial.println("i2cBus is safe??");
  Wire.begin();
  Serial.println("Starting front");

  initRangeFinder(forwardR, 0);
  Serial.println("Starting down");
  initRangeFinder(downR, 1);

  Serial.println("Starting right");
  initRangeFinder(rightR, 2);

  Serial.println("Starting left");
  initRangeFinder(leftR, 3);

  Serial.println("Starting up");
  initRangeFinder(upR, 4);
  nextReading = millis() + rangeFinderCycle;

  setupRangeFindersRun = true; 
  setupRoi();
}

