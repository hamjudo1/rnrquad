#include "../../vl53l0x/src/Adafruit_VL53L0X.h"

// #include <Adafruit_NeoPixel.h>
// i2c pin 31 SDA Arduino20 (default for wire library)
// i2c pin 32 SCL Arduino21 (likewise)

#include "../../state.h"

Adafruit_VL53L0X loxes[RFINDERS] = {Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(),
                                    Adafruit_VL53L0X()
};

#define TCAADDR 0x74
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

void tcaselect(uint8_t i)
{
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
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
    bool success = loxes[activeRangeFinderCnt].begin(1 + (i * 2), true);
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

void setupRangeFinders()
{
  // Setup I2C
  Wire.begin();
  int i, aPinNo;
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
    if (aPinNo >=
        0)
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
      Serial.print("quadRange active count ");
      Serial.print(activeRangeFinderCnt);
      Serial.print(" i ");
      Serial.println(i);
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
        loxes[activeRangeFinderCnt].startRanging();
        activeRF[activeRangeFinderCnt] = i;
        activeRangeFinderCnt++;
      }

      addSym(&(rangesInM[i]), allNames[i], "range finder", "3N");
      Serial.print("quadRange ");
      Serial.println(__LINE__);
    }
  }
  Serial.print("quadRange ");
  Serial.println(__LINE__);
}

void pollRangeFinders()
{
  int count = 0;
  unsigned long now = millis();
  for (int i = 0; i < activeRangeFinderCnt; i++)
  {
    int loxIndex = activeRF[i];
    if (loxes[i].updateRangeInMeters(&(rangesInM[loxIndex])))
    {
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
