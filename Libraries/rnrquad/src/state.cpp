#include "state.h"

boolean radioInitialized = false;
int mode;
long longestQuietTime = 0;
int tookTooLongCount = 0;
boolean readyToUpdateNeoPixels = false;
unsigned long nextNeoUpdate = 0;
unsigned long lastHeartBeat = 0;
int stuffQueued = 0;
int stuffUnqueued = 0;
int SPISlaveInterrupts = 0;
int SPISlaveCharactersRead = 0;
int SPISlaveCharactersWritten = 0;
int cortexState = STATE_CORTEX_ALIVE;
int voltageByte = 257;
float voltage = 4.2;
float showBrainStemLog = 0.0;
float showPacketLog = 0.0;
float showXn297LLog = 0.0;
float showUsageLog = 0.0;
float showLogLog = 0;
boolean unreadPacket = false;
boolean unprocessedPacket = false;
boolean whiteBoard = false;
boolean greenBoard = false;
float showRanges = 0.0;
float showPID = 0.0;
float showVerbose = 0.0;
float byPassManipulation = 1.0;
int inputNum = 0;
int userNum = 0;
float rangesInM[5];
unsigned long rangesTS[RFINDERS][2];
float rangeHist[RFINDERS][HISTDEPTH];
float rangeHistTotal[RFINDERS];
unsigned long rangeHistTS[RFINDERS][HISTDEPTH];
float rangeVel[RFINDERS];
int lastR[RFINDERS];
float tState;
int lCnt = 0;
float errorList[MAX_ERROR_REPORTS]; // these are to hold codes associated with errors
int errorListIndex = 0;
float A = 0.0;
float B = 0.0;
float C = 0.0;
float D = 0.0;
float E = 0.0;
float F = 0.0;

void showUsageTime()
{
  long totTime = debugTime + commTime + neoTime + rangeTime + thinkTime;
  float debugPercent = (debugTime * 100.0) / totTime;
  float commPercent = (commTime * 100.0) / totTime;
  float neoPercent = (neoTime * 100.0) / totTime;
  float rangePercent = (rangeTime * 100.0) / totTime;
  float thinkPercent = (thinkTime * 100.0) / totTime;
  Serial.print("debug %");
  Serial.print(debugPercent);
  Serial.print(" comm %");
  Serial.print(commPercent);
  Serial.print(" neo %");
  Serial.print(neoPercent);
  Serial.print(" range %");
  Serial.print(rangePercent);
  Serial.print(" think %");
  Serial.println(thinkPercent);
}

void pollDebug()
{
  lCnt++;
  // digitalWrite(simpleLED, lCnt % 2);
  if (lCnt % 40 == 0)
  {
    if (showUsageLog)
    {
      showUsageTime();
    }
  }
  pollWatch();
  pollBlink();
  while (Serial.available())
  {
    int c = Serial.read();
    processChar(c);
  }
}

void sysStatus()
{
  Serial.print("Uptime ");
  Serial.print(millis());
  Serial.print(" Longest quiet time: ");
  Serial.println(longestQuietTime);
  Serial.print("Count of take too long events ");
  Serial.println(tookTooLongCount);
  Serial.print("Queue stats queued: ");
  Serial.print(stuffQueued);
  Serial.print(", unqueued: ");
  Serial.println(stuffUnqueued);
  Serial.print("SPI slave interrupts ");
  Serial.println(SPISlaveInterrupts);
  Serial.print("Packets Received ");
  Serial.println(packetrx);
  Serial.print("registers written ");
  Serial.print(xn297_regs_written);
  Serial.print(", read ");
  Serial.println(xn297_regs_read);
  Serial.print("Status values seen ");
  for (int i = 0; i < 256; i++)
  {
    if (statusValCount[i])
    {
      Serial.print(i);
      Serial.print(":");
      Serial.print(statusValCount[i]);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.print("Status check count ");
  Serial.println(stem_status_check_count);

  if (radioInitialized)
  {
    Serial.println("Radio initialized");
  } else
  {
    Serial.println("Radio NOT initialized");
  }
}

void oldPollDebug()
{
  if (Serial.available())
  {
    int letter = Serial.read();
    if (letter >= 32 && letter <= 126)
    {
      mode = letter;
      if (mode == 'r')
      {
        Serial.println("Restarting the radio.");
        radioDefault(); // reset the radio
      } else if (mode == 's')
      {  // Status
        Serial.print("Uptime ");
        Serial.print(millis());
        Serial.print(" Longest quiet time: ");
        Serial.println(longestQuietTime);
        Serial.print("Count of take too long events ");
        Serial.println(tookTooLongCount);
        Serial.print("Queue stats queued: ");
        Serial.print(stuffQueued);
        Serial.print(", unqueued: ");
        Serial.println(stuffUnqueued);
        if (radioInitialized)
        {
          Serial.println("Radio initialized");
        } else
        {
          Serial.println("Radio NOT initialized");
        }
      } else if (mode == 'u')
      { // Update Neo Pixels
        Serial.println("Updating NeoPixels.");
        quietTime();
      } else if (mode == 'i')
      { // Init
        Serial.println("Initializing slave SPI");
        spiSlave_init();
      } else if (mode == 'a')
      {
        byPassManipulation = false;
        Serial.println("thinking on");
      } else if (mode == 'A')
      {
        byPassManipulation = true;
        Serial.println("thinking off");
      } else if (mode == 'c')
      {
        showPID = false;
        Serial.println("stop print PID data");
      } else if (mode == 'C')
      {
        showPID = true;
        Serial.println("print PID data");
      } else if (mode == 'b')
      {
        Serial.println("Turning off brainstem log listing");
        showBrainStemLog = false;
      } else if (mode == 'B')
      {
        Serial.println("Turning on brainstem log listing");
        showBrainStemLog = true;
      } else if (mode == 'x')
      {
        Serial.println("Turning off xn297L log listing");
        showXn297LLog = false;
      } else if (mode == 'X')
      {
        Serial.println("Turning on xn297L log listing");
        showXn297LLog = true;
      } else if (mode == 'p')
      {
        Serial.println("Turning off packet log listing");
        showPacketLog = false;
      } else if (mode == 'P')
      {
        Serial.println("Turning on packet log listing");
        showPacketLog = true;
      } else if (mode == 'd')
      {
        Serial.println("Turning off distance listing");
        showRanges = false;
      } else if (mode == 'D')
      {
        Serial.println("Turning on Distance listing");
        showRanges = true;
      } else if (mode == 'v')
      {
        Serial.println("Turning off verbosity");
        showVerbose = false;
      } else if (mode == 'V')
      {
        Serial.println("Turning on verbosity");
        showVerbose = true;
      } else if (letter >= '0' && letter <= '9')
      {
        inputNum = inputNum * 10 + (letter - '0');
      } else if (mode == ',')
      {
        userNum = inputNum;
        inputNum = 0;
      } else if (mode == '*')
      {
        if (inputNum > 0)
        {
          Serial.print("You asked to blink pin ");
          Serial.println(inputNum);
          blinkIt(inputNum);
          inputNum = 0;

        } else
        {
          Serial.println(
              "To blink an LED on a pin, enter a 1 or 2 digit number followed by *, for example 23* to blink an LED on pin 23");
        }
      } else if (mode == '%')
      {
        showUsageTime();
      } else
      {
        Serial.println("We are alive!");
      }
    }
  }
}

int blinkingPin = 0;

void blinkIt(int pinNo)
{

  if (blinkingPin != 0)
  {
    pinMode(blinkingPin, INPUT);
  }
  blinkingPin = pinNo;
  pinMode(blinkingPin, OUTPUT);
  digitalWrite(blinkingPin, HIGH);
  Serial.print("Trying to blink ");
  Serial.println(blinkingPin);
}

void pollBlink()
{
  if (blinkingPin > 0)
  {
    digitalWrite(blinkingPin, (millis() % 200 < 100));
  }
}

void leftTrimChanged(int trimVal)
{
  static int leftTrimPresses = 0;
  leftTrimPresses++;
  int choice = leftTrimPresses % 6;
  float angle = 60.0 * choice;
  for (int i = 0; i < 5; i++)
  {
    colorSingleDot(i, angle);
  }
  Serial.print("Mode ");
  Serial.print(angle, 0);
  switch (choice)
  {
    case 0:
      Serial.println(" green");
      break;
    case 1:
      Serial.println(" yellow");
      break;
    case 2:
      Serial.println(" red");
      break;
    case 3:
      Serial.println(" magenta");
      break;
    case 4:
      Serial.println(" blue");
      Serial.println("Motor test: Throttle direct to motors");
      break;
    case 5:
      Serial.println(" cyan");
      break;
  }
}

void dispPacketLog()
{
  Serial.print("Packets ");
  Serial.print(packetrx);
  Serial.print("rx array: ");
  for (int rxN = 0; rxN < 4; rxN++)
  {
    Serial.print(rx[rxN]);
    if (rxN < 3)
    {
      Serial.print(", ");
    }
  }

  Serial.print(" aux array: ");
  for (int aN = 0; aN < AUXNUMBER; aN++)
  {
    Serial.print((int) aux[aN]);
  }
  Serial.print(" trims array: ");
  for (int i = 0; i < 4; i++)
  {
    Serial.print((int) trims[i]);
    Serial.print(" ");
  }
  Serial.println();
  for (int b = 0; b < 15; b++)
  {
    Serial.print(rxdata[b]);
    Serial.print(" ");
  }
  Serial.println();
}

void dispXn297LLog()
{
  Serial.println("Please make dispXn297LLog() into a real function");
}

int sampleData[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

void writePacketTest()
{
  for (int i = 0; i < 20; i++)
  {
    int before = xn_readreg(FIFO_STATUS);
    writePacket(sampleData, 15);
    int after = xn_readreg(FIFO_STATUS);
  }
}

void freeRam()
{
  unsigned char *blocks[32];
  int i, allocCnt;
  for (i = 0; i < 32; i++)
  {
    unsigned char *block = (unsigned char *) malloc(1024);
    if (block == (unsigned char *) NULL)
    {
      for (int j = 0; j < i; j++)
      {
        free(blocks[j]);
      }
      Serial.print(i);
      Serial.println("k bytes were available.");
      return;
    }
    blocks[i] = block;
  }
  Serial.println("We cannot get here, but we did. Does this processor have more RAM?");
}

float vbattfilt, vbatt_com, GEstG0, GEstG1, GEstG2, sample1;

void setupDebug()
{
  mode = 0;
  addSym(&A, "A", "A from stem", "6N");
  addSym(&B, "B", "B from stem", "6N");
  addSym(&C, "C", "C from stem", "6N");
  addSym(&D, "D", "D from stem", "6N");
  addSym(&E, "E", "E from stem", "6N");
  addSym(&F, "F", "F from stem", "6N");
  addSym(&showBrainStemLog, "blog", "FLAG showBrainStemLog", "F");
  addSym(&showPacketLog, "xlog", "FLAG showPacketLog", "F");
  addSym(&showRanges, "slog", "FLAG showRanges (rangefinder data)", "F");
  addSym(&showPID, "plog", "FLAG showPID", "F");
  addSym(&showVerbose, "vlog", "FLAG showVerbose", "F");
  addSym(&showLogLog, "llog", "FLAG showLogLog", "F");
  addSym(&showUsageLog, "ulog", "FLAG showUsageLog", "F");
  addSym(&byPassManipulation, "bypass", "FLAG byPassManipulation", "F");
  addSym(&vbattfilt, "vbattfilt", "", "3N");
  addSym(&vbatt_com, "vbatt_com", "", "3N");
  addSym(&sample1, "sample1", "", "3N");
  addSym(&GEstG0, "GEstG0", "", "3N");
  addSym(&GEstG1, "GEstG1", "", "3N");
  addSym(&GEstG2, "GEstG2", "", "3N");
  addSym(&voltage, "volts", "", "3N");
  addSym(&estoprange, "estoprange", "E-stop on rangefinder detection of something close", "F");
  addCmd(writePacketTest, "wp", "Write packet from quad to remote control", NULL);
  addCmd(sysStatus, "stats", "show some debugging stats", NULL);
  addCmd(freeRam, "free", "report available RAM", NULL);
}


void quietTime()
{
  unsigned long startTime, stopTime, deltaTime;
  startTime = micros();
  if (readyToUpdateNeoPixels)
  {
    NeoUpdate();
  }
  //  writePacket(sampleData, 15);
  stopTime = micros();
  if (stopTime < startTime)
  {
    // clock rolled over.
    stopTime = stopTime + 1000000;
    startTime = startTime + 1000000;
  }
  deltaTime = stopTime - startTime;
  if (deltaTime > 1000)
  {
    tookTooLongCount++;
  }
  if (deltaTime > longestQuietTime)
  {
    longestQuietTime = deltaTime;
  }
}

#if NOTUSED
void cortexDebug(enum cortex_debug debug_event) {
  switch (debug_event) {
    case packet_in_fifo:
      pixel.setPixelColor(4, pixel.Color(25, 25, 0));
      break;
    case radio_confused:
      pixel.setPixelColor(4, pixel.Color(0, 0, 50));
      break;
    case we_are_bound:
      pixel.setPixelColor(4, pixel.Color(0, 30, 0));
      break;
    case range_display:
      break;
    case cortex_loop:
      break;
    default:
      break;
  }
}
#endif

void cortexDebugRange(enum cortex_debug debug_event, int rangeFinderIndex, int rangeInMM)
{

}

