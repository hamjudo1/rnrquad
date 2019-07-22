#include "rnrquad.h"
#include "state.h"


const int versionGPIO = 4; // grounded on greenboard.
long initTime = 0;
extern int activeRangeFinderCnt;

extern void pollWatcher();

long debugTime = 0, debugStart = 0;
long commTime = 0, commStart = 0;
long neoTime = 0, neoStart = 0;
long rangeTime = 0, rangeStart = 0;
long thinkTime = 0, thinkStart = 0;
long nextDebugPacket = 1000;
long iter = 0;

void baseSetup()
{
  pinMode(versionGPIO, INPUT_PULLUP);

  setupSymtable();  // symbols to be logged.
  setupDebug();
  setupNeoSupp();
  setupFlow();
  setupController();
  for (int i = 0; i < 5; i++)
  {
    colorSingleDot(i, 240);
  }
  colorSingleDot(1, 120.0);
  setupSerial();
  colorSingleDot(2, 120.0);

  setupComm();

  setupRangeFinders();
  colorSingleDot(3, 120.0);
  colorSingleDot(4, 120.0);
  initTime = millis();
}

void baseLoop()
{
  colorSingleDot(1, 60);
  debugStart = millis();
  pollDebug();
  colorSingleDot(1, 120);
  commStart = millis();

  debugTime = debugTime + commStart - debugStart;
  pollComm();
  colorSingleDot(1, 180);

  neoStart = millis();
  commTime = commTime + neoStart - commStart;
  colorSingleDot(1, 240);
  rangeStart = millis();
  pollRangeFinders();
  colorSingleDot(1, 300);
  thinkStart = millis();
  rangeTime = rangeTime + thinkStart - rangeStart;
  pollFlow();
  colorSingleDot(1, 360);
  thinkTime = thinkTime + millis() - thinkStart;
  if (micros() - lastHeartBeat > 50000)
  {

    if (thinkStart % 666 > 222)
    {
      rgbSingleDot1(0, 1.0, 0.0, 0.0);
    } else
    {
      rgbSingleDot1(0, 0.0, 1.0, 0.0);
    }
  } else
  {
    colorSingleDot(0, iter++);
  }
}

/**
 * Gets current values of all sensors
 */
SensorState getSensorState()
{
  //TODO
}

/**
 * Gets current controller input values
 */
ControllerState getControllerState()
{
  return refControl;
}

/**
 * Sets output controller values
 */
void setControllerState(ControllerState controllerState)
{
  sendMotorSignal(controllerState);
}

bool waitForConnection = true;

void setupSerial()
{
  Serial.begin(115200);
  if (waitForConnection)
  {
    for (int i = 4; i > 0; i--)
    {
      Serial.print(i);
      delay(1000);
    }
    Serial.println("greenBoard");
  }
}
