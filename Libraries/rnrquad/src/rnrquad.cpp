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
  setupUtility();
  setupDebug();
  Led::setup();
  setupFlow();
  setupController();
  for (int i = 0; i < 5; i++)
  {
    Led::hsvColorSingleLed(i, 240);
  }
  Led::hsvColorSingleLed(1, 120.0);
  setupSerial();
  Led::hsvColorSingleLed(2, 120.0);

  setupComm();

  setupRangeFinders();
  Led::hsvColorSingleLed(3, 120.0);
  Led::hsvColorSingleLed(4, 120.0);
  initTime = millis();
}

void baseLoop()
{
  Led::hsvColorSingleLed(1, 60);
  debugStart = millis();
  pollDebug();
  Led::hsvColorSingleLed(1, 120);
  commStart = millis();

  debugTime = debugTime + commStart - debugStart;
  pollComm();
  Led::hsvColorSingleLed(1, 180);

  neoStart = millis();
  commTime = commTime + neoStart - commStart;
  Led::hsvColorSingleLed(1, 240);
  rangeStart = millis();
  pollRangeFinders();
  Led::hsvColorSingleLed(1, 300);
  thinkStart = millis();
  rangeTime = rangeTime + thinkStart - rangeStart;
  pollFlow();
  Led::hsvColorSingleLed(1, 360);
  thinkTime = thinkTime + millis() - thinkStart;
  if (micros() - lastHeartBeat > 50000)
  {

    if (thinkStart % 666 > 222)
    {
      Led::rgbColorSingleLed(0, 1.0, 0.0, 0.0);
    } else
    {
      Led::rgbColorSingleLed(0, 0.0, 1.0, 0.0);
    }
  } else
  {
    Led::hsvColorSingleLed(0, iter++);
  }
  Led::poll();
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
 *
 * @param controllerState
 */
void setControllerState(ControllerState controllerState)
{
  sendMotorSignal(controllerState);
}

/**
 * Sets LED values
 * # | pos
 * 0 | back
 * 1 | left
 * 2 | left_front
 * 3 | right_front
 * 4 | right
 *
 * @param led
 * @param ledNumber
 */
void setLED(int ledNumber, RgbLed led)
{
  Led::rgbColorSingleLed(ledNumber, led.red, led.green, led.blue);
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
