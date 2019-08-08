#include "rnrquad.h"
#include "state.h"


const int versionGPIO = 4; // grounded on greenboard.
long initTime = 0;

extern void pollWatcher();

long debugTime = 0, debugStart = 0;
long commTime = 0, commStart = 0;
long neoTime = 0, neoStart = 0;
long rangeTime = 0, rangeStart = 0;
long thinkTime = 0, thinkStart = 0;
long nextDebugPacket = 1000;
long iter = 0;
unsigned long baseLoopEnd=0; // baseLoop is padded out to a constant time
float ml=0.025;
unsigned long loopTime;
unsigned long loopEnd;
unsigned long baseLoopTime;
float overrun;
void padLoopTime() {
  long excessTime = baseLoopEnd - micros();
  if ( excessTime > 50 ) {
    delayMicroseconds(excessTime);
  } else if ( excessTime < -1000 ) {
    Serial.print("Loop Time Exceeded by ");
    overrun = -(float)excessTime*0.000001;
    Serial.print(overrun,3);
    Serial.println(" seconds.");
    Serial.print("baseLoopTime is ");
    Serial.println(baseLoopTime);
    baseLoopEnd = micros();
  }
  baseLoopEnd += baseLoopTime;
}
void firstLoopPadding() {
  baseLoopTime = (long)(ml*1000000);
  baseLoopEnd = ((micros() /baseLoopTime)+1) * baseLoopTime;
  padLoopTime();
}
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
    Led::rgbColorSingleLed(i,0.2,0.2,0.0);
  }
  setupSerial();
  Led::rgbColorSingleLed(0,0.2,0.0,0.0);
  setupComm();
  Led::rgbColorSingleLed(1,0.2,0.0,0.0);
  setupRangeFinders();
  Led::rgbColorSingleLed(2,0.2,0.0,0.0);
  initTime = millis();
  Led::rgbColorSingleLed(3,0.2,0.0,0.0);
  firstLoopPadding();
  Led::rgbColorSingleLed(4,0.2,0.0,0.0);
}
bool debugHang = false; // Set to true Diagnose hangs in the main loop.

void baseLoop()
{
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 60);
  }
  debugStart = millis();
  pollDebug();
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 120);
  }
  commStart = millis();

  debugTime = debugTime + commStart - debugStart;
  pollComm();
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 180);
  }

  neoStart = millis();
  commTime = commTime + neoStart - commStart;
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 240);
  }
  rangeStart = millis();
  pollRangeFinders();
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 300);
  }
  thinkStart = millis();
  rangeTime = rangeTime + thinkStart - rangeStart;
  pollFlow();
  if ( debugHang ) {
    Led::hsvColorSingleLed(4, 360);
  }
  thinkTime = thinkTime + millis() - thinkStart;
  
  unsigned long now = micros();
  uint32_t color1 = red;
  uint32_t color2 = yellow;
  if (now - lastHeartBeat > 50000)
  {
    color1 = red;
    color2 = yellow;
    if ( lastHeartBeat == 0 ) {
      color2 = black;
    }
    if ( lastRadioContact == 0 ) { // never
      redSet.blink(0,color1,color2);
      redSet.blink(1,color1,color2);
      redSet.blink(2,color1,color2);
      redSet.blink(3,color2,color1);
      redSet.blink(4,color2,color1);
    } else if ( now - lastRadioContact > 200000 ) { // in the past, not recent
      redSet.blink(0,color1,color2);
      redSet.blink(1,color2,color1);
      redSet.blink(2,color1,color2);
      redSet.blink(3,color2,color1);
      redSet.blink(4,color1,color2);
    } else {  // recent
      redSet.blink(0,color1,color2);
      redSet.blink(1,color1,color2);
      redSet.blink(2,color1,color2);
      redSet.blink(3,color1,color2);
      redSet.blink(4,color2,color1);
    }
  } else
  {
    if ( voltage < 3.0 ) {
      color2 = black;
    } else {
      float intensity = (voltage - 3.0) * 0.22;
      color1 = Led::rgbColor((4.2 - 3.0) * 0.22, 0.0, 0.0);
      color2 = Led::rgbColor(intensity,0.0,0.0);
      if ( lastRadioContact == 0 ) { // never
	redSet.blink(0,color1,color2);
	redSet.blink(1,color1,color2);
	redSet.blink(2,color1,color2);
	redSet.blink(3,color2,color1);
	redSet.blink(4,color2,color1);
      } else if ( now - lastRadioContact > 200000 ) { // in the past, not recent
	redSet.blink(0,color1,color2);
	redSet.blink(1,color2,color1);
	redSet.blink(2,color1,color2);
	redSet.blink(3,color2,color1);
	redSet.blink(4,color1,color2);
      } else {  // recent
	redSet.blink(0,color1,color2);
	redSet.blink(1,color1,color2);
	redSet.blink(2,color1,color2);
	redSet.blink(3,color1,color2);
	redSet.blink(4,color1,color2);
      }
    }
  }
  Led::poll();
  padLoopTime();
}

/**
 * Gets current values of all sensors
 */
SensorState getSensorState()
{
  return refSensor;
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
    for (int i = 5; i > 0; i--)
    {
      Serial.print(i);
      delay(400);
    }
    Serial.println();
  }
}
