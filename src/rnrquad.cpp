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
  firstLoopPadding();
}
bool debugHang = false; // Set to true Diagnose hangs in the main loop.

void baseLoop()
{
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 60);
  }
  debugStart = millis();
  pollDebug();
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 120);
  }
  commStart = millis();

  debugTime = debugTime + commStart - debugStart;
  pollComm();
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 180);
  }

  neoStart = millis();
  commTime = commTime + neoStart - commStart;
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 240);
  }
  rangeStart = millis();
  pollRangeFinders();
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 300);
  }
  thinkStart = millis();
  rangeTime = rangeTime + thinkStart - rangeStart;
  pollFlow();
  if ( debugHang ) {
    Led::hsvColorSingleLed(1, 360);
  }
  thinkTime = thinkTime + millis() - thinkStart;
  
  if (micros() - lastHeartBeat > 50000)
  {
    redSet.blink(0,red,blue);
    redSet.blink(1,blue,red);
  } else
  {
    redSet.constant(0,red);
    redSet.constant(1,red);
  } 
  if ( voltage < 3.1 ) {
    redSet.blink(2,Led::hsvColor(60,1.0,1.0),Led::hsvColor(120,1.0,1.0));
    redSet.blink(3,Led::hsvColor(120,1.0,1.0),Led::hsvColor(60,1.0,1.0));
  } else if ( voltage < 3.2 ) {
    redSet.blink(2,Led::hsvColor(120,1.0,0.8),Led::hsvColor(180,1.0,0.8));
    redSet.blink(3,Led::hsvColor(180,1.0,0.8),Led::hsvColor(120,1.0,0.8));
  } else if ( voltage < 3.3 ) {
    redSet.blink(2,Led::hsvColor(180,1.0,0.6),Led::hsvColor(240,1.0,0.6));
    redSet.blink(3,Led::hsvColor(240,1.0,0.6),Led::hsvColor(180,1.0,0.6));
  } else {
    redSet.blink(2,Led::hsvColor(240,1.0,0.4),Led::hsvColor(300,1.0,0.4));
    redSet.blink(3,Led::hsvColor(300,1.0,0.4),Led::hsvColor(240,1.0,0.4));
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
