
#include <Arduino.h>
#include "wiring_private.h"
#include "state.h"
float newestXFlow = 0.0, newestYFlow = 0.0;
float newestQFlow = 0.0, newestTFlow = 0.0;
long sercom5timeStamp = 0;
Uart flowSerial (&sercom5, 6, 20, SERCOM_RX_PAD_2, UART_TX_PAD_0); // Create the new UART instance assigning it by arduino pin numbers 
void SERCOM5_Handler()
{
  flowSerial.IrqHandler();
  sercom5timeStamp = micros();
}
void setupFlow()
{
  // The Feather M0 is derived from the Arduino Zero. The Zero uses
  // SERCOM5 to talk to the second USB port. The Feather M0 doesn't
  // have that hardware, but the drivers still know about it. When
  // We use Sercom5, the driver has some special code for that serial
  // port.
  pinPeripheral(6, PIO_SERCOM); //Assign RX function to pin 6
  flowSerial.begin(19200);      // This will initialize SERCOM5 correctly, but mess up pin 6
  pinPeripheral(6, PIO_SERCOM); // Switch the port back to pin 6
  colorSingleDot(0, 0.0);
  addSym(&newestXFlow, "xFlow", "flow sensor", "1N" );
  addSym(&newestYFlow, "yFlow", "flow sensor", "1N" );
  addSym(&newestQFlow, "qFlow", "flow sensor", "0N" );
  addSym(&newestTFlow, "tFlow", "flow sensor", "0N" );
}

long totLag = 0, worstLag = 0, smallDelta = 1000000, bigDelta = 0;
long sampleCount = 0;
float xSpeed = 0.0, ySpeed = 0.0;
float xSum = 0.0, ySum = 0.0, qSum = 0;
unsigned long lastSample = 0;
unsigned long lastAveTime = 0;

int getAveFlow(float &xFlow, float &yFlow, float &qFlow) {
  long age = micros() - lastSample;
  if ( newestXFlow == 0.0 && newestYFlow == 0.0 ) {
    noFlowNoise ++;
  } else {
    noFlowNoise = 0;
  }
  if ( age > 50000 || newestQFlow < 71) { // 50k microseconds, ie 50milliseconds.)
    if ( age > 50000 ) {
      //for (int n = 3; n < 5; n++) {
        rgbSingleDot1(4, 1.0, 1.0, 1.0);
      //}
    } else {
      //for (int n = 3; n < 5; n++) {
        rgbSingleDot1(4, 1.0, 0.0, 0.0);
      //}
    }
    return 0;
  } 
  
  xFlow = 0.5*xFlow + 0.5*newestXFlow;
  yFlow = 0.5*yFlow + 0.5*newestYFlow;
  qFlow = newestQFlow;
  for (int n = 3; n < 5; n++) {
    float flowSquared = xFlow * xFlow + yFlow * yFlow;
    colorSingleDot(n, 120+(flowSquared / 20.0));
    //  rgbSingleDot1(n, 0.1, 0.1, 0.1);
  }
  return 1;
}
void processFlowPacket(int dx, int dy, int q, int t) {
  lastSample = sercom5timeStamp;
  newestXFlow =  dx; // Assumes flow sensor time is more stable than
  // measuring character timing. Will have to verify.

  newestYFlow = dy; // (1000 milliseconds per second, 40 milliseconds per loop,
  // Result is in arbitrary flow units per second.
  newestQFlow = q;
  newestTFlow = t;

}
int getAveFlowComplex(float &xFlow, float &yFlow, float &qFlow) {
  long sc = sampleCount;
  if ( sc != 0 && lastAveTime != 0 ) {
    sampleCount = 0;
    static unsigned long lastStart = 0;
    lastStart = lastAveTime;

    static unsigned long duration = 0;
    duration = lastSample - lastStart;
    if ( duration > 0 ) {
      xFlow = (1000000.0 * xSum) /  duration;
      yFlow = (1000000.0 * ySum) /  duration;
      qFlow = 0.05 * (float)qSum / (float)sc;
    }
    /*   Serial.print("Samples: ");
       Serial.print(sc);
       Serial.print(" xSum: ");
       Serial.print(xSum);
       Serial.print(" ySum: ");
       Serial.print(ySum);
       Serial.print(" qSum: ");
       Serial.println(qSum); */
    xSum = 0;
    ySum = 0;
    qSum = 0;
  } else {
    for (int n = 0; n < 5; n++) {
      rgbSingleDot1(n, 1.0, 1.0, 1.0);
    }
    xFlow = 0.0;
    yFlow = 0.0;
    qFlow = 64;
  }
  lastAveTime = lastSample;
  return sc;
}
void processFlowPacketComplex(int dx, int dy, int q, int t) {

  static unsigned long deltaT = 0;
  static unsigned long lag = 0;
  static unsigned long now = 0 ;
  static unsigned long ts = 0;
  ts = sercom5timeStamp;
  now = micros();
  if ( lastSample != 0 ) {
    deltaT = ts - lastSample;
    if ( deltaT < smallDelta ) {
      smallDelta = deltaT;
    }
    if ( deltaT > bigDelta ) {
      bigDelta = deltaT;
    }
    lag = now - ts;
    if ( lag > worstLag ) {
      worstLag = lag;
    }
    totLag += lag;
  }
  lastSample = ts;
  sampleCount++;
  if ( q > 70 ) {
    // Serial.print(".");

    //colorSingleDot(0, (q - 70) * 30);
    colorSingleDot(1, dx);
    colorSingleDot(3, dy);
    xSum += dx;
    ySum += dy;
    qSum += (q - 64);
    xSpeed = xSpeed * 0.9 + dx * 0.1;
    ySpeed = ySpeed * 0.9 + dy * 0.1;
    colorSingleDot(2, xSpeed);
    colorSingleDot(4, ySpeed);
    //  Serial.print("(");
    //   Serial.print(xSpeed);
    //  Serial.print(", ");
    //   Serial.print(ySpeed);
    //   Serial.println(")");
  } else {
    xSum = 0;
    qSum += (q - 64);
    ySum = 0;

    /*  if ( sampleCount != 0 ) {
        static float xFlow = 0.0, yFlow = 0.0, qFlow = 0.0;
        if ( getAveFlow(xFlow, yFlow, qFlow) > 0 ) {
          Serial.print("x: ");
          Serial.print(xFlow, 3);
          Serial.print(" y: ");
          Serial.print(yFlow, 3);
          Serial.print(" q: ");
          Serial.println(qFlow);
        }

         Serial.print("Now: ");
           Serial.print(now);
           Serial.print(" ts: ");
           Serial.print(ts);
           Serial.print(" sampleCount: ");
           Serial.print(sampleCount);
           Serial.print(" deltaT: ");
           Serial.print(deltaT);
           Serial.print(" range ");
           Serial.print(smallDelta);
           Serial.print(" - ");
           Serial.print(bigDelta);
           Serial.print(" lag: ");
           Serial.print(lag);
           Serial.print(" worst Lag: ");
           Serial.println(worstLag);
      } */
    /*
      xSpeed = 0.0;
      ySpeed = 0.0;
      xSum = 0.0;
      ySum = 0.0;
      totLag = 0;
      lag = 0;
      worstLag = 0;
      sampleCount = 0;
      smallDelta = 1000000;
      bigDelta = 0;
      lastSample = 0;
    */
  }
}
void pollFlow()
{
  static int flowIndex = 0;
  static int xFlow = 0;
  static int yFlow = 0;
  static int tFlow = 0;
  static int qFlow = 0;
  static int iter = 0;

  while ( flowSerial.available() > 0 ) {
    int c = flowSerial.read();
    if ( c < 0 ) {
      c = c + 256;
    }
    switch (flowIndex) {
      case 0:
        if ( c != 0xFE ) {
          flowIndex = -1;
        }
        break;
      case 1:
        // Reserved byte, always 0x04 on our sample units
        break;
      case 2:
        xFlow = c;
        break;
      case 3:
        xFlow += c * 256;
        if ( xFlow > 32767 ) {
          xFlow -= 65536;
        }
        break;
      case 4:
        yFlow = c;
        break;
      case 5:
        yFlow += c * 256;
        if ( yFlow > 32767 ) {
          yFlow -= 65536;
        }
        break;
      case 6:
        tFlow = c; // this is not Time.
        if ( tFlow > 127 ) {
          tFlow =256-tFlow;
        }
        break;
      case 7:
        qFlow = c; // Quality 64 is can't see anything, in the shop I was
        // seeing 71 - 80ish under normal view conditions. Where 71 is almost blind, and
        // 80ish is very high contrast.
        break;
      case 8:
        if ( c == 0xAA ) {
          processFlowPacket(xFlow, yFlow, qFlow, tFlow);
        }
        flowIndex = -1;
        break;
      default:
        flowIndex = -1;
        break;
    }
    flowIndex += 1;
  }
}

