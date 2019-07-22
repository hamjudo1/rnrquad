
#include <Arduino.h>
#include "wiring_private.h"
#include "../../state.h"

float rawXFlow = 0.0, rawYFlow = 0.0;
float rawQFlow = 0.0, rawTFlow = 0.0;
long sercom5timeStamp = 0;
unsigned long lastSample = 0; // Last real flow sensor data
unsigned long lastUpdate = 0; // Last time flow sensor data was updated, possibly indicating no data.
Uart flowSerial(&sercom5, 6, 20, SERCOM_RX_PAD_2, UART_TX_PAD_0); // Create the new UART instance assigning it by arduino pin numbers

/**
 * Magic
 */
void SERCOM5_Handler()
{
  flowSerial.IrqHandler();
  sercom5timeStamp = micros();
}


void processFlowPacket(int dx, int dy, int q, int t)
{
  float flowHeight = rangesInM[DOWNRANGE] - 0.040;
  if ( flowHeight < 0.0 ) {
    flowHeight = 0.0;
  }
  if ( (lastSample - sercom5timeStamp) < 50000 ) {
    refSensor.flowX = 0.5 * dx*flowHeight + refSensor.flowX; // Average in with recent last reading.
    refSensor.flowY = 0.5 * dy*flowHeight + refSensor.flowY;
  } else {
    refSensor.flowX = dx*flowHeight;  // No recent last reading, use this one only.
    refSensor.flowY = dy*flowHeight;
  }
  refSensor.flowQ = q;
  lastSample = sercom5timeStamp;
  lastUpdate = lastSample;
  refSensor.flowTimeStamp = (float)sercom5timeStamp/1000000;
  rawXFlow = dx; // Assumes flow sensor time is more stable than
  // measuring character timing. Will have to verify.

  rawYFlow = dy; // (1000 milliseconds per second, 40 milliseconds per loop,
  // Result is in arbitrary flow units per second.
  rawQFlow = q;
  rawTFlow = t;
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
  addSym(&rawXFlow, "rawxf", "flow sensor raw data", "1N");
  addSym(&rawYFlow, "rawyf", "flow sensor raw data", "1N");
  addSym(&rawQFlow, "rawqf", "flow sensor raw data", "0N");
  addSym(&rawTFlow, "rawtf", "flow sensor raw data", "1N");
  addSym(&(refSensor.flowX), "xf", "flow sensor X", "3N");
  addSym(&(refSensor.flowY), "yf", "flow sensor Y", "3N");
  addSym(&(refSensor.flowQ), "qf", "flow sensor Quality", "3N");
}

void pollFlow()
{
  static int flowIndex = 0;
  static int xFlow = 0;
  static int yFlow = 0;
  static int tFlow = 0;
  static int qFlow = 0;
  static int iter = 0;

  while (flowSerial.available() > 0)
  {
    int c = flowSerial.read();
    if (c < 0)
    {
      c = c + 256;
    }
    switch (flowIndex)
    {
      case 0:
        if (c != 0xFE)
        {
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
        if (xFlow > 32767)
        {
          xFlow -= 65536;
        }
        break;
      case 4:
        yFlow = c;
        break;
      case 5:
        yFlow += c * 256;
        if (yFlow > 32767)
        {
          yFlow -= 65536;
        }
        break;
      case 6:
        tFlow = c; // this is not Time.
        if (tFlow > 127)
        {
          tFlow = 256 - tFlow;
        }
        break;
      case 7:
        qFlow = c;
        // Quality 64 is can't see anything, in the shop I was
        // seeing 71 - 80ish under normal view conditions. Where 71 is almost blind, and
        // 80ish is very high contrast.
        break;
      case 8:
        if (c == 0xAA)
        {
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
  if ( micros() - lastUpdate > 50000 ) {
    
    lastUpdate = micros();

    refSensor.flowX *= 0.8;  // If no recent real data, make the old value decay away.
    refSensor.flowY *= 0.8; 
    refSensor.flowQ -= 2;
    
  }
}
