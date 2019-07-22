
#include <Arduino.h>
#include "wiring_private.h"
#include "../../state.h"

float newestXFlow = 0.0, newestYFlow = 0.0;
float newestQFlow = 0.0, newestTFlow = 0.0;
long sercom5timeStamp = 0;
Uart flowSerial(&sercom5, 6, 20, SERCOM_RX_PAD_2, UART_TX_PAD_0); // Create the new UART instance assigning it by arduino pin numbers

/**
 * Magic
 */
void SERCOM5_Handler()
{
  flowSerial.IrqHandler();
  sercom5timeStamp = micros();
}

unsigned long lastSample = 0;

int getAveFlow(float &xFlow, float &yFlow, float &qFlow)
{
  long age = micros() - lastSample;
  if (newestXFlow == 0.0 && newestYFlow == 0.0)
  {
    noFlowNoise++;
  } else
  {
    noFlowNoise = 0;
  }
  if (age > 50000 || newestQFlow < 71) // 50k microseconds, ie 50milliseconds
  {
    if (age > 50000)
    {
      Led::rgbColorSingleLed(4, 1.0, 1.0, 1.0);
    } else
    {
      Led::rgbColorSingleLed(4, 1.0, 0.0, 0.0);
    }
    return 0;
  }

  xFlow = 0.5 * xFlow + 0.5 * newestXFlow;
  yFlow = 0.5 * yFlow + 0.5 * newestYFlow;
  qFlow = newestQFlow;
  for (int n = 3; n < 5; n++)
  {
    float flowSquared = xFlow * xFlow + yFlow * yFlow;
    Led::hsvColorSingleLed(n, 120 + (flowSquared / 20.0));
  }
  return 1;
}

void processFlowPacket(int dx, int dy, int q, int t)
{
  lastSample = sercom5timeStamp;

  newestXFlow = dx; // Assumes flow sensor time is more stable than
  // measuring character timing. Will have to verify.

  newestYFlow = dy; // (1000 milliseconds per second, 40 milliseconds per loop,
  // Result is in arbitrary flow units per second.
  newestQFlow = q;
  newestTFlow = t;
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
  Led::hsvColorSingleLed(0, 0.0);
  addSym(&newestXFlow, "xFlow", "flow sensor", "1N");
  addSym(&newestYFlow, "yFlow", "flow sensor", "1N");
  addSym(&newestQFlow, "qFlow", "flow sensor", "0N");
  addSym(&newestTFlow, "tFlow", "flow sensor", "0N");
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
}