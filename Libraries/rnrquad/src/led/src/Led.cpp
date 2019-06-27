
#include "../../state.h"

const int DATAPIN = 17;
const int CLOCKPIN = 18;
Adafruit_DotStar pixel = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

// HSV code inspired by https://en.wikipedia.org/wiki/HSL_and_HSV
// Note, this version is suboptimal to make it easier for me to explain. There
// are better versions out there.
//
uint32_t hsvColor(float h, float s, float v)
{  // h is 0.-360., s and v are 0 to 1.0
  // h is Hue, s is Saturation, v is Value.
  float r = 0., g = 0., b = 0.;
  // 360 degree color wheel, fmod() stands for Floating point MODulo. It divides the first
  // number by the second number and returns the remainder, but with the sign the same as
  // the original.

  h = fmod(h, 360.0);
  if (h < 0.0)
  { //
    h = 360.0 - h;
  }

  float c = v * s;
  float hPrime = fmod(h, 60.0) / 60.0;
  float x = c * hPrime;

  if (h < 60.)
  {
    r = c;
    g = x * c;
  } else if (h < 120.)
  {
    r = c - x * c;
    g = c;
  } else if (h < 180)
  {
    g = c;
    b = x;
  } else if (h < 240)
  {
    g = c - x;
    b = c;
  } else if (h < 300)
  {
    r = x;
    b = c;
  } else
  {
    r = c;
    b = c - x;
  }
  float m = v - c;
  r = constrain(r + m, 0.0, 1.0);
  g = constrain(g + m, 0.0, 1.0);
  b = constrain(b + m, 0.0, 1.0);
  float brightness = 50.0;
  return pixel.Color(int(r * brightness), int(g * brightness), int(b * brightness));
}

void setupNeoSupp()
{
  int r = 50, g = 200, b = 50;
  pixel.begin();
  for (int n = 0; n < NUMPIXELS; n++)
  {
    pixel.setPixelColor(n, pixel.Color(r, g, b));
    delay(200);
    pixel.show();
  }
  r = 50;
  g = 50;
  b = 50;
  for (int n = 0; n < NUMPIXELS; n++)
  {
    pixel.setPixelColor(n, pixel.Color(r, g, b));
    delay(200);
    pixel.show();
  }
}

uint32_t shortRed(int pixNo, int phaseNo, void *options)
{
  if (phaseNo == 0)
  {
    return pixel.Color(40, 0, 0);
  } else if (phaseNo > 0)
  {
    return pixel.Color(0, 0, 0);
  } else
  {
    Serial.print("shortRed");
  }
}

uint32_t longRed(int pixNo, int phaseNo, void *options)
{
  if (phaseNo == 0)
  {
    return pixel.Color(0, 0, 0);
  } else
  {
    return pixel.Color(30, 0, 0);
  }
}

const int rxLEDOffset = 0;
extern float colorAngle;
bool everyOther = false;
int pixelPhase = 0;
int subPhase = 0;

void colorSingleDotA(int dotNo, float angle)
{
  pixel.setPixelColor(dotNo, hsvColor(angle, 1.0, 1.0));
  pixel.show();
}

void colorSingleDot(int dotNo, float angle)
{
  pixel.setPixelColor(dotNo, hsvColor(angle, 1.0, 1.0));
  pixel.show();
}

void rgbSingleDot1(int dotNo, float r, float g, float b)
{
  r = constrain(r, 0.0, 1.0);
  g = constrain(g, 0.0, 1.0);
  b = constrain(b, 0.0, 1.0);
  pixel.setPixelColor(dotNo, pixel.Color(int(255 * r), int(255 * g), int(255 * b)));
  pixel.show();
}

void showSmallInt(int iVal)
{
  int mod5 = iVal % 5;
  int div5 = iVal / 5;
  float color2 = div5 * 60;
  float color1 = color2 + 60;
  int dot = 0;
  for (; dot < mod5; dot++)
  {
    colorSingleDotA(dot, color1);
  }
  for (; dot < 5; dot++)
  {
    colorSingleDotA(dot, color2);
  }
}


