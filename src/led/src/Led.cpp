
#include "../../state.h"

const int DATAPIN = 17;
const int CLOCKPIN = 18;

Adafruit_DotStar pixel = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

LedSet::LedSet (color def) {
  defColor = def;
  for(int i=0; i<NUMPIXELS; i++) {
    dots[i] = defColor;
    altdots[i] = defColor;
  }
}
void LedSet::show() {
  unsigned long now = millis() % 1000;
  if ( (seconds() - refControl.leftTriggerTime) < 1.5 ) {
    for(int i=0; i<NUMPIXELS; i++) {
      pixel.setPixelColor(i,defColor);
    }
  } else {
    for(int i=0; i<NUMPIXELS; i++) {
      if ( now < 500 ) {
        pixel.setPixelColor(i, dots[i]);
      } else {
        pixel.setPixelColor(i,altdots[i]);
      }
    }
  }
  pixel.show();
}
void LedSet::blink(int pixelNo, uint32_t col1, uint32_t col2) {
  int pixNo = constrain(pixelNo,0,5);
  dots[pixNo] = col1;
  altdots[pixNo] = col2;
}
void LedSet::constant(int pixelNo, uint32_t col1) {
  blink(pixelNo, col1, col1);
}


// HSV code inspired by https://en.wikipedia.org/wiki/HSL_and_HSV
// Note, this version is suboptimal to make it easier for me to explain. There
// are better versions out there.
//
uint32_t Led::hsvColor(float h, float s, float v)
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
  float brightness = 255.0;
  return pixel.Color(int(r * brightness), int(g * brightness), int(b * brightness));
}

LedSet redSet(red), yellowSet(yellow), greenSet(green), cyanSet(cyan), blueSet(blue), magentaSet(magenta);

uint32_t flowToCol(float flowRate) {
  return Led::hsvColor((1+flowRate)*120.0,1.0,0.5);
}
uint32_t rangeToCol(float range) {
  if ( range < 0.01 ) {
     return 0x808080;
  } else {
    return Led::hsvColor(range*183.0,1.0,0.5);
  }
}
void Led::poll() {
  int choice = (int)(refControl.leftTrigger) % 6;

  switch (choice) {
  case 0:
     redSet.show();
     break;
  case 1:
     yellowSet.constant(0,rangeToCol(refSensor.rangeUp));
     yellowSet.constant(1,rangeToCol(refSensor.rangeRight));
     yellowSet.constant(2,rangeToCol(refSensor.rangeForward));
     yellowSet.constant(3,rangeToCol(refSensor.rangeDown));
     yellowSet.constant(4,rangeToCol(refSensor.rangeLeft));
     yellowSet.show();
     break;
  case 2:
     if ( refSensor.flowQ < 72 ) {
       for(int i=0;i<NUMPIXELS;i++) {
	 greenSet.blink(i,green,0);
	}
      } else {
	 greenSet.constant(0,flowToCol(refSensor.flowY));
	 greenSet.constant(1,flowToCol(-refSensor.flowX));
	 greenSet.constant(2,flowToCol(-refSensor.flowY));
	 greenSet.constant(3,flowToCol(-refSensor.flowY));
	 greenSet.constant(4,flowToCol(refSensor.flowX));
      }
     greenSet.show();
     break;
  case 3:
     cyanSet.show();
     break;
  case 4:
     blueSet.show();
     break;
  case 5:
     magentaSet.show();
     break;
   }
}
void Led::setup()
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
uint32_t bright(uint32_t color) {
  int r = (color >> 16) & 0xff;
  int g = (color >> 8) & 0xff;
  int b = (color) & 0xff;
  return pixel.Color(r*2, g*2, b*2);
}
uint32_t dim(uint32_t color) {
  int r = (color >> 16) & 0xff;
  int g = (color >> 8) & 0xff;
  int b = (color) & 0xff;
  return pixel.Color(r/2, g/2, b/2);
}
uint32_t verydim(uint32_t color) {
  int r = (color >> 16) & 0xff;
  int g = (color >> 8) & 0xff;
  int b = (color) & 0xff;
  return pixel.Color(r/8, g/8, b/8);
}
void Led::hsvColorSingleLed(int dotNo, float angle)
{
  pixel.setPixelColor(dotNo, hsvColor(angle, 1.0, 1.0));
  pixel.show();
}

void Led::rgbColorSingleLed(int dotNo, float r, float g, float b)
{
  r = constrain(r, 0.0, 1.0);
  g = constrain(g, 0.0, 1.0);
  b = constrain(b, 0.0, 1.0);
  pixel.setPixelColor(dotNo, pixel.Color(int(255 * r), int(255 * g), int(255 * b)));
  pixel.show();
}
