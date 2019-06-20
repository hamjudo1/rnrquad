
#include "state.h"

const long minNeoUpdateDelay = 40; // Wait at least 75ms before updating neopixel array.
#define NUMPIXELS  5
const int NUMPHASES = 3;
const int SUBPHASES = 5;

class neoRule {
public:
  uint32_t (*fun_ptr)(int pixNo, int phaseNo, void *options);

  void *opt;
};

#ifdef USE_DOTSTAR
#define DATAPIN    17
#define CLOCKPIN   18
Adafruit_DotStar pixel = Adafruit_DotStar(
    NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

#endif
#if NUMPIXELS != 0
neoRule rules[NUMPHASES][NUMPIXELS];
#endif

uint32_t filler(int pixNo, int phaseNo, void *options) {
  if (phaseNo < 0) {
    Serial.print("filler");
  } else {
    float angle = (phaseNo * 360.0) / NUMPHASES + pixNo * 15;
    return hsvColor(angle, 1.0, 1.0);
  }
}

typedef struct frStruct {
  float *val;
  float minval;
  float maxval;
  float minC;
  float maxC;
};

float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t fixedColor(int pixNo, int phaseNo, void *options) {
  if (phaseNo < 0) {
    Serial.print("fixedColor");
  }
  return *(uint32_t *) options;
}

void *permCopyColorAsVoid(uint32_t color) {
  uint32_t *permCopy = (uint32_t *) malloc(sizeof(uint32_t));
  *permCopy = color;
  return (void *) permCopy;
}

void setFixedColor(int pixNo, uint32_t color) {
  setPixRule(&fixedColor, pixNo, -1, permCopyColorAsVoid(color));
}

uint32_t showFloat(int pixNo, int phaseNo, void *options) {
  if (phaseNo < 0) {
    Serial.print("showFloat");
  } else {
    frStruct *fs = (frStruct *) options;
    float color = mapf(*(fs->val), fs->minval, fs->maxval, fs->minC, fs->maxC);
    return hsvColor(color, 1.0, 1.0);
  }
}
// floatRange() the specified pixel's color will change smoothly from minColor to maxColor
// as *v changes from minv to maxv.

void floatRange(int pixNo, float *v, float minv, float maxv, float minColor, float maxColor) {
  if (pixNo < NUMPIXELS) {
    frStruct *fs = (frStruct *) malloc(sizeof(frStruct));
    fs->val = v;
    fs->minval = minv;
    fs->maxval = maxv;
    fs->minC = minColor;
    fs->maxC = maxColor;
    setPixRule(&showFloat, pixNo, -1, (void *) fs);
  }
}

uint32_t showRange(int pixNo, int phaseNo, void *options) {
  if (pixNo < NUMPIXELS) {
    if (phaseNo < 0) {
      Serial.print("showRange");
    } else {
      float angle;
      float range = *(float *) options;
      int rangeStep = (int) (range * 5);
      if (rangeStep > 5) {
        rangeStep = 5;
      }
      angle = rangeStep * 60;
      /* Serial.print("showRange(");
        Serial.print(pixNo);
        Serial.print(", ");
        Serial.print(phaseNo);
        Serial.print(", ");
        Serial.print(range,3);
        Serial.print (")  ");
        Serial.print(rangeStep);
        Serial.print(", ");
        Serial.println(angle); */


      return hsvColor(angle, 1.0, 1.0);
    }
  }
  return 0;
}

#ifndef USE_DOTSTAR
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, J5X5, NEO_GRB + NEO_KHZ800);
#endif

// HSV code inspired by https://en.wikipedia.org/wiki/HSL_and_HSV
// Note, this version is suboptimal to make it easier for me to explain. There
// are better versions out there.
//
uint32_t hsvColor(float h, float s, float v) {  // h is 0.-360., s and v are 0 to 1.0
  // h is Hue, s is Saturation, v is Value.
  float r = 0., g = 0., b = 0.;
  // 360 degree color wheel, fmod() stands for Floating point MODulo. It divides the first
  // number by the second number and returns the remainder, but with the sign the same as
  // the original.

  h = fmod(h, 360.0);
  if (h < 0.0) { //
    h = 360.0 - h;
  }

  float c = v * s;
  float hPrime = fmod(h, 60.0) / 60.0;
  float x = c * hPrime;

  if (h < 60.) {
    r = c;
    g = x * c;
  } else if (h < 120.) {
    r = c - x * c;
    g = c;
  } else if (h < 180) {
    g = c;
    b = x;
  } else if (h < 240) {
    g = c - x;
    b = c;
  } else if (h < 300) {
    r = x;
    b = c;
  } else {
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

void setupNeoSupp() {
  int r = 50, g = 200, b = 50;
  pixel.begin();
  for (int n = 0; n < NUMPIXELS; n++) {
    pixel.setPixelColor(n, pixel.Color(r, g, b));
    delay(200);
    pixel.show();
  }
  r = 50;
  g = 50;
  b = 50;
  for (int n = 0; n < NUMPIXELS; n++) {
    pixel.setPixelColor(n, pixel.Color(r, g, b));
    delay(200);
    pixel.show();
  }


#if NUMPIXELS != 0
  Serial.print(NUMPIXELS);
#ifdef USE_DOTSTAR
  Serial.println(" dotstars");
#else
  Serial.println(" Neopixels");
#endif
  for (int pixNo = 0; pixNo < NUMPIXELS; pixNo++) {
    for (int phaseNo = 0; phaseNo < NUMPHASES; phaseNo++) {
      rules[phaseNo][pixNo].fun_ptr = &filler;
      rules[phaseNo][pixNo].opt = (void *) NULL;
    }
  }
#endif
}

uint32_t shortRed(int pixNo, int phaseNo, void *options) {
  if (phaseNo == 0) {
    return pixel.Color(40, 0, 0);
  } else if (phaseNo > 0) {
    return pixel.Color(0, 0, 0);
  } else {
    Serial.print("shortRed");
  }
}

uint32_t longRed(int pixNo, int phaseNo, void *options) {
  if (phaseNo == 0) {
    return pixel.Color(0, 0, 0);
  } else {
    return pixel.Color(30, 0, 0);
  }
}

void setPixRule(uint32_t (*fp)(int, int, void *), int pixNo, int phaseNo, void *options) {
  if (pixNo < NUMPIXELS) {
    if (phaseNo == -1) {
      for (int i = 0; i < NUMPHASES; i++) {
        rules[i][pixNo].fun_ptr = fp;
        rules[i][pixNo].opt = options;
      }
    } else {
      rules[phaseNo][pixNo].fun_ptr = fp;
      rules[phaseNo][pixNo].opt = options;
    }
  }
}

const int rxLEDOffset = 0;
extern float colorAngle;
boolean everyOther = false;
int pixelPhase = 0;
int subPhase = 0;

void NeoUpdate() {
#if   NUMPIXELS > 0
  if (false) {
    unsigned long now = millis();
    // We switched from Neopixels to Dotstars.
    // The Neopixel code was extra complex because Neopixels need strict timing and disable interrupts.
    //There are still echos from that complexity.
    // pixel.show() disables interrupts, so it can only be called when the cortex isn't talking.
#if WHITEBOARD_WIRING == 1
    readyToUpdateNeoPixels = false;
    nextNeoUpdate = now + minNeoUpdateDelay;
#endif
    subPhase = (subPhase + 1) % SUBPHASES;
    if (subPhase == 0) {
      pixelPhase = (pixelPhase + 1) % NUMPHASES;
    }
    for (int pixNo = 0; pixNo < NUMPIXELS; pixNo++) {
      if (rules[pixelPhase][pixNo].fun_ptr != NULL) {
        pixel.setPixelColor(pixNo, rules[pixelPhase][pixNo].fun_ptr(pixNo, pixelPhase, rules[pixelPhase][pixNo].opt));
      }
    }
    pixel.show();
  }

#endif

}

void colorSingleDotA(int dotNo, float angle) {
  if (greenBoard && true) {
    pixel.setPixelColor(dotNo, hsvColor(angle, 1.0, 1.0));
    pixel.show();
  }
}

void colorSingleDot(int dotNo, float angle) {
  if (greenBoard && true) {
    //  Serial.print("dot");
    //  Serial.print(dotNo);
    //  Serial.print(" ");
    //up;  Serial.println(angle,0);
    pixel.setPixelColor(dotNo, hsvColor(angle, 1.0, 1.0));
    pixel.show();
  }
}

void rgbSingleDot1(int dotNo, float r, float g, float b) {
  r = constrain(r, 0.0, 1.0);
  g = constrain(g, 0.0, 1.0);
  b = constrain(b, 0.0, 1.0);
  pixel.setPixelColor(dotNo, pixel.Color(int(255 * r), int(255 * g), int(255 * b)));
  pixel.show();
}

void pollNeoSupp() {
#if WHITEBOARD_WIRING == 1
  unsigned long now = millis();
  if ( now > nextNeoUpdate ) { // Set a flag

    readyToUpdateNeoPixels = true;
    if ( now > nextNeoUpdate + 1000) {
      NeoUpdate();
    }
  }
#else
  NeoUpdate();
#endif
}

void showSmallInt(int iVal) {
  int mod5 = iVal % 5;
  int div5 = iVal / 5;
  float color2 = div5 * 60;
  float color1 = color2 + 60;
  int dot = 0;
  for (; dot < mod5; dot++) {
    colorSingleDotA(dot, color1);
  }
  for (; dot < 5; dot++) {
    colorSingleDotA(dot, color2);
  }
}


