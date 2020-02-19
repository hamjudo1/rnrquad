//
// Created by Florian VanKampen on 2019-07-22.
//

#ifndef RNRQUAD_LED_H
#define RNRQUAD_LED_H


enum color {
  red = 0x3f0000,
  yellow = 0x3f3f00,
  green = 0x003f00,
  cyan = 0x003f3f,
  blue = 0x00003f,
  magenta = 0x3f003f, 
  black = 0x000000,
  white = 0x3f3f3f,
} ;
const int NUMPIXELS = 5;  
class LedSet {
public:
  LedSet(color def);
  void show();
  void reset();
  void hsv(int dotNo, float h, float s, float v);
  void rgb(int dotNo, float r, float g, float b);
  void named(int dotNo, color name);
  void constant(int dotNo,uint32_t col);
  void blink(int dotNo,uint32_t col1, uint32_t col2);
  uint32_t dots[NUMPIXELS];
  uint32_t altdots[NUMPIXELS];
  uint32_t defColor;
  uint32_t dim(uint32_t);
  uint32_t bright(uint32_t);
};
class Led
{
public:
  static void setup();
  static void poll();
  static void hsvColorSingleLed(int dotNo, float angle);
  static void rgbColorSingleLed(int dotNo, float r, float g, float b);
  static uint32_t hsvColor(float h, float s, float v);
  static uint32_t rgbColor(float r, float g, float b);
};

#endif
