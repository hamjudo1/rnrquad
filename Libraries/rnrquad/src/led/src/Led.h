//
// Created by Florian VanKampen on 2019-07-22.
//

#ifndef RNRQUAD_LED_H
#define RNRQUAD_LED_H


enum color {
  red = 0xff0000,
  yellow = 0xffff00,
  green = 0x00ff00,
  cyan = 0x00ffff,
  blue = 0x0000ff,
  magenta = 0xff00ff, 
} ;
const int NUMPIXELS = 5;  
class LedSet {
public:
  LedSet(color def);
  void hsv(int dotNo, float h, float s, float v);
  void rgb(int dotNo, float r, float g, float b);
  void named(int dotNo, color name);
  uint32_t dots[NUMPIXELS];
};
class Led
{
public:
  static void setup();
  static void poll();
  static void hsvColorSingleLed(int dotNo, float angle);
  static void rgbColorSingleLed(int dotNo, float r, float g, float b);
  static uint32_t hsvColor(float h, float s, float v);
};

#endif
