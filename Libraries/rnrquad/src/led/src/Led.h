//
// Created by Florian VanKampen on 2019-07-22.
//

#ifndef RNRQUAD_LED_H
#define RNRQUAD_LED_H

enum LedMode
{
  manual,
  debug,
  error,
};

class Led
{
public:
  static void setupNeoSupp();
  static void hsvColorSingleLed(int dotNo, float angle);
  static void rgbColorSingleLed(int dotNo, float r, float g, float b);
  static LedMode ledMode;
  static uint32_t hsvColor(float h, float s, float v);
};

#endif
