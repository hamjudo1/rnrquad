//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_RGBLED_H
#define RNRQUAD_RGBLED_H


class RgbLed
{
public:
  RgbLed();

  float red;
  float green;
  float blue;

  RgbLed setRgb(float r, float g, float b);

  RgbLed setHsv(float hue, float saturation, float value);
};


#endif
