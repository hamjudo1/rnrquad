//
// Created by Florian VanKampen on 2019-06-17.
//

#include "RgbLed.h"

RgbLed::RgbLed ()
{
    red = 0;
    green = 0;
    blue = 0;
}

RgbLed RgbLed::setRgb(float r, float g, float b)
{
    red = r;
    green = g;
    blue = b;
}

RgbLed RgbLed::setHsv(float hue, float saturation, float value)
{
    //TODO
}