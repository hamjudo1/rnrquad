#include "../../vl53l0x/src/Adafruit_VL53L0X.h"

// i2c pin 31 SDA Arduino20 (default for wire library)
// i2c pin 32 SCL Arduino21 (likewise)

#include "../../state.h"

Adafruit_VL53L0X loxes[RFINDERS] = {
    Adafruit_VL53L0X(),
    Adafruit_VL53L0X(),
    Adafruit_VL53L0X(),
    Adafruit_VL53L0X(),
    Adafruit_VL53L0X()
};

#define TCAADDR 0x74
// J5 1 CPU pin 12 Port PA07 Arduino Pin 9.
// J5 2 CPU pin 11 Port PA06 Arduino Pin 8.
// J5 3 CPU pin 10 Port PA05 Arduino Pin 18.
// J5 4 CPU pin 9 Port PA04 Arduino Pin 17.
// J5 5 CPU pin 8 Port PB09 Arduino Pin 16.
const int J5X1 = 9;
const int J5X2 = 8;
const int J5X3 = 18;
const int J5X4 = 17;
const int J5X5 = 16;

// J5 connects the range finders and NEO pixels.
// J5X1 Yellow to “UP”, Arduino 9
// J5X2 White to “LEFT”, Arduino 8
// J5X3 Blue to “DOWN”, Arduino 18
// J5X4 Green to “FRONT”, Arduino 17
// J5X5 Blue to Neopixel array, Arduino 16
// xshut on "right" range finder is not connected.

rangeConfigElem_t rangeConfig[] = {
    {-1, true, "front", 0},
    {7,  true, "down",  0},
    {0,  true, "right", 0},
    {19, true, "left",  0},
    {16, true, "up",    0},
};