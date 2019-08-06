#ifndef RNRQUAD_UTILITY_H
#define RNRQUAD_UTILITY_H

class Utility
{
  public:
  static char *dupString(char *origString);
  static char *catString2(char *origString1, char *origString2);
  static void stripW(char *s);
  static float mapf(float val, float in_min, float in_max, float out_min, float out_max);
  static float time();
};

#endif