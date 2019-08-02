//
// Created by Florian VanKampen on 2019-06-20.
//
#include "state.h"

float throttleUpTime = 0.0;  // Timestamp when we were last on the ground with throttle down.
float notokay = 0.0;

void setupUtility() {
  addSym(&notokay, "notokay", "reason given by okayToFly() for not flying", "3");
}

float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 allocate space for a copy of a string.
 These strings should be freed with free() if
 they aren't intended to be permanent.
 */
char *dupString(char *origString)
{
  char *newString = (char *) malloc(strlen(origString) + 1);
  strcpy(newString, origString);
  return newString;
}

char *catString2(char *origString1, char *origString2)
{
  int newLen = 1 + strlen(origString1) + strlen(origString2);
  char *newString = (char *) malloc(newLen);
  char *n = newString;
  for (; *origString1; origString1++)
  {
    *n++ = *origString1;
  }
  for (; *origString2; origString2++)
  {
    *n++ = *origString2;
  }
  *n = '\0';
  return newString;
}

// strip whitespace in place, (space, tab, linefeed, CR), from beginning and end of a null terminated string.
void stripW(char *s)
{
  char *r, *w, *l; // r: read position, w: write position, l: last non-whitespace in string.
  r = s;
  while (*r == '\t' || *r == '\r' || *r == ' ' || *r == '\n')
  {
    r++;
  }
  l = r;
  if (r == s)
  { // nothing stripped from front of string.
    while (*r)
    {
      if (!(*r == '\t' || *r == '\r' || *r == ' ' || *r == '\n'))
      {
        l = r + 1;
      }
      r++;
    }
  } else
  { // leading whitespace removed, must shift characters.
    w = s;
    l = w;
    while (*r)
    {
      *w = *r;  // Copy each character
      if (!(*r == '\t' || *r == '\r' || *r == ' ' || *r == '\n'))
      {
        l = w + 1;
      }
      r++;
      w++;
    }
  }
  *l = '\0';
}

float seconds() {
  return (float)(micros()*0.000001);
}

float age(float timestamp) {
  return seconds()-timestamp;
}
float flightTime () {
  return seconds() - throttleUpTime;
}
bool okayToFly()
{
  static bool throttleDownSinceLastEvent = false;
  notokay = 0;
  if (rx[3] < 0.1)
  {
    throttleDownSinceLastEvent = true;
    notokay = 1.0; // + event;
  } else if (!throttleDownSinceLastEvent)
  {
    notokay = 2.0; // + event;
  } else if (voltage < 3.1)
  {
    notokay = 3.0; // + event;
    throttleDownSinceLastEvent = false;
    redSet.blink(2,red,green);
    redSet.blink(3,green,red);

  }
  if ( notokay ) {
    throttleUpTime = seconds(); // Actually throttle up time is the last time the throttle wasn't up.
    return false;
  }
  return true;
}
