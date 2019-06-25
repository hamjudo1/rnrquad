//
// Created by Florian VanKampen on 2019-06-17.
//

#include <rnrquad.h>
#include "Hover.h"


void setup()
{
  baseSetup();
  setupThinking();
}

void loop()
{
  baseLoop();
  pollThinking();
}