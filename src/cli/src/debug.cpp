//
// Created by Florian VanKampen on 2019-06-20.
//

#include "../../state.h"


void setupDebug()
{
  addSym(&showBrainStemLog, "blog", "FLAG showBrainStemLog", "F");
  addSym(&showPacketLog, "xlog", "FLAG showPacketLog", "F");
  addSym(&showLogLog, "llog", "FLAG showLogLog", "F");
  addSym(&showUsageLog, "ulog", "FLAG showUsageLog", "F");
  addSym(&byPassManipulation, "bypass", "FLAG byPassManipulation", "F");
  addSym(&voltage, "volts", "", "3N");
  addCmd(sysStatus, "stats", "show some debugging stats", NULL);
  addCmd(freeRam, "free", "report available RAM", NULL);
}

void pollDebug()
{
  static int lCnt = 0;
  lCnt++;
  if (lCnt % 40 == 0)
  {
    if (showUsageLog)
    {
      showUsageTime();
    }
  }
  pollWatch();
  while (Serial.available())
  {
    int c = Serial.read();
    processChar(c);
  }
}

void freeRam()
{
  unsigned char *blocks[32];
  int i, allocCnt;
  for (i = 0; i < 32; i++)
  {
    unsigned char *block = (unsigned char *) malloc(1024);
    if (block == (unsigned char *) NULL)
    {
      for (int j = 0; j < i; j++)
      {
        free(blocks[j]);
      }
      Serial.print(i);
      Serial.println("k bytes were available.");
      return;
    }
    blocks[i] = block;
  }
  Serial.println("We cannot get here, but we did. Does this processor have more RAM?");
}