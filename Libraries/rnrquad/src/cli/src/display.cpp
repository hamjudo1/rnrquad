//
// Created by Florian VanKampen on 2019-06-20.
//
#include "../../state.h"

void setupDisplay()
{
//  addCmd(showUsageTime, "");
//  addCmd(sysStatus, "");
}

void showUsageTime()
{
  long totTime = debugTime + commTime + neoTime + rangeTime + thinkTime;
  float debugPercent = (debugTime * 100.0) / totTime;
  float commPercent = (commTime * 100.0) / totTime;
  float neoPercent = (neoTime * 100.0) / totTime;
  float rangePercent = (rangeTime * 100.0) / totTime;
  float thinkPercent = (thinkTime * 100.0) / totTime;
  Serial.print("debug %");
  Serial.print(debugPercent);
  Serial.print(" comm %");
  Serial.print(commPercent);
  Serial.print(" neo %");
  Serial.print(neoPercent);
  Serial.print(" range %");
  Serial.print(rangePercent);
  Serial.print(" think %");
  Serial.println(thinkPercent);
}

void sysStatus()
{
  Serial.print("Uptime ");
  Serial.print(millis());
  Serial.print("Count of take too long events ");
  Serial.println(tookTooLongCount);
  Serial.print("Queue stats queued: ");
  Serial.print(stuffQueued);
  Serial.print(", unqueued: ");
  Serial.println(stuffUnqueued);
  Serial.print("SPI slave interrupts ");
  Serial.println(SPISlaveInterrupts);
  Serial.print("Packets Received ");
  Serial.println(packetrx);
  Serial.print("registers written ");
  Serial.print(xn297_regs_written);
  Serial.print(", read ");
  Serial.println(xn297_regs_read);
  Serial.print("Status values seen ");
  for (int i = 0; i < 256; i++)
  {
    if (statusValCount[i])
    {
      Serial.print(i);
      Serial.print(":");
      Serial.print(statusValCount[i]);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.print("Status check count ");
  Serial.println(stem_status_check_count);

  if (radioInitialized)
  {
    Serial.println("Radio initialized");
  } else
  {
    Serial.println("Radio NOT initialized");
  }
}

void dispPacketLog()
{
  Serial.print("Packets ");
  Serial.print(packetrx);
  Serial.print("rx array: ");
  for (int rxN = 0; rxN < 4; rxN++)
  {
    Serial.print(rx[rxN]);
    if (rxN < 3)
    {
      Serial.print(", ");
    }
  }

  Serial.print(" aux array: ");
  for (int aN = 0; aN < AUXNUMBER; aN++)
  {
    Serial.print((int) aux[aN]);
  }
  Serial.print(" trims array: ");
  for (int i = 0; i < 4; i++)
  {
    Serial.print((int) trims[i]);
    Serial.print(" ");
  }
  Serial.println();
  for (int b = 0; b < 15; b++)
  {
    Serial.print(rxdata[b]);
    Serial.print(" ");
  }
  Serial.println();
}

void dispXn297LLog()
{
  Serial.println("Please make dispXn297LLog() into a real function");
}