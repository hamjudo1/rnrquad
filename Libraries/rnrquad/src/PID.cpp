// Left - Right, Up - Down, Front - Back PID loops
simplePID lrPID, udPID, fbPID;

void pidInit()
{
  lrPID.begin("lr", &lrError, &lrOutput, &lrKP, &lrKI, &lrKD);
  udPID.begin("ud", &udError, &udOutput, &udKP, &udKI, &udKD);
  fbPID.begin("fb", &fbError, &fbOutput, &fbKP, &fbKI, &fbKD);
}