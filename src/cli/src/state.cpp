#include "../../state.h"

bool radioInitialized = false;
bool i2cBusSafe = false;
int mode;
int tookTooLongCount = 0;
bool readyToUpdateNeoPixels = false;
unsigned long nextNeoUpdate = 0;
unsigned long lastHeartBeat = 0;
int stuffQueued = 0;
int stuffUnqueued = 0;
int SPISlaveInterrupts = 0;
int SPISlaveCharactersRead = 0;
int SPISlaveCharactersWritten = 0;
int cortexState = STATE_CORTEX_ALIVE;
int voltageByte = 257;
float voltage = 4.2;
float showBrainStemLog = 0.0;
float showPacketLog = 0.0;
float showXn297LLog = 0.0;
float showUsageLog = 0.0;
float showLogLog = 0;
bool unreadPacket = false;
bool unprocessedPacket = false;
float showRanges = 0.0;
float showPID = 0.0;
float showVerbose = 0.0;
float byPassManipulation = 1.0;
int inputNum = 0;
int userNum = 0;
float rangesInM[5];
unsigned long rangesTS[RFINDERS][2];
float rangeHist[RFINDERS][HISTDEPTH];
float rangeHistTotal[RFINDERS];
unsigned long rangeHistTS[RFINDERS][HISTDEPTH];
float rangeVel[RFINDERS];
int lastR[RFINDERS];
float tState;
int lCnt = 0;
float errorList[MAX_ERROR_REPORTS]; // these are to hold codes associated with errors
int errorListIndex = 0;
float A = 0.0;
float B = 0.0;
float C = 0.0;
float D = 0.0;
float E = 0.0;
float F = 0.0;

float vbattfilt, vbatt_com, GEstG0, GEstG1, GEstG2, sample1;
