// Symbol table library. Names can be given to regular floating point variables, then
// those variables can be displayed, and changed. Each variable has a display width to
// control the number of trailing digits. This is set by including a digit 0 - 9 in the
// option string. If an entry has an "L" in the option string, the variable will be logged
// at each timestep.
#include <Arduino.h>
#include "../../state.h"

typedef struct entry
{
  float *value; // Pointer to value being logged.
  float lastV;  // Last value logged, if unchanged, no need to log it again.
  const char *name;
  const char *description;
  const char *options;
  entry *next;
  int8_t trailingDigits; // 0+ number of trailing digit: -1 bool (on/off)
  uint8_t logging; // 0: never logged, 1: logged now, 2: logged, but not now.
  uint8_t tag;
};
typedef struct cmdEntry
{
  void (*fun_ptr)(void);

  const char *name;
  const char *description;
  const char *options;
  cmdEntry *next;
};

entry *head = new entry;
entry *tail = head;
const int MAXSYMS = 256;
entry *tagIndex[MAXSYMS];
entry *lastCheck = NULL;
cmdEntry *cmdHead = new cmdEntry;
cmdEntry *cmdTail = cmdHead;
const int LOGSIZE = 1024;
float dataLog[LOGSIZE];
uint8_t tagLog[LOGSIZE];
float watchTime = 0.5;
int lp = 0;

void displayVariable(float val, int trailingDigits)
{
  if (trailingDigits == -1)
  {
    if (val)
    {
      Serial.print("ON");
    } else
    {
      Serial.print("off");
    }
  } else
  {
    Serial.print(val, trailingDigits);
  }
}

void initTable()
{
  head->value = (float *) NULL;
  head->lastV = 0.0;
  head->name = "Millis";
  head->description = "The list of symbols";
  head->next = (entry *) NULL;
  head->logging = 2;
  head->trailingDigits = 0;
  head->tag = 0;
  tagIndex[0] = head;
}

// call logSyms() at each timestep.
void logSyms()
{
  unsigned long now = millis();
  if (lp >= LOGSIZE)
  {
    return;
  }
  tagLog[lp] = 0;
  dataLog[(lp++)] = (float) now;
  entry *p = head->next;
  int tagNo = 1;
  while (p != (entry *) NULL)
  {
    if (p->logging == 2)
    {
      if (*(p->value) != p->lastV)
      {
        if (tagNo != p->tag)
        {
          Serial.print("tag out of sync ");
          Serial.print(tagNo);
          Serial.print(" ");
          Serial.println(p->tag);
        }
        tagLog[lp] = tagNo;
        dataLog[(lp++)] = *(p->value);
        p->lastV = *(p->value);
      }
      if (lp >= LOGSIZE)
      {
        return;
      }
    }
    p = p->next;
    tagNo++;
  }
  if (showLogLog)
  {
    Serial.print(" ");
    Serial.println(now);
  }
}

// dump logged data as a CSV file
entry *v;

void printEndOfRecord()
{
  if (v == (entry *) NULL)
  {
    Serial.println(",,,");
  } else
  {
    Serial.print(",");
    Serial.print(v->name);
    Serial.print(",");
    displayVariable(*(v->value), v->trailingDigits);
    Serial.print(",\"");
    Serial.print(v->description);
    Serial.println("\"");
    v = v->next;
  }
}

void dumpLog()
{
  entry *p = head;
  v = head->next;
  Serial.println();
  Serial.print(p->name);
  Serial.print(" ");
  Serial.println(p->tag);
  p = p->next;
  while (p != (entry *) NULL)
  {
    if (p->logging > 0)
    {
      Serial.print(p->name);
      Serial.print(" ");
      Serial.println(p->tag);
      p->lastV = 0.0;
    }
    p = p->next;
  }
  int lmax = lp;
  if (lp > LOGSIZE)
  {
    lmax = LOGSIZE;
  }
  for (int l = 0; l < lmax; l++)
  {
    Serial.print(tagIndex[tagLog[l]]->name);
    Serial.print(" ");
    Serial.print(tagLog[l]);
    Serial.print(" ");
    Serial.println(dataLog[l]);
  }
}

void printLog()
{
  int record = 0;
  int lmax = lp;
  int tagNo = 0;
  entry *p = head;
  v = head->next;
  Serial.println();
  Serial.print(p->name);
  p = p->next;
  while (p != (entry *) NULL)
  {
    if (p->logging > 0)
    {
      Serial.print(",");
      Serial.print(p->name);
      p->lastV = 0.0;
    }
    p = p->next;
    if (p == (entry *) NULL)
    {
      Serial.println(",name,value,description");
    }
  }
  if (lp == 0)
  {
    Serial.print("No data logged yet!");
    return;
  }
  if (lmax > LOGSIZE)
  {
    lmax = LOGSIZE;
  }
  p = head;
  tagNo = 0;
  for (int l = 0; l < lmax; l++)
  {
    if (showLogLog)
    {
      Serial.print("Tag ");
      Serial.print(tagLog[l]);
      Serial.print(" Val ");
      Serial.println(dataLog[l]);
    } /*
    if ( tagLog[l] == 0 ) {
      p = head;
      tagNo = 0;
      displayVariable(dataLog[l], p->trailingDigits);
    } else { */
    while (tagNo != tagLog[l])
    {
      if (p == (entry *) NULL)
      {
        printEndOfRecord();
        tagNo = 0;
        p = head;
      } else if (p->logging > 0)
      {
        Serial.print(",");
        displayVariable(p->lastV, p->trailingDigits);
        p = p->next;
        tagNo++;
      } else
      {
        p = p->next;
        tagNo++;
      }
    }
    if (tagNo != 0)
    {
      Serial.print(",");
    }
    displayVariable(dataLog[l], p->trailingDigits);
    p->lastV = dataLog[l];
    p = p->next;
    tagNo++;
  }
  Serial.println();
}

void addCmd(void (*cmd_ptr)(void), const char *n, const char *d, char *opt)
{
  cmdEntry *newCmd = new cmdEntry;
  newCmd->fun_ptr = cmd_ptr;
  newCmd->name = n;
  newCmd->description = d;
  newCmd->options = opt;
  newCmd->next = (cmdEntry *) NULL;
  cmdTail->next = newCmd;
  cmdTail = newCmd;
}

int symCount = 0;

void addSym(float *v, const char *n, const char *d, const char *opt)
{
  symCount++;
  entry *newSym = new entry;
  newSym->value = v;
  newSym->name = n;
  newSym->description = d;
  newSym->trailingDigits = 1;
  newSym->logging = 0;
  newSym->tag = symCount;
  if (symCount > MAXSYMS)
  {
    while (1)
    {
      Serial.println("Too many symbols");
      delay(1000);
    }
  }
  tagIndex[symCount] = newSym;
  for (int op = 0; opt[op]; op++)
  {
    if (opt[op] >= '0' && opt[op] <= '9')
    {
      newSym->trailingDigits = opt[0] - '0';
    } else if (opt[op] == 'L')
    { // Log in the future
      newSym->logging = 1;
    } else if (opt[op] == 'N')
    { // log Now
      newSym->logging = 2;
    } else if (opt[op] == 'F')
    { // flag, display as On/Off.
      newSym->trailingDigits = -1;
    }
  }
  newSym->next = (entry *) NULL;
  newSym->options = opt;
  tail->next = newSym;
  tail = newSym;

  if (showLogLog)
  {
    Serial.print(symCount);
    Serial.print(" ");
    Serial.println(n);
  }
}

void addSym(float *v, const char *n, const char *d)
{
  addSym(v, n, d, "1");
}

cmdEntry *getCmdEntry(char *name)
{
  cmdEntry *p = cmdHead->next;
  while (p != (cmdEntry *) NULL)
  {
    if (strcmp(p->name, name) == 0)
    {
      return p;
    }
    p = p->next;
  }
  return (cmdEntry *) NULL;
}

bool runCmd(char *name)
{
  cmdEntry *p = getCmdEntry(name);
  if (p != (cmdEntry *) NULL && p->fun_ptr != NULL)
  {
    p->fun_ptr();
    return true;
  }
  return false;
}

entry *getSymEntry(char *name)
{
  entry *p = head->next;
  while (p != (entry *) NULL)
  {
    if (strcmp(p->name, name) == 0)
    {
      return p;
    }
    p = p->next;
  }
  return (entry *) NULL;
}

// Given a symbol name, return the pointer to variable.
float *getSymPtr(char *name)
{
  entry *p = head->next;
  while (p != (entry *) NULL)
  {
    if (strcmp(p->name, name) == 0)
    {
      return p->value;
    }
    p = p->next;
  }
  return (float *) NULL;
}

void listTable()
{
  entry *p = head->next;
  while (p != (entry *) NULL)
  {
    Serial.print(p->name);
    Serial.print(" ");
    displayVariable(*(p->value), p->trailingDigits);
    Serial.print(" ");
    Serial.println(p->description);
    p = p->next;
  }
}

const int watchListSize = 10;
entry *watchList[watchListSize];
int watching = 0;

void clearWatch()
{
  for (int i = 0; i < watchListSize; i++)
  {
    watchList[i] = (entry *) NULL;
  }
  watching = 0;
}
void quiet() {
  clearWatch();
  showPacketLog = 0.0;
  showUsageLog = 0.0;
  showXn297LLog = 0.0;
  showBrainStemLog = 0.0;
  showLogLog = 0.0;
}
// One of the early characters transmitted during an attempted download handshake is the "#", which is not
// otherwise used in our interface. Thus it is interpretted as a shutoff all printing to allow reprogramming.
// Note that it does not require a trailing carriage return or linefeed.
void downloadAttemptDetected() {
  quiet();
}

void echoWatch() { // Print command line for recreating the current display
  for (int i = 0; i < watchListSize; i++)
  { 
    if( watchList[i] ) {
      Serial.print(watchList[i]->name);
      Serial.print(" ;w; ");
    }
  }
  Serial.println();
}

void watch()
{
  if (lastCheck)
  {
    for (int i = 0; i < watchListSize; i++)
    {
      if (!watchList[i])
      {
        watchList[i] = lastCheck;
        Serial.print("Watching ");
        Serial.println(lastCheck->name);
        watching++;
        return;
      }
    }
    Serial.println("Watch list is full");
  } else
  {
    Serial.println("Watch what?");
  }
}

void unwatch()
{
  if (lastCheck)
  {
    for (int i = 0; i < watchListSize; i++)
    {
      if (lastCheck == watchList[i])
      {
        watchList[i] = NULL;
        Serial.print("Unwatching ");
        Serial.println(lastCheck->name);
        watching--;
        return;
      }
    }
    Serial.print("Did not find ");
    Serial.print(lastCheck->name);
    Serial.println(" in watch list.");
  } else
  {
    Serial.println("Unwatch what?");
  }
}

unsigned long nextWatch = 0;

void showWatchRecord(long now)
{
  Serial.print("Millis=");
  Serial.print(now);
  for (int i = 0; i < watchListSize; i++)
  {
    if (watchList[i])
    {
      v = watchList[i];
      Serial.print(", ");
      Serial.print(v->name);
      Serial.print("=");
      displayVariable(*(v->value), v->trailingDigits);
    }
  }
  if (watching > 0)
  {
    Serial.println();
  }
}

void buttonPressed(int buttonNo)
{
  unsigned long now = millis();
  Serial.print("button ");
  Serial.print(buttonNo);
  Serial.print(" ");
  showWatchRecord(now);
}

void pollWatch()
{
  unsigned long now = millis();
  static float lastWatchTime = -1;
  static long interval = 0;
  if (watchTime > 0.001)
  {
    if (lastWatchTime != watchTime)
    {
      lastWatchTime = watchTime;
      interval = watchTime * 1000.0;
      long periods = now / interval;
      if (periods < 1)
      {
        nextWatch = interval;
      } else
      {
        nextWatch = (periods - 1) * interval;
      }

    }
    if (watching > 0)
    {
      if (now > nextWatch)
      {
        showWatchRecord(now);
        nextWatch = nextWatch + interval;
        if (now > nextWatch)
        {
          nextWatch = now + interval;
        }
      }
    }
  }
}

void printHelp()
{
  cmdEntry *p = cmdHead->next;
  while ((cmdEntry *) NULL != p)
  {
    Serial.print(p->name);
    Serial.print("; ");
    Serial.println(p->description);
    p = p->next;
  }
  Serial.println("<varname>; display varname and value");
  Serial.println("<varname> <NN.NNN>; set varname to value NN.NNN");
  Serial.println(" use \";\" or end of line to separate commands");
}

void silentSetVar(char *varName, float val)
{
  entry *symPt = getSymEntry(varName);
  if (symPt != (entry *) NULL)
  {
    float *varPt = symPt->value;
    *varPt = val;
  } else
  {
    float *varPt = new float;
    *varPt = val;
    char *newName = dupString(varName);
    addSym(varPt, newName, "from brainstem", "6N");
  }
}

float tokenVal = 0.0;

void processChar(int c)
{
  static int state = 'B';  // B: Beginning of line
  // N: parsing a number(integer)
  // F: fractional part of number
  // E: end of line
  static char varName[32];
  static int vp = 0;

  static float dp = 0.1;
  static bool negative = false;
  static bool gotVal = false;
  float *varPt;
  if ( c == '#' ) {
    downloadAttemptDetected();  // bootloader sends a #
    state = 'E';
    vp = 0;
  } else if (c == '\n' || c == '\r' || c == ';')
  {
    state = 'E';
  } else if (state == 'B')
  {
    if (c == ' ' || c == '\t')
    {
      if (vp == 0)
      {
        state = 'B'; // ignore leading whitespace.
      } else
      {
        state = 'N';
      }
    } else
    {
      if (vp < 31)
      {
        varName[vp++] = c;
        varName[vp] = 0;
      }
    }
  } else if (state == 'N')
  {
    if (c >= '0' && c <= '9')
    {
      tokenVal = tokenVal * 10 + c - '0';
      gotVal = true;
    } else if (c == '.')
    {
      state = 'F';
      dp = 0.1;
      gotVal = true;
    } else if (c == '-')
    {
      negative = true;
      gotVal = true;
    } else if (!gotVal && (c == ' ' || c == '\t'))
    {
      // Ignore whitespace here.
    } else
    {
      state = 'E';
    }
  } else if (state == 'F')
  {
    if (c >= '0' && c <= '9')
    {
      tokenVal = tokenVal + ((c - '0') * dp);
      dp = dp / 10.0;
    } else
    {
      state = 'E';
    }
  }

  // Did we reach end of line?
  if (state == 'E')
  {
    if (vp > 0)
    {
      stripW(varName);
      if (negative)
      {
        tokenVal = -tokenVal;
      }
      entry *symPt = getSymEntry(varName);

      if (symPt != (entry *) NULL)
      {
        float *varPt = symPt->value;
        Serial.print(" old value for ");
        Serial.print(varName);
        Serial.print(" ");
        displayVariable(*varPt, symPt->trailingDigits);
        if (gotVal)
        {
          Serial.print(" new value: ");
          displayVariable(tokenVal, symPt->trailingDigits);
          *varPt = tokenVal;
        } else
        {
          lastCheck = symPt;
        }

        Serial.println();
      } else
      {
        if (runCmd(varName))
        {

        } else if (gotVal)
        {
          Serial.print("\nNew Variable \"");
          Serial.print(varName);
          Serial.print("\" set to ");
          varPt = new float;
          *varPt = tokenVal;
          char *newName = dupString(varName);
          addSym(varPt, newName, "new variable");
          Serial.println(tokenVal);
        } else
        {
          Serial.print("\nDid not find \"");
          Serial.print(varName);
          Serial.println("\"");
        }
      }
    }
    state = 'B';
    vp = 0;
    tokenVal = 0.0;
    dp = 0.1;
    negative = false;
    gotVal = false;
  }
}

void setupSymtable()
{
  initTable();
  addCmd(&printHelp, "help", "Help, list available commands", NULL);
  addCmd(&listTable, "?", "List variables", NULL);
  addCmd(&dumpLog, "dump", "Dump log", NULL);
  addCmd(&printLog, "log", "Display log as csv", NULL);
  addCmd(&logSyms, "logSyms", "Log symbols now", NULL);
  addSym(&watchTime, "watchTime", "interval between watch reports", "3");
  addCmd(&watch, "watch", "Periodically print a variable", NULL);
  addCmd(&watch, "w", "Periodically print a variable w less typing", NULL);
  addCmd(&unwatch, "unwatch", "remove variable from watch list", NULL);
  addCmd(&clearWatch, "clear", "empty the watch list", NULL);
  addCmd(&echoWatch, "ws", "Display current watch settings", NULL);
  addCmd(&quiet,"quiet", "turn off debug printing", NULL);
}
/*

  void loop() {
  while (Serial.available() ) {
    int c = Serial.read();
    processChar(c);
  }
  delay(100);
  altitude += 1.0;
  throttle *= 1.0001;
  battVoltage *= 0.99;
  logSyms();
  } */
