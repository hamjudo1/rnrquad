#define WHITEBOARD_WIRING 0

#if WHITEBOARD_WIRING == 1
// The whiteboard doesn't have an LED on pin 6, known as "simpleLED". But driving
// pin 6 doesn't hurt anything.
#else
#define USE_DOTSTAR
#endif

#ifdef USE_DOTSTAR
#include <Adafruit_DotStar.h>
#else
#include <Adafruit_NeoPixel.h>
#endif
const int simpleLED = 6;
extern const long minNeoUpdateDelay; // Wait at least 75ms before updating neopixel array.
extern const int NUMPIXELS;
// J5X1  Arduino9 (up)
extern const int J5X1 ;
extern const char *J5X1name ;
// J5X2  Arduino8 (right)
extern const int J5X2;
extern const char *J5X2name;
// J5X3  (Down)
extern const int J5X3 ;
extern const char *J5X3name ;
// J5X4  (left)
extern const int J5X4 ;
extern const char *J5X4name ;
// j5X5 (front)
extern const int J5X5 ;
extern const char *J5X5name ;
extern void setupFlow();
extern void pollFlow();
extern uint32_t showRange(int pixNo, int phaseNo, void *options);
extern void setFixedColor(int pixNo, uint32_t color);
extern void floatRange(int pixNo, float*v, float minv, float maxv, float minColor, float maxColor);
extern uint32_t filler(int pixNo, int phaseNo, void *options);
extern uint32_t shortRed(int pixNo, int phaseNo, void *options);
extern uint32_t longRed(int pixNo, int phaseNo, void *options);
extern uint32_t hsvColor(float h,float s,float v);
extern void setPixRule(uint32_t (*fp)(int, int, void *), int pixNo, int phaseNo,void *options);
extern char* dupString(char* origString);// create a copy of a string in malloc'ed memory.
extern char *catString3(char *origString1, char *origString2, char *origString3);
extern char *catString2(char *origString1, char *origString2);
extern void setupNeoSupp();
extern void pollNeoSupp();
extern void showSmallInt(int iVal);
extern void buttonPressed(int buttonNo);
extern void colorSingleDot(int dotNo,float angle);
extern void rgbSingleDot1(int dotNo, float r, float g, float b);
extern void NeoUpdate();
extern long longestQuietTime  ;
extern int tookTooLongCount ;
extern boolean readyToUpdateNeoPixels  ;
extern unsigned long nextNeoUpdate ;
extern int statusValCount[256];
extern int stuffQueued;
extern int stuffUnqueued;
extern int SPISlaveInterrupts;
extern int SPISlaveCharactersRead;
extern int SPISlaveCharactersWritten;
extern int stem_status_check_count;
extern int packetrx; // count of packets received.
extern int xn297_regs_written;
extern int xn297_regs_read;
extern int voltageByte;
extern int noFlowNoise;
extern float voltage;
extern boolean radioInitialized;
extern float showBrainStemLog;
extern float showXn297LLog;
extern float showPacketLog; 
extern float showRanges;
extern float showPID;
extern float showLogLog;
extern float showVerbose;
extern float estoprange; // E-Stop on range finder within 10 CM.
extern float byPassManipulation;
extern boolean unreadPacket; // We have read a packet from the XN297L, but it hasn't been sent to the brainstem yet.
extern boolean unprocessedPacket;
extern boolean whiteBoard ;
extern boolean greenBoard;
extern float A, B, C, D, E, F; // variables from brainstem
//Decoded packet state.
#define AUXNUMBER 16
#define HISTDEPTH 5
const int RFINDERS = 5;
extern float rangesInM[RFINDERS];
const int P0 = 0, P1 = 1, PHASES = 2;
extern unsigned long rangesTS[RFINDERS][PHASES]; // range finding timestamps.

extern float rangeHist[RFINDERS][HISTDEPTH];
extern unsigned long rangeHistTS[RFINDERS][HISTDEPTH];
extern float rangeVel[RFINDERS]; // velocity in meters per second, negative is approaching.
extern int lastR[RFINDERS];
extern float rangeHistTotal[RFINDERS];
extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];
extern int rxdata[15];
extern  char trims[4];
extern float trim1;
extern float trim1offset;
extern float trim0;
extern unsigned long lastHeartBeat;
const int MAX_ERROR_REPORTS = 12;
extern float errorList[MAX_ERROR_REPORTS];
extern int errorListIndex;
typedef struct rangeConfigElem {
  int8_t j5Index;
  boolean enabled;
  const char *Name;
  int rangeInMM;
} rangeConfigElem_t;
void initTable();
void logSyms();
void addSym(float *v, const char *n, const char* d, char *opt);
void addSym(float *v, const char *n,  const char* d);
void silentSetVar(char *varName, float val);
boolean runCmd(char *n);
// Given a symbol name, return the pointer to variable.
float *getSymPtr(char *name);
void processChar(int c);
void setupSymtable();
void writePacket(int data[], uint8_t size);
extern int getAveFlow(float &xFlow, float &yFlow, float &qFlow);
extern void setupThinking();
extern void pollThinking();
extern void keyboardThrottle();
extern float tState;
const int DOWNRANGE = 1;
const int FRONTRANGE = 0;
const int RIGHTRANGE = 2;
const int LEFTRANGE = 3;
extern long debugTime, debugStart;
extern long commTime, commStart;
extern long neoTime, neoStart;
extern long rangeTime, rangeStart;
extern long thinkTime, thinkStart;
// The Status byte from the Cortex has these bits:
#define STATE_CORTEX_ALIVE      0x01
#define STATE_BOUND             0x02
#define STATE_GOT_PACKET        0x04
#define STATE_WANT_IMU_DATA     0x08
#define STATE_WANT_BATTERY_DATA 0x10
#define STATE_MUST_BE_ZERO      0x80 // If this isn't zero, the SPI bus is stuck high.
     // If the bus is briefly stuck high, that means the cortex CPU is rebooting or
     // a test probe is touching multiple pins.
     // If this is permanent you have proof that the hardware was hand assembled
     // with care, but not precision.
     // 
extern int cortexState;
// When connecting test equipment to the SPI bus,
// it is easy to accidentally pull the bus high. When that
// happens, every bit is stuck at 1. That might be temporary
// or permanent, but we shouldn't do anything dangerous.
extern boolean stateIndicateHardwareFault; 

enum cortex_debug {
  packet_in_fifo=0,
  we_are_bound,
  radio_confused,
  range_display,
  cortex_loop,
};
//typedef enum cortex_debug corex_debug_t;
extern void cortexDebugRange(enum cortex_debug  debug_event, int rangeFinderIndex, int rangeInMM);
extern void cortexDebug(enum cortex_debug debug_event);
extern void pollDebug();
extern void pollWatch();
extern void setupDebug();
extern void quietTime();
extern void dispXn297LLog();
extern void dispPacketLog();
extern void leftTrimChanged(int trimVal);
extern void pollRangeFinders();
extern void setupRangeFinders();
extern void pollBlink();
extern void blinkIt(int pinNo);
extern void replaceRx(float newrx[4], uint8_t oldPacket[15], uint8_t newPacket[15] ); 
extern void updateChecksum(uint8_t packet[15]);
extern void xn_writepayload( int data[] , int size );
extern void addCmd(void (*cmd_ptr)(void), const char *n, const char* d, char *opt);
extern int xn_readreg( int reg);
extern float mapf(float val, float in_min, float in_max, float out_min, float out_max);
