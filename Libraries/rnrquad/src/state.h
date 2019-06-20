#ifndef RNRQUAD_STATE_H
#define RNRQUAD_STATE_H

#include "Arduino.h"
#include "classes/src/ControllerState.h"
#include "classes/src/Lidar.h"
#include "classes/src/RgbLed.h"
#include "classes/src/SensorState.h"

typedef unsigned char uint8_t;

#define WHITEBOARD_WIRING 0

#define XN_CONFIG      0x00  // This was just "CONFIG" in H8mini_blue_board/Silverware
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define XN_STATUS      0x07   // This was just "STATUS" in H8mini_blue_board/Silverware
// That caused a name space conflict, so we changed it to XN_STATUS
#define OBSERVE_TX  0x08
#define CD          0x09 // RPD in one source
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

// bit masks
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
// Bits in RF_SETUP
/*************************data rate****************************************/
#define         DR_1M                          0X00        //通信速率 1Mbps
#define         DR_2M                          0X40       //通信速率 2Mbps
#define         DR_250K                        0XC0       //通信速率 250Kbps
/*************************power********************************************/
#define         RF13dBm                         0x3f//44//0x3F                            // 13dBm
#define         RF10dBm                         0X0F                            // 10dBm
#define         RF8dBm                          0x15                            // 8dbm
#define         RF7dBm                          0x07                            // 7dbm
#define         RF5dBm                          0x2c                            // 5dbm
#define         RF4dBm                          0x06                            // 4dbm
#define         RF2dBm                          0x05                            // 2dbm
#define         RF0dBm                          0X0B                            // 0dBm
#define         RF_3dBm                         0x04                            // -3dBm
#define         RF_6dBm                         0x0A                            // -6dBm
#define         RF_10dBm                        0x02                            // -10dBm
#define         RF_18dBm                        0x01                            // -18dBm
#define         RF_30dBm                        0x00                            // -30dBm

// Commands
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define W_CX_PAYLOAD  0xE7
#define HEARTBEAT     0xE9 // Brainstem is alive.

#define NOP           0

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

extern void pollComm();
extern void setupComm();
extern void setupSerial();
extern int radioDefault(void);
extern void xn297L_debug(void);
extern int XN297L_regs[32];  // brainstem only looks at 0x07, 0x0F, 0x17
extern int XN297L_goodPayloadIn; // 2 payload buffers, read one from xn297L while sending the other to the brainstem
extern int XN297L_goodPayloadOut;
extern uint8_t XN297L_payloadIn[2][15];// 15 bytes from the remote.
extern uint8_t XN297L_payloadOut[2][15];// 15 bytes from the remote.

// From H8mini_blue_board/Silverware/src/xn297.h

extern void xn_writereg(int reg, int val);
extern void xn_writereg(int reg, uint8_t data[], uint8_t size);
extern int xn_readreg(int reg);
extern void xn_readpayload( uint8_t *data , int size );
extern int xn_command(int command);
extern int spi_recvbyte( void);
extern void spi_sendbyte( int b);
extern void spi_cson(void); // start of packet stuff.
extern void spi_csoff(void); // end of packet
extern void writeregs(uint8_t data[], uint8_t size);
extern void processPacket(uint8_t rxdata[15]);
extern void checkPacket(void);

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

typedef enum {
    RX_MODE_BIND,
    RX_MODE_NORMAL
} rx_mode_t;
extern rx_mode_t rxmode;

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

extern int spi_recvbyte( void);
extern void spi_sendbyte( int b);
extern void spi_cson(void); // start of packet stuff.
extern void spi_csoff(void); // end of packet
extern void spiSlave_init(); // initialize SPI slave (connection to brain stem)

#endif