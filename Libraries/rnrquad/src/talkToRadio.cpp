
#include "state.h"
#include "objects/src/ControllerState.h"

extern void xn_writereg(int reg, int val);

rx_mode_t rxmode = RX_MODE_BIND;
int rxdata[15];
int telemetry_send = 0;
int packetrx = 0;

#define XN_TO_RX  0x8F // B10001111
#define XN_TO_TX 0x8E //  B10000010

// packet period in uS
#define PACKET_PERIOD 3000
#define PACKET_PERIOD_TELEMETRY 5000


int failsafe = 0;


unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;
int telemetry_enabled = 0;
int packet_period = PACKET_PERIOD;
char lasttrim[4];
char rfchannel[4];
int rxaddress[5];
int rf_chan = 0;

// defines for things that do not normally need changing


#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_FR 3
#define MOTOR_BR 2

#define PIDNUMBER 3

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f


#define ROLL 0
#define PITCH 1
#define YAW 2

// this should be precalculated by the compiler as it's a constant
#define FILTERCALC(sampleperiod, filtertime) (1.0f - ( 6.0f*(float)sampleperiod) / ( 3.0f *(float)sampleperiod + (float)filtertime))


#define RXMODE_BIND 0
#define RXMODE_NORMAL (!RXMODE_BIND)

// defines for bayang protocol radio
#define CH_ON (AUXNUMBER - 2) // 14
#define CH_OFF (AUXNUMBER - 1) // 15
#define CH_FLIP 0
#define CH_EXPERT 1
#define CH_HEADFREE 2
#define CH_RTH 3
#define CH_AUX1 4
#define CH_AUX2 5
#define CH_EMG 10
#define CH_TO 11
// trims numbers have to be sequential, start at CH_PIT_TRIM
#define CH_PIT_TRIM 6
#define CH_RLL_TRIM 7
#define CH_THR_TRIM 8
#define CH_YAW_TRIM 9
// next 3 channels only when *not* using USE_STOCK_TX
#define CH_INV 6
#define CH_VID 7
#define CH_PIC 8
#define CH_9 9
#define CH_12 12
#define CH_13 13

// defines for cg023 protocol
#define CH_CG023_LED 3
#define CH_CG023_FLIP 0
#define CH_CG023_STILL 2
#define CH_CG023_VIDEO 1

#define CH_H7_FLIP 0
#define CH_H7_VIDEO 1
#define CH_H7_FS 2

#define CH_CX10_CH0 0
#define CH_CX10_CH2 2

#define CH_AUX3 CH_OFF
#define CH_AUX4 CH_OFF

// devo tx channel mapping
// also for nr24multipro
#define DEVO_CHAN_5 CH_INV
#define DEVO_CHAN_6 CH_FLIP
#define DEVO_CHAN_7 CH_PIC
#define DEVO_CHAN_8 CH_VID
#define DEVO_CHAN_9 CH_HEADFREE
#define DEVO_CHAN_10 CH_RTH

// multimodule mapping ( taranis )
#define MULTI_CHAN_5 CH_FLIP
#define MULTI_CHAN_6 CH_RTH
#define MULTI_CHAN_7 CH_PIC
#define MULTI_CHAN_8 CH_VID
#define MULTI_CHAN_9 CH_HEADFREE
#define MULTI_CHAN_10 CH_INV

// used for the pwm driver
#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3


#define int32 int_fast32_t
#define int16 int_fast16_t
#define int8 int_fast8_t

#define uint32 uint_fast32_t
#define uint16 uint_fast16_t
#define uint8 uint_fast8_t

// for h-bridge states
#define FREE 2
#define BRAKE 3
#define DIR1 1
#define DIR2 0
// for inverted flight motor direction
#define FORWARD DIR2
#define REVERSE DIR1


float rx[4];
char aux[AUXNUMBER];
char lastaux[AUXNUMBER];
char auxchange[AUXNUMBER];

unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

void nextchannel()
{
  rf_chan++;
  rf_chan &= 3; // same as %4
  xn_writereg(0x25, rfchannel[rf_chan]);
}

float packettodata(int *data)
{
  return (((data[0] & 0x0003) * 256 + data[1]) - 512) * 0.001953125;
}

void datatopacket(float val, uint8_t *packet)
{
  int newVal;
  if (val < 0.0)
  {
    newVal = int(0.5 + (val / 0.001953125)); // Convert from -1.0 to 1.0 to -512 to 511;
  } else
  {
    newVal = int(0.5 + (val / 0.001953125)); // Convert from -1.0 to 1.0 to -512 to 511;
  }
  newVal = newVal + 512;  // Should be a number 0 to 1023
  newVal = constrain(newVal, 0, 1023); // Really is a number 0 to 1023.
  packet[1] = newVal & 0xff;
  packet[0] = newVal / 256 | (packet[0] & 0xfc);
}

void throttletopacket(float val, uint8_t *packet)
{
  int newVal = int(0.5 + (val / 0.000976562f)); // Convert from 0.0 to 1.0 to 0 to 511;
  int byte0 = 0, byte1 = 0;

  newVal = constrain(newVal, 0, 1023); // Really is a number 0 to 1023.
  packet[1] = newVal & 0xff;
  packet[0] = newVal / 256 | (packet[0] & 0xfc);
}

void updateChecksum(uint8_t packet[15])
{
  int sum = 0;
  for (int i = 0; i < 14; i++)
  {
    sum += packet[i];
  }
  packet[14] = sum & 0xff;
}

bool verifyChecksum(uint8_t packet[15])
{
  int sum = 0;
  for (int i = 0; i < 14; i++)
  {
    sum += packet[i];
  }
  return packet[14] == (sum & 0xff);
}

// newPacket will have stick values from newrx[] and everything else from oldPacket.
void replaceRx(float newrx[4], uint8_t oldPacket[15], uint8_t newPacket[15])
{
  for (int i = 0; i < 14; i++)
  {
    newPacket[i] = oldPacket[i]; // Last byte is checksum, which we will calculate at the end.
  }

  datatopacket(newrx[0], &newPacket[4]);
  datatopacket(newrx[1], &newPacket[6]);
  datatopacket(newrx[2], &newPacket[10]);
  throttletopacket(newrx[3], &newPacket[8]);
}

bool auxChanged = false;
char trims[4];
void setupController(void)
{
  addSym(&refControl.throttle,"rt","3n");
  addSym(&refControl.leftStickXPosition,"rx2","3n");
  addSym(&refControl.rightStickXPosition,"rx0","3n");
  addSym(&refControl.rightStickYPosition,"rx1","3n");
}
static int decodepacket(void)
{
  if (rxdata[0] == 165)
  {
    int sum = 0;
    for (int i = 0; i < 14; i++)
    {
      sum += rxdata[i];
    }
    if ((sum & 0xFF) == rxdata[14])
    {
      rx[0] = packettodata(&rxdata[4]);
      rx[1] = packettodata(&rxdata[6]);
      rx[2] = packettodata(&rxdata[10]);
      // throttle
      rx[3] =
          ((rxdata[8] & 0x0003) * 256 +
           rxdata[9]) * 0.000976562f;
      refControl.throttle = rx[3];
      refControl.leftStickXPosition = rx[2];
      refControl.rightStickXPosition = rx[0];
      refControl.rightStickYPosition = rx[1];

      trims[0] = rxdata[6] >> 2;
      refControl.trim0 = (float) trims[0] - 31.0;
      trims[1] = rxdata[4] >> 2;
       
      refControl.trim1 = 31.0 - (float) trims[1]; // Yes the sign is different
      trims[2] = rxdata[8] >> 2;
      trims[3] = rxdata[10] >> 2;

      for (int i = 0; i < 2; i++)
        if (trims[i] != lasttrim[i])
        {
          aux[CH_PIT_TRIM + i] = trims[i] > lasttrim[i]; // 6, 7, 8
          lasttrim[i] = trims[i];
        }
      aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0;   // inverted flag 6

      aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;   // 7

      aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;  // 8
      aux[CH_9] = (rxdata[2] & 0x40) ? 1 : 0;          // 9

      aux[CH_TO] = (rxdata[3] & 0x20) ? 1 : 0;   // take off flag  11

      aux[CH_EMG] = (rxdata[3] & 0x04) ? 1 : 0;   // emg stop flag 10

      aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0; // 0

      aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0; // 1
      aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0; // 2
      aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0;   // rth channel 3
      aux[CH_12] = (rxdata[2] & 0x40) ? 1 : 0;
      aux[CH_13] = (rxdata[1] & 0x02) ? 1 : 0;
      auxChanged = false;
      for (int i = 0; i < AUXNUMBER - 2; i++)
      {
        auxchange[i] = 0;
        if (lastaux[i] != aux[i])
        {
          auxchange[i] = 1;
          auxChanged = true;
          if (i == 8)
          {
            buttonPressed(8);
          }
          if (i == 7)
          {
            buttonPressed(7);
          }
        }
        lastaux[i] = aux[i];
      }
      if (showPacketLog && auxChanged)
      {
        dispPacketLog();
      }

      return 1;       // valid packet
    }
    return 0;             // sum fail
  }
  return 0;                   // first byte different
}


void processPacket(uint8_t inbData[15])
{
  for (int i = 0; i < 15; i++)
  {
    rxdata[i] = inbData[i];
  }
  if (rxdata[0] == 123)
  {
    Serial.print("123 packet received ");
    Serial.println(rxdata[1]);

    return;
  }
  if (rxmode == RX_MODE_BIND)
  {
    // rx startup , bind mode
    if (rxdata[0] == 0xa4) // 0xa4 is normal, 0xa3 is with telemetry
    {
      // bind packet
      rfchannel[0] = rxdata[6];
      rfchannel[1] = rxdata[7];
      rfchannel[2] = rxdata[8];
      rfchannel[3] = rxdata[9];

      uint8_t rxaddr[6] = {0x2a,};

      for (int i = 1; i < 6; i++)
      {
        rxaddr[i] = rxdata[i];
      }
      // write new rx address
      writeregs(rxaddr, sizeof(rxaddr));
      rxaddr[0] = 0x30; // tx register ( write ) number

      // write new tx address
      writeregs(rxaddr, sizeof(rxaddr));

      xn_writereg(0x25, rfchannel[rf_chan]);    // Set channel frequency
      rxmode = RX_MODE_NORMAL;
      // cortexDebug(we_are_bound);
      cortexState |= STATE_BOUND;
    } else
    {
      // cortexDebug(radio_confused);
    }
  } else
  {
    unsigned long temptime = micros();


    bool pass = decodepacket();

    if (pass)
    {
      cortexState |= STATE_GOT_PACKET;
      unreadPacket = true;
      packetrx++;
      // if (telemetry_enabled)
      //   beacon_sequence();
      skipchannel = 0;
      timingfail = 0;
      lastrxchan = rf_chan;
      lastrxtime = temptime;
      failsafetime = temptime;
      failsafe = 0;
      if (!telemetry_send)
        nextchannel();
    } else
    {
      // RX failure warning goes here.
    }
  }
}


void writeregs(uint8_t data[], uint8_t size)
{
  spi_cson();
  for (uint8_t i = 0; i < size; i++)
  {
    spi_sendbyte(data[i]);
  }
  spi_csoff();
}


// Logic analyzer says
// | 39 01 | 2A 00 00 00 00 00 | 21 00 | 22 01 | 26 39 | 31 0F | 24 00 | 23 03 | E2 | 25 00 | 3D 38 | FD 00 | 20 8F | 0F C6 | 07 4E | 07 4E

int radioDefault(void)
{
  // Gauss filter amplitude - lowest
  static uint8_t demodcal[2] = {0x39, B00000001};
  writeregs(demodcal, sizeof(demodcal));


  static uint8_t rxaddr[6] = {0x2a, 0, 0, 0, 0, 0};
  writeregs(rxaddr, sizeof(rxaddr));

  xn_writereg(EN_AA, 0);      // aa disabled | 21 00 |
  xn_writereg(EN_RXADDR, 1);  // pipe 0 only  | 22 01 |
  xn_writereg(RF_SETUP, 0x3F);    // power:max / data rate: 1Mbps
  // ored with "1", which must be the other two parameters.
  xn_writereg(RX_PW_P0, 15);  // payload size | 31 0f|
  xn_writereg(SETUP_RETR, 0); // no retransmissions ( redundant?) | 24 00 |
  xn_writereg(SETUP_AW, 3);   // address size (5 bytes) | 23 03 |
  xn_command(FLUSH_RX);      // | E2 |
  xn_writereg(RF_CH, 0);      // bind on channel 0  | 25 00 |

  xn_writereg(0x1d, B00111000);   // 64 bit payload , software ce  | 3D 38 |
  spi_cson();                       //    | FD 00 |
  spi_sendbyte(0xFD);         // internal CE high command
  spi_sendbyte(0);            // required for above
  spi_csoff();
  xn_writereg(0, XN_TO_RX);   // power up, crc enabled, rx mode | 20 8F |

  int rxcheck = xn_readreg(0x0f);   // | 0f ?? |
  Serial.print("rxcheck ");
  Serial.println(rxcheck);
  return rxcheck;
}
