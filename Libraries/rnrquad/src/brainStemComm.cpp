// started with examples from https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-spi
// but changed much.
// master to radio (xn297L) slave
// Sercom2
// DOPO 0x2 SPI_PAD_3_SCK1
// DIPO 0x0 SERCOM_RX_PAD_0

// SS    XN297L pin1 pad2 pin23 pa14 sercom 2.2
// CLK XN297L pin2  pad1  pin22 pa13 sercom 2.1
// MISO XN297L (pin3 direct) pad0 pin21 pa12 Sercom 2.0
// MOSI XN297L (Pin3 resistor) pad3  pin24 pa15 Sercom 2.3
/*
  Quad MISO CPU pin 28 Port PA19 Arduino Pin 12.
  Quad MOSI CPU pin 25 Port PA16 Arduino Pin 11.
  Quad SS CPU pin 27 Port PA18 Arduino Pin 10.
  Quad CLK CPU pin 26 Port PA17 Arduino Pin 13.

  Radio MOSI CPU pin 19 Port PB10 Arduino Pin 23.
  Radio MISO CPU pin 21 Port PA12 Arduino Pin 22.
  Radio CLK CPU pin 20 Port PB11 Arduino Pin 24.
  Radio SS CPU pin 24 Port PA15 Arduino Pin 5.
*/
// #include <SERCOM.h>
#include <SPI.h>

#include "state.h"

const int DIO8 = 8; // arduino8 DIO8 on logic analyzer
const int mySPI_DOPO = 0x2;
const int mySPI_DIPO = 0x0;
const int mySPI_MISO = 22;
const int mySPI_MOSI = 23;
const int mySPI_CLK = 24;
const int mySPI_SS = 5;


char *regList[] = {"CONFIG", "EN_AA", "EN_RXADDR", "SETUP_AW", "SETUP_RETR", "RF_CH", "RF_SETUP", "STATUS",
                   "OBSERVE_TX", "CD", "RX_ADDR_P0", "RX_ADDR_P1", "RX_ADDR_P2", "RX_ADDR_P3", "RX_ADDR_P4",
                   "RX_ADDR_P5",
                   "TX_ADDR", "RX_PW_P0", "RX_PW_P1", "RX_PW_P2", "RX_PW_P3", "RX_PW_P4", "RX_PW_P5", "FIFO_STATUS"
};

#include "wiring_private.h" // pinPeripheral() function
// #define PIN_SPI_MISO         (22u)
// #define PIN_SPI_MOSI         (23u)
// #define PIN_SPI_SCK          (24u)
// #define PERIPH_SPI           sercom4
// #define PAD_SPI_TX           SPI_PAD_2_SCK_3
// #define PAD_SPI_RX           SERCOM_RX_PAD_0
// SPIClass mySPI(&sercom2, mySPI_MISO, mySPI_CLK, mySPI_MOSI, SPI_PAD_3_SCK_1, SERCOM_RX_PAD_0);
// #define PIN_SPI_MISO         (22u)
// #define PIN_SPI_MOSI         (5u)
// #define PIN_SPI_SCK          (38u)
// #define PERIPH_SPI           sercom2
// #define PAD_SPI_TX           SPI_PAD_3_SCK_1
// #define PAD_SPI_RX           SERCOM_RX_PAD_0
#include <SPI.h>

const int PIN_SPI_SS = mySPI_SS;
boolean talk = true;
boolean pTalk = true;
int talkMax = 100;
int talki = 0;
int packetPrint = 500;
int outByte = 255;
int lastCmd = -1;
boolean waitForRead = false;
boolean waitForWrite = false;
boolean waitForCmd = false;
const int slaveLogSize = 1024;

uint8_t dVals[slaveLogSize];
uint8_t pNos[slaveLogSize];
int ivp = 0;
int packetStart = 0;
boolean showIt = false;
boolean skipIt = false;

int lastSaved = -1;
int lastShown = -1;
int cheatCnt = 0;
int byteToSend = 0;
int packetPos = 0;
boolean singlePass = false;

int outQueue[256];
int queueSize = 0;
int queuePnt = 0;

int queueBytes(uint8_t data[], int count)
{
  for (int c = 0; c < count; c++)
  {
    outQueue[c] = data[c];
  }
  queuePnt = 0;
  queueSize = count;
  stuffQueued = stuffQueued + count;
}

int getQueuedByte()
{
  if (queuePnt < queueSize)
  {
    stuffUnqueued++;
    return outQueue[queuePnt++];
  } else
  {
    return 0;
  }
}

int stem_status_check_count = 0;
float colorAngle = 0.0;
int countList[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
int inbound[16];
int SPI_STATE = 'N'; // N for Normal, W for Write, R for Read
void processSPIByte(int D)
{
  static int lastCortexState = 0;
  // Just keep a circular buffer and overwrite old stuff.
  if (singlePass && ivp > slaveLogSize)
  {
    return;
  }
  if (D < 0)
  {
    pNos[ivp % slaveLogSize] = 128; // Log an error.
    dVals[ivp % slaveLogSize] = -D;
    ivp++;
    return;
  } else
  {
    dVals[ivp % slaveLogSize] = D;
    pNos[ivp % slaveLogSize] = packetPos;
  }
  if (packetPos == 0)
  {
    SPI_STATE = 'N';
    switch (D)
    {
      case 0x07: // send our status register
        byteToSend = cortexState;
        /*
          if ( cortexState != lastCortexState ) {
          lastCortexState = cortexState;
          Serial.print("C: ");
          Serial.println(cortexState);
          }
        */
        stem_status_check_count++;
        break;
        // Read other register
      case 0x00:
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x04:
      case 0x05:
      case 0x06:
      case 0x08:
      case 0x09:
      case 0x0A:
      case 0x0B:
      case 0x0C:
      case 0x0D:
      case 0x0E:
      case 0x0F:
      case 0x10:
      case 0x11:
      case 0x12:
      case 0x13:
      case 0x14:
      case 0x15:
      case 0x16:
      case 0x17:
      case 0x18:
      case 0x19:
      case 0x1A:
      case 0x1B:
      case 0x1C:
      case 0x1D:
      case 0x1E:
      case 0x1F:
        byteToSend = XN297L_regs[D];
        break;  // write register
      case 0x20:
      case 0x21:
      case 0x22:
      case 0x23:
      case 0x24:
      case 0x25:
      case 0x26:
      case 0x27:
      case 0x28:
      case 0x29:
      case 0x2A:
      case 0x2B:
      case 0x2C:
      case 0x2D:
      case 0x2E:
      case 0x2F:
      case 0x30:
      case 0x31:
      case 0x32:
      case 0x33:
      case 0x34:
      case 0x35:
      case 0x36:
      case 0x37:
      case 0x38:
      case 0x39:
      case 0x3A:
      case 0x3B:
      case 0x3C:
      case 0x3D:
      case 0x3E:
      case 0x3F:
        SPI_STATE = 'W';
        break;
      case R_RX_PAYLOAD: // 0x61 Read 15 byte Payload
        cortexState = ~STATE_GOT_PACKET & cortexState;
        queueBytes(XN297L_payloadOut[XN297L_goodPayloadOut], 15);
        //  queueBytes( XN297L_payloadIn[XN297L_goodPayloadIn], 15);

        //  queueBytes(rxdata,15);
        // queueBytes(countList, 15);

        break;
      case W_TX_PAYLOAD: // 0xA0 write 15 byte payload
        break;
      case FLUSH_TX: // 0xE1
      case FLUSH_RX: // 0xE2
      case REUSE_TX_PL: // 0xE3
        break;
      case W_CX_PAYLOAD:
        SPI_STATE = 'R';
        break;
      case HEARTBEAT: // cortex is alive.
        lastHeartBeat = micros();
        SPI_STATE = 'H';
        break;
      case 0xFF:
        break;
    }
  } else if (SPI_STATE == 'R')
  {
    inbound[packetPos - 1] = D;
  } else if (SPI_STATE == 'H')
  {
    inbound[packetPos - 1] = D;
  }
  packetPos++;
  ivp++;
}

void packetComplete()
{
  if (SPI_STATE == 'H')
  {
    if (packetPos > 0)
    {
      int prevVB = voltageByte;
      voltageByte = inbound[0];
      if (voltageByte != prevVB)
      {
        // Serial.print("Voltage: ");
        voltage = (float) voltageByte / 50.0;
        // Serial.println((float)voltageByte / 50.0, 2);
      }
    } else
    {
      //  voltageByte = 257; // Impossible data value.
      static int borkCnt = 0;
      borkCnt++;
      if (borkCnt % 100 == 1)
      {
        Serial.print(borkCnt);
        Serial.println(" Borked");
      }
    }
    SPI_STATE = 'N';
  } else if (SPI_STATE == 'R')
  {
    if (inbound[0] == 135)
    {
      char cmdname[16];
      int ip = 1, vp = 0;
      while (inbound[ip] && ip < packetPos - 1)
      {
        cmdname[vp++] = inbound[ip++];
      }
      cmdname[vp] = inbound[ip++];
      runCmd(cmdname);
    } else if (inbound[0] == 134)
    {
      char varname[16];
      int ip = 1, vp = 0;
      while (inbound[ip] && ip < packetPos - 1)
      {
        varname[vp++] = inbound[ip++];
      }
      varname[vp] = inbound[ip++];
      float value = 0.0;
      unsigned char *fp = (unsigned char *) &value;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
      silentSetVar(varname, value);
    } else if (inbound[0] == 136)
    {
      int ip = 1;
      A = 0.0;
      B = 0.0;
      C = 0.0;
      unsigned char *fp = (unsigned char *) &A;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
      fp = (unsigned char *) &B;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
      fp = (unsigned char *) &C;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
    } else if (inbound[0] == 137)
    {
      int ip = 1;
      D = 0.0;
      E = 0.0;
      F = 0.0;
      unsigned char *fp = (unsigned char *) &D;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
      fp = (unsigned char *) &E;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
      fp = (unsigned char *) &F;
      for (int i = 0; i < 4 && ip < (packetPos - 1); i++)
      {
        *fp++ = inbound[ip++];
      }
    } else
    {
      Serial.print(SPI_STATE);
      Serial.print(" brainstem says: ");
      for (int i = 0; i < packetPos - 1; i++)
      {
        Serial.print(inbound[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  packetPos = 0;
}

int lastShownIvp = 0;

void dispIntVal()
{
  int iPoint = ivp - slaveLogSize;

  if (iPoint < lastShownIvp)
  {
    iPoint = lastShownIvp;
  }
  int bCnt = 0;
  if (radioInitialized)
  {
    Serial.print("Radio Status ");
  } else
  {
    Serial.print("Radio NOT initialized ");
  }
  Serial.print(XN297L_regs[0x0f], 16);

  Serial.print(", ");
  Serial.print(XN297L_regs[0x17], 16);
  Serial.print(", ");
  Serial.println(XN297L_regs[0x07], 16);
  Serial.print("Snapshot, last ");
  Serial.print(slaveLogSize);
  Serial.println(" bytes ");
  int i;
  for (i = iPoint; i < ivp; i++)
  {
    if (pNos[i % slaveLogSize] == 0)
    {
      Serial.print(" | ");
    } else if (pNos[i % slaveLogSize] == 128)
    {
      Serial.print(" *:"); // Error
    } else
    {
      Serial.print(" ");
    }
    if (dVals[i % slaveLogSize] < 16)
    {
      Serial.print("0"); // Always print 2 digit hex
    }
    Serial.print(dVals[i % slaveLogSize], 16);
    bCnt++;
    if (bCnt % 64 == 0)
    {
      Serial.println();
    }
  }
  lastShownIvp = i;
}


void SERCOM1_Handler()
{
  // Serial.println("In SPI Interrupt");
  SPISlaveInterrupts++;
  boolean dataWritten = false;
  boolean byteProcessed = false; // Should be true by the end of the function.
  uint8_t data = 0;

  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; //Read SPI interrupt register

  if (interrupts & (1 << 7))
  {
    SERCOM1->SPI.INTFLAG.bit.ERROR = 1;
  }
  // Serial.println(interrupts);
  // Bit 3 – SSL: Slave Select Low
  // This flag is cleared by writing a one to it.
  // This bit is set when a high to low transition is detected on the _SS pin in slave mode and Slave Select Low
  // Detect (CTRLB.SSDE) is enabled.
  // Writing a zero to this bit has no effect.
  // Writing a one to this bit will clear the flag.
  if (interrupts & (1 << 3))  // End of SPI packet.
  {
    if (!byteProcessed)
    {
      byteProcessed = true;
    }

    SERCOM1->SPI.INTFLAG.bit.SSL = 1; //clear slave select interrupt
    packetComplete();

  }
  //  Bit 2 – RXC: Receive Complete
  // This flag is cleared by reading the Data (DATA) register or by disabling the receiver.
  // This flag is set when there are unread data in the receive buffer. If address matching is enabled, the first data
  // received in a transaction will be an address.
  // Writing a zero to this bit has no effect.
  // Writing a one to this bit has no effect.
  if (interrupts & (1 << 2))
  {
    // Serial.println("SPI Data Received Complete Interrupt");
    data = SERCOM1->SPI.DATA.reg; //Read data register


    // SERCOM1->SPI.INTFLAG.bit.RXC = 1; //clear receive complete interrupt // Cleared by reading data.
    if (!byteProcessed)
    {
      processSPIByte(data);
      byteProcessed = true;
    } else
    {
      processSPIByte(-3);
    }

  }
  //  Bit 1 – TXC: Transmit Complete
  // This flag is cleared by writing a one to it or by writing new data to DATA.
  // In master mode, this flag is set when the data have been shifted out and there are no new data in DATA.
  // In slave mode, this flag is set when the _SS pin is pulled high. If address matching is enabled, this flag is only set
  // if the transaction was initiated with an address match.
  // Writing a zero to this bit has no effect.
  // Writing a one to this bit will clear the flag.
  if (interrupts & (1 << 1))
  {
    //    Serial.println("SPI Data Transmit Complete Interrupt");
    // SERCOM1->SPI.INTFLAG.bit.TXC = 1; //clear transmit complete interrupt
    SERCOM1->SPI.DATA.reg = byteToSend;
    byteToSend = getQueuedByte();
    dataWritten = true;
    byteProcessed = true;

  }
  // Bit 0 – DRE: Data Register Empty
  // This flag is cleared by writing new data to DATA.
  // This flag is set when DATA is empty and ready for new data to transmit.
  // Writing a zero to this bit has no effect.
  if (interrupts & (1 << 0))
  {
    // Serial.println("SPI Data Register Empty Interrupt");

    if (!dataWritten)
    {
      SERCOM1->SPI.DATA.reg = byteToSend;
      byteToSend = getQueuedByte();
    } else
    {
      SERCOM1->SPI.INTFLAG.bit.DRE = 1;
    }

  }
  if (!byteProcessed)
  {
    processSPIByte(-2);
    byteProcessed = true;
  }
}

void spiSlave_init()
{
  //Configure SERCOM1 SPI PINS
  // PORTA.DIR.reg &= ~PORT_PA16; //Set PA16 as input (MOSI) (Arduino 11)
  // PORTA.DIR.reg &= ~PORT_PA17; //Set PA17 as input (SCK)  (Arduino 13)
  // PORTA.DIR.reg &= ~PORT_PA18; //Set PA18 as input (SS)   (Arduino 10)
  // PORTA.DIR.reg |= PORT_PA19; //Set PA19 as output (MISO) (Arduino 12)
  // Quad MISO CPU pin 28 Port PA19 Arduino Pin 12.
  // Quad MOSI CPU pin 25 Port PA16 Arduino Pin 11.
  // Quad SS CPU pin 27 Port PA18 Arduino Pin 10.
  // Quad CLK CPU pin 26 Port PA17 Arduino Pin 13.

  PORT->Group[PORTA].PINCFG[16].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
  PORT->Group[PORTA].PMUX[8].bit.PMUXE = 0x2; //SERCOM 1 is selected for peripherial use of this pad
  PORT->Group[PORTA].PINCFG[17].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
  PORT->Group[PORTA].PMUX[8].bit.PMUXO = 0x2; //SERCOM 1 is selected for peripherial use of this pad
  PORT->Group[PORTA].PINCFG[18].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
  PORT->Group[PORTA].PMUX[9].bit.PMUXE = 0x2; //SERCOM 1 is selected for peripherial use of this pad
  PORT->Group[PORTA].PINCFG[19].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
  PORT->Group[PORTA].PMUX[9].bit.PMUXO = 0x2; //SERCOM 1 is selected for peripherial use of this pad

  //Disable SPI 1
  SERCOM1->SPI.CTRLA.bit.ENABLE = 0;
  while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE);

  //Reset SPI 1
  SERCOM1->SPI.CTRLA.bit.SWRST = 1;
  while (SERCOM1->SPI.CTRLA.bit.SWRST || SERCOM1->SPI.SYNCBUSY.bit.SWRST);

  //Setting up NVIC
  NVIC_EnableIRQ(SERCOM1_IRQn);
  NVIC_SetPriority(SERCOM1_IRQn, 2);

  //Setting Generic Clock Controller!!!!
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM1_CORE) | //Generic Clock 0
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is the source
                      GCLK_CLKCTRL_CLKEN; // Enable Generic Clock Generator

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); //Wait for synchronisation


  //Set up SPI Control A Register
  SERCOM1->SPI.CTRLA.bit.DORD = 0; //MSB first
  SERCOM1->SPI.CTRLA.bit.CPOL = 0; //SCK is low when idle, leading edge is rising edge
  SERCOM1->SPI.CTRLA.bit.CPHA = 0; //data sampled on leading sck edge and changed on a trailing sck edge
  SERCOM1->SPI.CTRLA.bit.FORM = 0x0; //Frame format = SPI
  SERCOM1->SPI.CTRLA.bit.DIPO = 0; //DATA PAD0 MOSI is used as input (slave mode)
  SERCOM1->SPI.CTRLA.bit.DOPO = 0x2; //DATA PAD3 MISO is used as output
  SERCOM1->SPI.CTRLA.bit.MODE = 0x2; //SPI in Slave mode
  SERCOM1->SPI.CTRLA.bit.IBON = 0x1; //Buffer Overflow notification
  SERCOM1->SPI.CTRLA.bit.RUNSTDBY = 1; //wake on receiver complete

  //Set up SPI control B register
  //SERCOM1->SPI.CTRLB.bit.RXEN = 0x1; //Enable Receiver
  SERCOM1->SPI.CTRLB.bit.SSDE = 0x1; //Slave Selecte Detection Enabled
  SERCOM1->SPI.CTRLB.bit.CHSIZE = 0; //character size 8 Bit
  //SERCOM1->SPI.CTRLB.bit.PLOADEN = 0x1; //Enable Preload Data Register
  //while (SERCOM1->SPI.SYNCBUSY.bit.CTRLB);

  //Set up SPI interrupts
  SERCOM1->SPI.INTENSET.bit.SSL = 0x1; //Enable Slave Select low interrupt
  SERCOM1->SPI.INTENSET.bit.RXC = 0x1; //Receive complete interrupt
  SERCOM1->SPI.INTENSET.bit.TXC = 0x1; //Transmit complete interrupt
  SERCOM1->SPI.INTENSET.bit.ERROR = 0x1; //Receive complete interrupt
  SERCOM1->SPI.INTENSET.bit.DRE = 0x1; //Data Register Empty interrupt
  //init SPI CLK
  //SERCOM1->SPI.BAUD.reg = SERCOM_FREQ_REF / (2*4000000u)-1;
  //Enable SPI
  SERCOM1->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE);
  SERCOM1->SPI.CTRLB.bit.RXEN = 0x1; //Enable Receiver, this is done here due to errate issue
  while (SERCOM1->SPI.SYNCBUSY.bit.CTRLB); //wait until receiver is enabled
}

int spi_recvbyte()
{
  return SPI.transfer(0);
}

void spi_sendbyte(int b)
{
  SPI.transfer(b);
}

void spi_csoff(void)
{
  SPI.endTransaction();
  digitalWrite(PIN_SPI_SS, HIGH);
}

void spi_cson(void)
{
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
}

void setupComm()
{
  int cnt = 0;
  for (int i = 0; i < 5; i++)
  {
    colorSingleDot(i, 240);
  }
  colorSingleDot(0, 120);
  SPI.begin();
  colorSingleDot(1, 120);
  spiSlave_init();
  colorSingleDot(2, 120);

  pinMode(PIN_SPI_SS, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH);
  colorSingleDot(3, 120);
  radioInitialized = (radioDefault() == 0xC6);
  colorSingleDot(4, 120);
}

#ifdef NOTUSED
void pollWatcher() {
  int LEDs = ivp / 1024;
  int i;
  Serial.print("ivp ");
  Serial.println(ivp);
  for ( i = 0; i < LEDs; i++ ) {

    pixel.setPixelColor(i, pixel.Color(0, 30, 0));
  }
  for ( ; i < 8; i++ ) {
    pixel.setPixelColor(i, pixel.Color(0, 00, 10));
  }
  if ( ivp > slaveLogSize ) {
    dispIntVal();

    float loopCnt = 0.0;
    while (true) {
      for (int n = 0; n < 8; n++) {

        pixel.setPixelColor(n, hsvColor((float)n * 45.0 + loopCnt, 1.0, 1.0));
      }
      loopCnt = loopCnt + 0.1;
      delay(20);

    }
  }

}
#endif
unsigned long nextDebugPrint = 0;

void pollComm()
{


  if (XN297L_regs[0x0f] != 0xC6)
  {
    radioDefault();

  }

  // xn_readreg(FIFO_STATUS);
  checkPacket();

  // Serial.println("Looping");
  unsigned long now = millis();
  if (now > nextDebugPrint)
  {
    nextDebugPrint = now + 500;
    if (showBrainStemLog)
    {
      dispIntVal();
    }
    if (showPacketLog)
    {
      dispPacketLog();
    }
    if (showXn297LLog)
    {
      dispXn297LLog();
    }

  }
  xn297L_debug();
}


