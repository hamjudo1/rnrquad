// From H8mini_blue_board/Silverware/src/drv_xn297_3wire.c
// THe silverware hardware is doing 3 wire SPI in software.
// The M0 has hardware for 4 wire SPI, which we convert to 3 wire by adding a resistor
// and ignoring the bytes we read back as we send, and by sending 0 bytes when we are receiving.
//
#include <SPI.h>  // just to get definition of bool

#include "state.h"

int XN297L_regs[32];
uint8_t XN297L_payloadIn[2][15]; // float buffered
uint8_t XN297L_payloadOut[2][15];
int XN297L_goodPayloadIn = 0;
int XN297L_goodPayloadOut = 0;
int XN297L_goodBuffer = 0;

int xn297_regs_written = 0;
int xn297_regs_read = 0;

void xn_writereg(int reg, int val)
{
  xn297_regs_written++;
  reg = reg & 0x3F;
  reg = reg | 0x20;
  spi_cson();
  spi_sendbyte(reg);
  spi_sendbyte(val);
  spi_csoff();
}

void xn_writereg(int reg, uint8_t data[], uint8_t size)
{
  xn297_regs_written++;
  reg = reg & 0x3F;
  reg = reg | 0x20;
  spi_cson();
  spi_sendbyte(reg);
  for (uint8_t i = 0; i < size; i++)
  {
    spi_sendbyte(data[i]);
  }
  spi_csoff();
}

int xn_readreg(int reg)
{
  xn297_regs_read++;
  reg = reg & 0x1F;
  spi_cson();
  spi_sendbyte(reg);
  int val = spi_recvbyte();
  spi_csoff();
  XN297L_regs[reg] = val;
  return val;
}

int statusValCount[256];

void checkPacket()
{
  int status = xn_readreg(XN_STATUS);
  statusValCount[status]++;
  uint8_t outbound[15];

  if ((status & B00001110) != B00001110)
  {
    xn_readpayload(XN297L_payloadIn[!XN297L_goodPayloadIn], 15);
    xn_writereg(XN_STATUS, B00001110); // Clear status.

    XN297L_goodPayloadIn = !XN297L_goodPayloadIn;
    processPacket(XN297L_payloadIn[XN297L_goodPayloadIn]);
  }
}

int xn_command(int command)
{
  spi_cson();
  spi_sendbyte(command);
  spi_csoff();
  return 0;
}


void xn_readpayload(uint8_t *data, int size)
{
  int index = 0;
  spi_cson();
  spi_sendbyte(B01100001); // read rx payload
  while (index < size)
  {
    data[index] = spi_recvbyte();
    index++;
  }
  spi_csoff();
}


void xn_writerxaddress(int *addr)
{
  int index = 0;
  spi_cson();
  spi_sendbyte(0x2a);
  while (index < 5)
  {
    spi_sendbyte(addr[index]);
    index++;
  }
  spi_csoff();
}


void xn_writetxaddress(int *addr)
{
  int index = 0;
  spi_cson();
  spi_sendbyte(0x10 | 0x20);
  while (index < 5)
  {
    spi_sendbyte(addr[index]);
    index++;
  }
  spi_csoff();
}


void xn_writepayload(int data[], int size)
{
  int index = 0;
  uint8_t csum = 0;
  spi_cson();
  spi_sendbyte(0xA0); // write tx payload
  size--; // calculate and send an accurate checksum.
  while (index < size)
  {
    csum = csum + data[index];
    spi_sendbyte(data[index]);
    index++;
  }
  spi_sendbyte(csum);
  spi_csoff();
}

