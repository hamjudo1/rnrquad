typedef unsigned char uint8_t;

extern void pollComm();
extern void setupComm();
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

extern int spi_recvbyte( void);
extern void spi_sendbyte( int b);
extern void spi_cson(void); // start of packet stuff.
extern void spi_csoff(void); // end of packet
extern void spiSlave_init(); // initialize SPI slave (connection to brain stem)

typedef enum {
  RX_MODE_BIND,
  RX_MODE_NORMAL
} rx_mode_t;
extern rx_mode_t rxmode;

