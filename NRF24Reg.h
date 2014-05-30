// Register addresses -----------------
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
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
#define DYNPD       0x1C
#define FEATURE     0x1D


// Bit Mnemonics ----------------------

// CONFIG
#define MASK_RX_DR  0x40
#define MASK_TX_DS  0x20
#define MASK_MAX_RT 0x10
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01
// EN_AA
#define ENAA_P5     0x20
#define ENAA_P4     0x10
#define ENAA_P3     0x08
#define ENAA_P2     0x04
#define ENAA_P1     0x02
#define ENAA_P0     0x01
// EN_RXADDR
#define ERX_P5      0x20
#define ERX_P4      0x10
#define ERX_P3      0x08
#define ERX_P2      0x04
#define ERX_P1      0x02
#define ERX_P0      0x01
// SETUP_AW
#define AW          0x01
// SETUP_RETR
#define ARD         0x10
#define ARC         0x01
// RF_SETUP
#define CONT_WAVE   0x80
#define RF_DR_LOW   0x20
#define PLL_LOCK    0x10
#define RF_DR_HIGH  0x08
#define RF_PA_HIGH  0x04
#define RF_PA_LOW   0x02
// STATUS
#define RX_DR       0x40
#define TX_DS       0x20
#define MAX_RT      0x10
#define RX_P_NO     0x02
#define TX_FULL     0x01
// OBSERVE_TX
#define PLOS_CNT    0x10
#define ARC_CNT     0x01
// FIFO_STATUS
#define TX_REUSE    0x40
#define TX_FULL_FIFO 0x20 // annoyingly this has the same mnemonic as in STATUS
#define TX_EMPTY    0x10
#define RX_FULL     0x02
#define RX_EMPTY    0x01
// DYNPD
#define DPL_P5      0x20
#define DPL_P4      0x10
#define DPL_P3      0x08
#define DPL_P2      0x04
#define DPL_P1      0x02
#define DPL_P0      0x01
// FEATURE
#define EN_DPL      0x04
#define EN_ACK_PAY  0x02
#define EN_DYN_ACK  0x01


// SPI Commands ----------------------
#define REGISTER_MASK       0x1F
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP                 0xFF