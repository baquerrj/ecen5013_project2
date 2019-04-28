/*!
 * @file	nRF240L.h
 *
 * @brief
 *
 *  Created on: Apr 19, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef _NRF240L_H_
#define _NRF240L_H_
#define RX_P_NO     1
#define TX_FULL     0
/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/* Memory Map */
#define NRF_REG_CONFIG       (0x00)
#define NRF_REG_EN_AA        (0x01)
#define NRF_REG_EN_RXADDR    (0x02)
#define NRF_REG_SETUP_AW     (0x03)
#define NRF_REG_SETUP_RETR   (0x04)
#define NRF_REG_RF_CH        (0x05)
#define NRF_REG_RF_SETUP     (0x06)
#define NRF_REG_STATUS       (0x07)
#define NRF_REG_OBSERVE_TX   (0x08)
#define NRF_REG_CD           (0x09)
#define NRF_REG_RX_ADDR_P0   (0x0A)
#define NRF_REG_RX_ADDR_P1   (0x0B)
#define NRF_REG_RX_ADDR_P2   (0x0C)
#define NRF_REG_RX_ADDR_P3   (0x0D)
#define NRF_REG_RX_ADDR_P4   (0x0E)
#define NRF_REG_RX_ADDR_P5   (0x0F)
#define NRF_REG_TX_ADDR      (0x10)
#define NRF_REG_RX_PW_P0     (0x11)
#define NRF_REG_RX_PW_P1     (0x12)
#define NRF_REG_RX_PW_P2     (0x13)
#define NRF_REG_RX_PW_P3     (0x14)
#define NRF_REG_RX_PW_P4     (0x15)
#define NRF_REG_RX_PW_P5     (0x16)
#define NRF_REG_FIFO_STATUS  (0x17)
#define NRF_REG_DYNPD	    (0x1C)
#define NRF_REG_FEATURE	    (0x1D)

//Commands Byte
#define NRF_CMD_FLUSH_TX     (0xE1)
#define NRF_CMD_FLUSH_RX     (0xE2)
#define NRF_CMD_W_TX_PAYLOAD    (0xA0)
#define NRF_CMD_R_RX_PAYLOAD    (0x61)
#define NRF_CMD_ACTIVATE     (0x50)
#define NRF_ACTIVATE_DATA    (0x73)
#define NRF_CMD_RXPAYLD_W    (0x60)
#define NRF_NOP              (0xFF)


#define DEFAULT_TX_ADDRESS_1B   (0xE7)
#define DEFAULT_TX_ADDRESS_2B   (0xE7)
#define DEFAULT_TX_ADDRESS_3B   (0xE7)
#define DEFAULT_TX_ADDRESS_4B   (0xE7)
#define DEFAULT_TX_ADDRESS_5B   (0xE7)

#define _BV(x)  (1ULL<<(x))

#define NRF_TX_ADDR_LEN      (5)

/* Bit Mnemonics */
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
#define RF_PWR      6
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0


/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

//Masks
#define NRF_RX_DR   6
#define NRF_TX_DS   5
#define NRF_MAX_RT  4

#define NRF_MASK_RX_DR      _BV(NRF_RX_DR)
#define NRF_MASK_TX_DS      _BV(NRF_TX_DS)
#define NRF_MASK_MAX_RT	    _BV(NRF_MAX_RT)

#define NRF_MASK_STATUS_RX_DR   NRF_MASK_RX_DR
#define NRF_MASK_STATUS_TX_DS  	NRF_MASK_TX_DS
#define NRF_MASK_STATUS_MAX_RT	NRF_MASK_MAX_RT

#define NRF_MASK_CONFIG_RX_DR   NRF_MASK_RX_DR
#define NRF_MASK_CONFIG_TX_DS  	NRF_MASK_TX_DS
#define NRF_MASK_CONFIG_MAX_RT	NRF_MASK_MAX_RT

#define NRF_CONFIG_MAX_RT_INT(x)     ((((uint8_t)x)<<NRF_MASK_CONFIG_MAX_RT)&(1<<NRF_MASK_CONFIG_MAX_RT))
#define NRF_CONFIG_RX_DR_INT(x)      ((((uint8_t)x)<<NRF_MASK_CONFIG_RX_DR)&(1<<NRF_MASK_CONFIG_RX_DR))

#define NRF_CONFIG_TX_DS_INT(x)      ((((uint8_t)x)<<NRF_MASK_CONFIG_TX_DS)&(1<<NRF_MASK_CONFIG_TX_DS))

#define NRF_PWR_UP                  1
#define NRF_MASK_PWR_UP             _BV(NRF_PWR_UP)
#define NRF_MASK_CONFIG_PWR_UP      NRF_MASK_PWR_UP
#define NRF_CONFIG_PWR_UP(x)        ((((uint8_t)x)<<NRF_MASK_CONFIG_PWR_UP)&(1<<NRF_MASK_CONFIG_PWR_UP))

#define NRF_PRIM_RX                 0
#define NRF_MASK_PRIM_RX            _BV(NRF_PRIM_RX)
#define NRF_MASK_CONFIG_PRIM_RX     NRF_MASK_PRIM_RX
#define NRF_CONFIG_PRIM_RX(x)       ((((uint8_t)x)<<NRF_MASK_CONFIG_PRIM_RX)&(1<<NRF_MASK_CONFIG_PRIM_RX))


#define NRF_MASK_STATUS_TX_FULL          (1<<0)
#define NRF_MASK_FIFO_STATUS_TX_FULL     (1<<5)
#define NRF_MASK_FIFO_STATUS_RX_FULL     (1<<1)
#define NRF_MASK_FIFO_STATUS_TX_EMPTY    (1<<4)
#define NRF_MASK_FIFO_STATUS_RX_EMPTY    (0<<5)

#define NRF_MASK_INT_MAXRT   (1<<3)
#define NRF_MASK_INT_TXDS    (1<<4)
#define NRF_MASK_INT_TXDR    (1<<5)






#endif /* _NRF240L_H_ */
