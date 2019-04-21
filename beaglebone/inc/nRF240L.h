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


//Commands Byte
#define NORDIC_TXFIFO_FLUSH_CMD (0xE1)
#define NORDIC_RXFIFO_FLUSH_CMD (0xE2)
#define NORDIC_W_TXPAYLD_CMD    (0xA0)
#define NORDIC_R_RXPAYLD_CMD    (0x61)
#define NORDIC_ACTIVATE_CMD     (0x50)
#define NORDIC_ACTIVATE_DATA    (0x73)
#define NORDIC_RXPAYLD_W_CMD    (0x60)
#define NORDIC_NOP              (0xFF)
//Register Addresses
#define NORDIC_CONFIG_REG       (0x00)
#define NORDIC_EN_AA_REG        (0x01)
#define NORDIC_EN_RXADDR_REG    (0x02)
#define NORDIC_SETUP_AW         (0x03)
#define NODIC_SETUP_RETR_REG    (0x04)
#define NORDIC_STATUS_REG       (0x07)
#define NORDIC_RF_SETUP_REG     (0x06)
#define NORDIC_RF_CH_REG        (0x05)
#define NORDIC_TX_ADDR_REG      (0x10)
#define NORDIC_TX_ADDR_LEN      (5)

#define NORDIC_RX_ADDR_P0_REG   (0x0A)
#define NORDIC_RX_ADDR_P1_REG   (0x0B)
#define NORDIC_RX_ADDR_P2_REG   (0x0C)
#define NORDIC_RX_ADDR_P3_REG   (0x0D)
#define NORDIC_RX_ADDR_P4_REG   (0x0E)
#define NORDIC_RX_ADDR_P5_REG   (0x0F)

#define NORDIC_FIFO_STATUS_REG  (0x17)
#define NORDIC_RX_PW_P0_REG     (0x11)

#define DEFAULT_TX_ADDRESS_1B   (0xE7)
#define DEFAULT_TX_ADDRESS_2B   (0xE7)
#define DEFAULT_TX_ADDRESS_3B   (0xE7)
#define DEFAULT_TX_ADDRESS_4B   (0xE7)
#define DEFAULT_TX_ADDRESS_5B   (0xE7)

//Masks
#define NORDIC_STATUS_RX_DR_MASK    (1<<6)
#define NORDIC_STATUS_TX_DS_MASK  	(1<<5)
#define NORDIC_STATUS_MAX_RT_MASK	(1<<4)

#define NORDIC_CONFIG_MAX_RT_MASK      	4
#define NORDIC_CONFIG_MAX_RT_INT(x)     ((((uint8_t)x)<<NORDIC_CONFIG_MAX_RT_MASK)&(1<<NORDIC_CONFIG_MAX_RT_MASK))

#define NORDIC_CONFIG_RX_DR_MASK        6
#define NORDIC_CONFIG_RX_DR_INT(x)      ((((uint8_t)x)<<NORDIC_CONFIG_RX_DR_MASK)&(1<<NORDIC_CONFIG_RX_DR_MASK))

#define NORDIC_CONFIG_TX_DS_MASK        5
#define NORDIC_CONFIG_TX_DS_INT(x)      ((((uint8_t)x)<<NORDIC_CONFIG_TX_DS_MASK)&(1<<NORDIC_CONFIG_TX_DS_MASK))

#define NORDIC_CONFIG_PWR_UP_MASK       1
#define NORDIC_CONFIG_PWR_UP(x)         ((((uint8_t)x)<<NORDIC_CONFIG_PWR_UP_MASK)&(1<<NORDIC_CONFIG_PWR_UP_MASK))

#define NORDIC_CONFIG_PRIM_RX_MASK      0
#define NORDIC_CONFIG_PRIM_RX(x)        ((((uint8_t)x)<<NORDIC_CONFIG_PRIM_RX_MASK)&(1<<NORDIC_CONFIG_PRIM_RX_MASK))


#define NORDIC_STATUS_TX_FULL_MASK          (1<<0)
#define NORDIC_FIFO_STATUS_TX_FULL_MASK     (1<<5)
#define NORDIC_FIFO_STATUS_RX_FULL_MASK     (1<<1)
#define NORDIC_FIFO_STATUS_TX_EMPTY_MASK    (1<<4)
#define NORDIC_FIFO_STATUS_RX_EMPTY_MASK    (0<<5)

#define NORDIC_INT_MAXRT_MASK   (1<<3)
#define NORDIC_INT_TXDS_MASK    (1<<4)
#define NORDIC_INT_TXDR_MASK    (1<<5)


#endif /* _NRF240L_H_ */
