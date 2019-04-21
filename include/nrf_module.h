/*!
 * @file	nrf_module.h
 *
 * @brief
 *
 *  Created on: Apr 20, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef _NRF_MODULE_H_
#define _NRF_MODULE_H_

#include "nRF240L.h"
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "driverlib/gpio.h"
#include "delay.h"
#include "my_spi.h"

typedef void (*interrupt_handler_t)( void );

typedef enum
{
    NRF_MODE_TX = 0,
    NRF_MODE_RX = 1

} nrf_mode_e;

typedef enum
{
    NRF_DR_1Mbps = 0,
    NRF_DR_2Mbps = 1

} nrf_data_rate_e;

typedef enum
{
    NRF_POWER_LOW = 0,
    NRF_POWER_MED = 2,
    NRF_POWER_HIGH = 3

} nrf_power_e;

extern uint32_t g_sysClock;


/*!
* @brief - Enable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_enable( void )
{
    GPIOPinWrite( NORDIC_CSN_PORT, NORDIC_CSN_PIN, 0 );
    delayUs(50);
}

/*!
* @brief - Disable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_disable( void )
{
    GPIOPinWrite( NORDIC_CSN_PORT, NORDIC_CSN_PIN, NORDIC_CSN_PIN );
}

/*!
* @brief - Enable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_enable( void )
{
    GPIOPinWrite( NORDIC_CE_PORT,NORDIC_CE_PIN, NORDIC_CE_PIN );
}

/*!
* @brief - Disable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_disable( void )
{
    GPIOPinWrite( NORDIC_CE_PORT,NORDIC_CE_PIN, 0 );
}


/*!
 * @brief Send command to NRF module
 *
 * @param[in] command
 */
static inline void nrf_write_command( uint8_t command )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, command );
    spi_read_byte( SPI_1 );

    nrf_chip_disable();
    return;
}

/*!
* @brief - Initialize the nrf module
* Initialized the GPIO connections pertaining to the Nordic module
* @return void
**/
int8_t nrf_module_init( uint8_t use_interrupt, interrupt_handler_t handler );

/*!
* @brief - Disable the GPIO connections set up earlier for the Nordic module
* @return void
**/
void nrf_module_disable( void );

/*!
* @brief - Read a register from the nrf module
* @param - reg uint8_t
* @return uint8_t
**/
uint8_t nrf_read_register( uint8_t reg );

/*!
* @brief - Write to a register from the nrf module
* @param - reg uint8_t
* @param - value uint8_t
* @return void
**/
void nrf_write_register( uint8_t reg, uint8_t value );

/*!
* @brief - Write to the nrf module's status register
* @param - statusValue uint8_t
* @return void
**/
void nrf_write_status( uint8_t statusValue );

/*!
* @brief - Read the nrf module's status register
* @return uint8_t
**/
uint8_t nrf_read_status( void );

/*!
* @brief - Write to the nrf module's config register
* @param - configValue uint8_t
* @return void
**/
void nrf_write_config( uint8_t configValue );

/*!
* @brief - Read the nrf module's config register
* @return uint8_t
**/
uint8_t nrf_read_config( void );


/*!
* @brief - Read the nrf module's RF setup register
* @return uint8_t
**/
uint8_t nrf_read_rf_setup( void );

/*!
* @brief - Write to the nrf module's RF setup register
* @param - rfStatusValue uint8_t
* @return void
**/
void nrf_write_rf_setup( uint8_t rfSetupValue );

/*!
* @brief - Read the nrf module's RF CH register
* @return uint8_t
**/
uint8_t nrf_read_rf_ch( void );

/*!
* @brief - Write to the nrf module's RF CH register
* @param - channel uint8_t
* @return void
**/
void nrf_write_rf_ch( uint8_t channel );

/*!
* @brief - Reads 5 bytes of the nrf module's tx ADDR register
* @param - address uint8_t *
* @return void
**/
void nrf_read_tx_addr( uint8_t * address );

/*!
* @brief - Writes 5 bytes of the nrf module's tx ADDR register
* @param - tx_addr uint8_t *
* @return void
**/
void nrf_write_tx_addr( uint8_t * tx_addr );

/*!
* @brief - Read the nrf module's FIFO status register
* @return address uint8_t
**/
uint8_t nrf_read_fifo_status( void );

/*!
 * @brief - Send the command FLUSH_tx to the nrf module
 * @return void
*/
void nrf_flush_tx_fifo( void );

/*!
* @brief - Send the command FLUSH_RX to the nrf module
* @return void
**/
void nrf_flush_rx_fifo( void );


/*!
* @brief - Send the activation command to the nrf module
* Activates the features: R_RX_PL_WID, W_ACK_PAYLOAD, W_tx_PAYLOAD_NOACK
* @return void
**/
void nrf_module_setup( nrf_data_rate_e data_rate, nrf_power_e power);


void nrf_read_rx_pipe_addr( uint8_t pipe_num, uint8_t *address );
void nrf_write_rx_pipe_addr( uint8_t pipe_num, uint8_t *rx_addr );

void nrf_write_En_AA( uint8_t data );
uint8_t nrf_read_En_AA( void );
void nrf_write_setup_retry( uint8_t data );
uint8_t nrf_read_setup_retry( void );

uint8_t nrf_read_data( uint8_t *data, uint8_t len );
uint8_t nrf_transmit_data( uint8_t *data, uint8_t len, uint8_t toRXMode );

void nrf_write_tx_payload( uint8_t *data, uint8_t len );
void nrf_tx_pulse( void );


void nrf_open_read_pipe( uint8_t rx_pipe_number, uint8_t rx_addr[5], uint8_t payload_size );

void nrf_open_write_pipe( uint8_t tx_addr[5] );

void nrf_close_write_pipe( void );

void nrf_close_read_pipe( uint8_t rx_pipe_number );

void nrf_init_test( void );


#endif   /* _NRF_MODULE_H */
