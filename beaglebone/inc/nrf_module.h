/*!
 * @file  nrf_module.h
 * @brief 
 *
 *  <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/21/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


#ifndef  _NRF_MODULE_H_
#define  _NRF_MODULE_H_

#include <stdbool.h>
#include <stdint.h>

#include "my_spi.h"
#include "delay.h"

#include "mraa/spi.h"
#include "mraa/gpio.h"

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


/*! Chip-Select GPIO */
extern mraa_gpio_context NRF_CSN_GPIO;

/*! Chip-Enable GPIO */
extern mraa_gpio_context NRF_CE_GPIO;

/*!
* @brief - Enable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_enable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CSN_GPIO, 0 );
    if( MRAA_SUCESS != retVal )
    {
        mraa_result_print( retVal );
    }   
    delayUs(50);
}

/*!
* @brief - Disable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_disable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CSN_GPIO, 1 );
    if( MRAA_SUCESS != retVal )
    {
        mraa_result_print( retVal );
    }   
}

/*!
* @brief - Enable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_enable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CE_GPIO, 1 );
    if( MRAA_SUCESS != retVal )
    {
        mraa_result_print( retVal );
    }
}

/*!
* @brief - Disable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_disable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CE_GPIO, 0 );
    if( MRAA_SUCESS != retVal )
    {
        mraa_result_print( retVal );
    }
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
static inline uint8_t nrf_read_register( uint8_t reg )
{
   uint8_t data = 0;

   nrf_chip_disable();
   nrf_chip_enable();

   spi_write_byte( SPI_1, reg );
   data = spi_read_byte( SPI_1 );

   nrf_chip_disable();
   return data;
}

/*!
* @brief - Write to a register from the nrf module
* @param - reg uint8_t
* @param - value uint8_t
* @return void
**/
static inline void nrf_write_register( uint8_t reg, uint8_t value )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, reg | 0x20 );
    spi_read_byte( SPI_1 );
    spi_write_byte( SPI_1, value );
    spi_read_byte( SPI_1 );

    nrf_chip_disable();
    return;
}


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
nrf_chip_disable();
    nrf_chip_enable();

    n
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


#endif   /* _NRF_MODULE_H_ */
