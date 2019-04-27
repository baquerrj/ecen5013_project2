/*!
 * @file  nrf_module.h
 * @brief Nordic nRF24 Transceiver Interface
 *
 * BeagleBone Green and nRF24 Transceiver interface. Reference: https://github.com/mlsorensen/RF24
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
#include "nRF240L.h"
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
    NRF_DR_1MBPS = 0,
    NRF_DR_2MBPS = 1

} nrf_data_rate_e;

typedef enum
{
    NRF_POWER_LOW = 0,
    NRF_POWER_MED = 2,
    NRF_POWER_HIGH = 3

} nrf_power_e;

typedef enum
{
    NRF_CRC_DISABLED = 0,
    NRF_CRC_8,
    NRF_CRC_16
} nrf_crc_e;

typedef enum
{
    NRF_PA_MIN = 0,
    NRF_PA_LOW,
    NRF_PA_HIGH,
    NRF_PA_MAX,
    NRF_PA_ERROR
} nrf_pa_level_e;

/*! Chip-Select GPIO */
extern mraa_gpio_context NRF_CSN_GPIO;

/*! Chip-Enable GPIO */
extern mraa_gpio_context NRF_CE_GPIO;

/*!
 * @brief - Enable the chip select connection to Nordic
 * @return void
 */
static inline void nrf_chip_enable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CSN_GPIO, 0 );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_result_print( retVal );
    }
    delayUs(50);
}

/*!
 * @brief - Disable the chip select connection to Nordic
 * @return void
 */
static inline void nrf_chip_disable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CSN_GPIO, 1 );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_result_print( retVal );
    }
}

/*!
 * @brief - Enable TX/RX from the Nordic module
 * @return void
 */
static inline void nrf_radio_enable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CE_GPIO, 1 );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_result_print( retVal );
    }
}

/*!
 * @brief - Disable TX/RX from the Nordic module
 * @return void
 */
static inline void nrf_radio_disable( void )
{
    mraa_result_t retVal = mraa_gpio_write( NRF_CE_GPIO, 0 );
    if( MRAA_SUCCESS != retVal )
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

    spi_write_byte( SPI_0, command );
    spi_read_byte( SPI_0 );

    nrf_chip_disable();
    return;
}


/*!
 * @brief - Initialize the nrf module
 * Initialized the GPIO connections pertaining to the Nordic module
 * @return void
 */

uint8_t nrf_module_init( void );

/*!
 * @brief - Powers on NRF24 module and writing default configuration
 *
 * @returns void
 */
void nrf_power_on( void );

/*!
 * @brief Powers off NRF24 module
 * @return void
 */
void nrf_power_off( void );


void nrf_set_retries( uint8_t delay, uint8_t count );

uint8_t nrf_get_retries( void );

void nrf_set_crc_length( nrf_crc_e length );

nrf_crc_e nrf_get_crc_length( void );


void nrf_set_palevel( nrf_pa_level_e level );

nrf_pa_level_e nrf_get_palevel( void );

void nrf_listen( void );

void nrf_stop_listening( void );

/*!
 * @brief - Read a register from the nrf module
 * @param - reg uint8_t
 * @return uint8_t
 */
static inline uint8_t nrf_read_register( uint8_t reg )
{
   uint8_t data = 0;

   nrf_chip_disable();
   nrf_chip_enable();

   spi_write_byte( SPI_0, reg );
   data = spi_read_byte( SPI_0 );

   nrf_chip_disable();
   return data;
}

/*!
 * @brief - Write to a register from the nrf module
 * @param - reg uint8_t
 * @param - value uint8_t
 * @return void
 */
static inline void nrf_write_register( uint8_t reg, uint8_t value )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_0, reg | 0x20 );
    //spi_read_byte( SPI_0 );
    spi_write_byte( SPI_0, value );
    //spi_read_byte( SPI_0 );SPI

    nrf_chip_disable();
    return;
}


static inline uint8_t nrf_write_packet( uint8_t reg, const uint8_t *buf, uint8_t len )
{
    uint8_t status;

    nrf_chip_disable();
    nrf_chip_enable();

    status = spi_write_byte( SPI_0, W_REGISTER | (REGISTER_MASK & reg ) );
    while( len-- )
    {
        spi_write_byte( SPI_0, *buf++ );
    }

    nrf_chip_disable();
    return status;
}

/*!
 * @brief - Write to the nrf module's status register
 * @param - statusValue uint8_t
 * @return void
 */
void nrf_write_status( uint8_t statusValue );

/*!
 * @brief - Read the nrf module's status register
 * @return uint8_t
 */
uint8_t nrf_read_status( void );

/*!mraa_spi_write
 * @brief - Write to the nrf module's config register
 * @param - configValue uint8_t
 * @return void
 */
void nrf_write_config( uint8_t configValue );

/*!
 * @brief - Read the nrf module's config register
 * @return uint8_t
 */
uint8_t nrf_read_config( void );


/*!
 * @brief - Read the nrf module's RF setup register
 * @return uint8_t
 */
uint8_t nrf_read_rf_setup( void );

/*!
 * @brief - Write to the nrf module's RF setup register
 * @param - rfStatusValue uint8_t
 * @return void
 */
void nrf_write_rf_setup( uint8_t rfSetupValue );

/*!
 * @brief - Read the nrf module's RF CH register
 * @return uint8_t
 */
uint8_t nrf_get_channel( void );

/*!
 * @brief - Write to the nrf module's RF CH register
 * @param - channel uint8_t
 * @return void
 */
void nrf_set_channel( uint8_t channel );

/*!
 * @brief - Reads 5 bytes of the nrf module's tx ADDR register
 * @param - address uint8_t *
 * @return void
 */
void nrf_read_tx_addr( uint8_t * address );

/*!
 * @brief - Writes 5 bytes of the nrf module's tx ADDR register
 * @param - tx_addr uint8_t *
 * @return void
 */
void nrf_write_tx_addr( uint8_t * tx_addr );

/*!
 * @brief - Read the nrf module's FIFO status register
 * @return address uint8_t
 */
uint8_t nrf_read_fifo_status( void );

/*!
 * @brief - Send the command FLUSH_tx to the nrf module
 * @return void
*/
void nrf_flush_tx( void );

/*!
 * @brief - Send the command FLUSH_RX to the nrf module
 * @return void
 */
void nrf_flush_rx( void );


/*!
 * @brief - Send the activation command to the nrf module
 * Activates the features: R_RX_PL_WID, W_ACK_PAYLOAD, W_tx_PAYLOAD_NOACK
 * @return void
 */
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


#endif   /* _NRF_MODULE_H_ */
