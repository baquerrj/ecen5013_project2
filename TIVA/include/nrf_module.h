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
    NRF_DR_1MBPS = 0,
    NRF_DR_2MBPS = 1,
    NRF_DR_250KBPS,
    NRF_DR_MAX
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
    NRF_CRC_16,
    NRF_CRC_MAX
} nrf_crc_e;

typedef enum
{
    NRF_PA_MIN = 0,
    NRF_PA_LOW,
    NRF_PA_HIGH,
    NRF_PA_MAX,
    NRF_PA_ERROR
} nrf_pa_level_e;



extern uint32_t g_sysClock;

/*!
* @brief - Enable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_enable( void )
{
    GPIOPinWrite( NRF_CSN_PORT, NRF_CSN_PIN, 0 );
    delayUs(50);
}

/*!
* @brief - Disable the chip select connection to Nordic
* @return void
**/
static inline void nrf_chip_disable( void )
{
    GPIOPinWrite( NRF_CSN_PORT, NRF_CSN_PIN, NRF_CSN_PIN );
}

/*!
* @brief - Enable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_enable( void )
{
    GPIOPinWrite( NRF_CE_PORT, NRF_CE_PIN, NRF_CE_PIN );
}

/*!
* @brief - Disable TX/RX from the Nordic module
* @return void
**/
static inline void nrf_radio_disable( void )
{
    GPIOPinWrite( NRF_CE_PORT, NRF_CE_PIN, 0 );
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

    spi_write_byte( NRF_SPI, command );
    spi_read_byte( NRF_SPI );

    nrf_chip_disable();
    return;
}

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

    spi_write_byte( NRF_SPI, reg );
    spi_read_byte( NRF_SPI );
    spi_write_byte( NRF_SPI, NRF_NOP );
    data = spi_read_byte( NRF_SPI );

    nrf_chip_disable();
    return data;
}


/*!
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
static inline uint8_t nrf_read_packet( uint8_t reg, uint8_t *buf, uint8_t len )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( NRF_SPI, R_REGISTER | (REGISTER_MASK & reg) );
    spi_read_byte( NRF_SPI );
    spi_read_packet( NRF_SPI, buf, len );

    nrf_chip_disable();
    return 0;
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

    spi_write_byte( NRF_SPI, W_REGISTER | (REGISTER_MASK & reg) );
    spi_read_byte( NRF_SPI );
    spi_write_byte( NRF_SPI, value );
    spi_read_byte( NRF_SPI );

    nrf_chip_disable();
    return;
}

/*!
 * @brief - Write to a register from the nrf moudle
 * @param[in]   register Register to write to
 * @param[in]   buf Where to get data from
 * @param[in]   len Number of bytes to write
 * @returns 0
 */
static inline uint8_t nrf_write_packet( uint8_t reg, const uint8_t *buf, uint8_t len )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( NRF_SPI, W_REGISTER | (REGISTER_MASK & reg) );
    spi_write_packet( NRF_SPI, buf, len );

    nrf_chip_disable();
    return 0;
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

uint8_t nrf_set_datarate( nrf_data_rate_e speed );

/*!
 * @brief Set the number and delay of retries upon failed submit
 *
 * @param[in]   delay How long to wait between each retry (multiples of 250us)
 * @param[in]   count How many retires before giving up
 */
void nrf_set_retries( uint8_t delay, uint8_t count );

/*!
 * @brief Get number of retries configured
 *
 * @returns number of retries
 */
uint8_t nrf_get_retries( void );

/*!
 * @brief Set the CRC length
 *
 * @param[in]   length NRF_CRC_8 for 8-bit, or NRF_CR_16 for 16-bit CRC
 * @returns void
 */
void nrf_set_crc_length( nrf_crc_e length );

/*!
 * @brief Get the CRC length
 *
 * @returns NRF_CRC_DISABLED if disabled
 */
nrf_crc_e nrf_get_crc_length( void );

/*!
 * @brief St Power Amplifier (PA) level to one of four levels
 *
 * @param[in]   level Desired PA level
 */
void nrf_set_palevel( nrf_pa_level_e level );

/*!
 * @brief Fetches the current PA level
 *
 * @returns current PA level
 */
nrf_pa_level_e nrf_get_palevel( void );

/*!
 * @brief Start listening on the open reading pipes
 *
 * It is expected that nrf_open_reading_pipe() is called first. Call
 * nrf_stop_listening() before call nrf_write(). Call nrf_available() to check
 * for incoming traffic, and nrf_read() to get it.
 *
 * @returns void
 */
void nrf_start_listening( void );

/*!
 * @brief Stop listening for incoming messages
 *
 * Call prior to calling nrf_write()
 *
 * @returns void
 */
void nrf_stop_listening( void );


/*!
 * @brief Write the transmit payload
 *
 * The size of the data written is fixed by nrf_set_payload_size()
 * @param[im]   buf Where to get the data from
 * @param[in]   len number of bytes to send
 * @return Current value of the status register
 */
uint8_t nrf_write_payload( const void *buf, uint8_t len, const uint8_t writeType );


/*!
 * @brief Read the receive payload
 * The size ofthe data written is the fixed payload size
 *
 * @param[in]   buf where to put the data
 * @param[in]   len number of bytes to read
 * @returns Curent value of the status register
 */
uint8_t nrf_read_payload( uint8_t *buf, uint8_t len );

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


uint8_t nrf_get_dynamic_payload_size( void );


void what_happened( uint8_t *tx_ok, uint8_t *tx_fail, uint8_t *rx_ready );


/*!
 * @brief Turn on or off the special features of the chip
 *
 * @returns void
 */
void nrf_toggle_features( void );


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
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
uint8_t nrf_write( void *data, uint8_t len );


/*!
 * @brief Tests where there are bytes available to read
 *
 * @returns 0 if no payload available
 */
uint8_t nrf_available( void );



/*!
 * @brief Read the payload
 *
 * @param[out]  buf pointer to store data
 * @param[in]   len number of bytes to read
 * @returns 0 if payload was not delivered successfully
  */
uint8_t nrf_read( void *buf, uint8_t len );




/*!
 * @brief Open a pipe for writing
 *
 * Only one pipe can be open at a time
 *
 * @param[in]   address The 40-bit address of the pipe to open. This can
 * be any value whatsoever, as long as you are the only one writing to it
 * and only one other radio is listening to it.
 * @returns void
 */
void nrf_open_writing_pipe( uint64_t address );



/*!
 * @brief Open a pipe for reading
 *
 * Up to 6 pipes can be open for reading at one. Open all the reading
 * pipes and then call nrf_start_listening()
 *
 * @param[in]   number Which pipe to open (0-5)
 * @param[in]   address The 40-bit address of the pipe to open
 * @returns void
 */
void nrf_open_reading_pipe( uint8_t number, uint64_t address );

void nrf_tx_pulse( void );

void nrf_close_write_pipe( void );

void nrf_close_read_pipe( uint8_t rx_pipe_number );

void nrf_init_test( void );

/*!
 * @brief Decode and print contents of status register
 *
 * @param[in]   status content of NRF status register
 * @returns void
 */
void print_status( uint8_t status );
void print_details( void );

#endif   /* _NRF_MODULE_H */
