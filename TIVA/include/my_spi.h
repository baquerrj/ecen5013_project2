/*!
 * @file    my_spi.h
 *
 * @brief
 *
 *  Created on: Apr 19, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef _MY_SPI_H_
#define _MY_SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/debug.h"

#include "uart.h"

#define SPI_1MZ 1000000
#define SPI_2MZ 2000000

/*! Enum for SPI busses */
typedef enum
{
    SPI_0,
    SPI_1,
    SPI_2,
    SPI_3
} spi_e;

extern uint32_t g_sysClock;

extern const uint32_t SPI[4];

extern const uint32_t SPI_SYSCTL[4];

/*!
 * @brief   Enable the clock gate control for SPI
 * @returns void
 */
static inline void spi_clock_init( spi_e spi )
{
    MAP_SysCtlPeripheralEnable( SPI_SYSCTL[ spi ] );
    uint32_t src = SSIClockSourceGet( SPI[ spi ] );
    if( src == SSI_CLOCK_SYSTEM )
    {
        printf( "SSI Using System Clock\n" );
    }
    else if( src == SSI_CLOCK_PIOSC )
    {
        printf( "SSI Using PIOSC\n" );
    }

    SSIConfigSetExpClk( SPI[ spi ], g_sysClock, SSI_FRF_MOTO_MODE_0,
                        SSI_MODE_MASTER, SPI_1MZ, 8 );

}


/*!
 * @brief   Perform the initialization routine for the SPI module
 * Configure SPI in 3 wire mode and use a GPIO pin for chip select
 * @returns void
 */
void spi_init( spi_e spi );


/*!
 * @brief   Disable the GPIO pins earlier initialized for the SPI module
 * @returns void
 */
static inline void spi_disable( spi_e spi )
{
    SSIDisable( SPI[ spi ] );
    MAP_SysCtlPeripheralDisable( SPI_SYSCTL[ spi ] );
}

/*!
 * @brief   Blocks until SPI transmit buffer has completed transmitting
 * @returns void
 */
static inline void spi_flush( spi_e spi )
{
    while( SSIBusy(SPI[ spi ]) );
}

static inline void spi_flush_rx( spi_e spi )
{
    uint32_t throaway;
    while( SSIDataGetNonBlocking( SPI[ spi ], &throaway ) );
}


/*!
 * @brief   Read a si1ngle byte from the SPI bus
 * @returns uint8_t
 */
static inline uint8_t spi_read_byte( spi_e spi )
{
    uint32_t data;
    SSIDataGet( SPI[ spi ], &data );
    return ( (uint8_t)(data & 0xFF) );
}


/*!
 * @brief   Write a single byte on to the SPI bus
 * @returns void
 */
static inline void spi_write_byte( spi_e spi, uint8_t byte )
{
    SSIDataPut( SPI[ spi ],((uint32_t)byte & 0x000000FF) );
    spi_flush( spi );
}



/*!
 * @brief   Send a packet on to the SPI bus
 * Send multiple bytes given a pointer to an array and the number of bytes to be sent
 * @returns void
 */
void spi_write_packet( spi_e spi, const uint8_t* p, size_t length );

/*!
 * @brief   Read a packet from the SPI bus
 * Read multiple bytes given a pointer to an array for storage and the number of bytes to be read
 * @returns void
 */
void spi_read_packet( spi_e spi, uint8_t* p, size_t length );


#endif /* _MY_SPI_H_ */
