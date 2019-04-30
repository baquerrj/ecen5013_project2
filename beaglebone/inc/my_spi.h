/*!
 * @file  my_spi.h
 * @brief 
 *
 *  <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/20/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


#ifndef  _MY_SPI_H
#define  _MY_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "mraa/spi.h"


#define SPI_1MZ 1000000
#define SPI_2MZ 2000000

#define NOP 0xFF
/*! Enum for possible SPI busses */
typedef enum
{
    SPI_0,
    SPI_1,
    SPI_MAX
} spi_e;


typedef mraa_spi_context spi_t;

spi_t SPI[ SPI_MAX ];

/*!
 * @brief   Initialize SPI bus
 * Configure SPI in 3 wire mode and use a GPIO pin for chip select
 *
 * @returns ID of initialized SPI
 */
spi_e spi_init( spi_e spi );


/*!
 * @brief   Disable the GPIO pins earlier initialized for the SPI module
 * @returns void
 */
spi_e spi_disable( spi_e spi );


/*!
 * @brief   Blocks until SPI RX buffer is flushed
 *
 * @returns void
 */
static inline void spi_flush_rx( spi_e spi )
{
    while( mraa_spi_write( SPI[ spi ], NOP ) );
    return;
}


/*!
 * @brief   Read a single byte from the SPI bus
 * @returns uint8_t
 */
static inline int8_t spi_read_byte( spi_e spi )
{
    return mraa_spi_write( SPI[ spi ], NOP );
}


/*!
 * @brief   Write a single byte on to the SPI bus
 */
static inline int8_t spi_write_byte( spi_e spi, uint8_t byte )
{
    return mraa_spi_write( SPI[ spi ], byte );
}


/*!
 * @brief   Send a packet on to the SPI bus
 * Send multiple bytes given a pointer to an array and the number of bytes to be sent
 * @returns number of bytes written
 */
int8_t spi_write_packet( spi_e spi, const uint8_t* p, size_t len );


/*!
 * @brief   Read a packet from the SPI bus
 * Read multiple bytes given a pointer to an array for storage and the number of bytes to be read
 * @returns number of bytes read
 */
int8_t spi_read_packet( spi_e spi, uint8_t* p, size_t len );




#endif   /* _MY_SPI_H */
