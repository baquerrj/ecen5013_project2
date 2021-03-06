/*!
 * @file  spi.c
 * @brief 
 *
 * <+DETAILED+>
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "spi.h"


spi_t SPI[ 2 ] = { NULL, NULL };
static uint32_t opened[ 2 ] = {0};

static spi_e spi_release( spi_e spi )
{
    if( spi > SPI_1 )
    {
        return -1;
    }

    mraa_result_t retVal = mraa_spi_stop( SPI[ spi ] );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_result_print( retVal );
        return -1;
    }
    return spi;
}


spi_e spi_init( spi_e spi )
{
    if( spi > SPI_1 )
    {
        return -1;
    }

    if( opened[ spi ] && (NULL != SPI[ spi ]) )
    {
        opened[ spi ]++;
        return spi;
    }

    mraa_spi_context spi_context = mraa_spi_init( spi );
    if( NULL == spi_context )
    {
        return 0;
    }
    
    SPI[ spi ] = spi_context;
    mraa_result_t retVal = mraa_spi_frequency( SPI[ spi ], SPI_1MZ );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_result_print( retVal );
        return spi_release( spi );
    }

    opened[ spi ]++;
    return spi;
}

spi_e spi_disable( spi_e spi )
{
    if( spi > SPI_1 )
    {
        return -1;
    }

    if( --opened[ spi ] )
    {
        return spi;
    }

    
    return spi_release( spi );
}


int8_t spi_write_packet( spi_e spi, const uint8_t *p, size_t len )
{
    if( spi > SPI_1 )
    {
        return -1;
    }

    uint8_t i = 0;
    while( i < len )
    {
        spi_write_byte( spi, *(p+i) );
        ++i;
    }
    return len;
}

int8_t spi_read_packet( spi_e spi, uint8_t *p, size_t len )
{
    if( spi > SPI_1 )
    {
        return -1;
    }

    uint8_t i = 0;
    while( i < len )
    {
        *(p+i) = spi_read_byte( spi );
        ++i;
    }
    return len;
}

void SPI0_IQRHandler( void )
{
    return;
}

    

