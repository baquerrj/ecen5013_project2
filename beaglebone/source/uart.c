/*!
 * @file  my_uart.c
 * @brief 
 *
 * <+DETAILED+>
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#include "uart.h"
#define BAUD_115200    B115200
#define BAUD_921600    B921600

#define BAUDRATE    BAUD_115200

const char *const PORT[5] = {"","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4"};

static struct termios _config;

static int32_t opened = 0;
static int my_fd = -1;

int uart_open( port_e com_port )
{
    if( COM_PORT_4 != com_port )
    {
        return -1;
    }

    if( (0 < opened) && (-1 != my_fd ) )
    {
        if( 64 > opened )
        {
            return my_fd;
        }
        else
        {
            return -1;
        }
    }

    int fd;
    struct termios config;

    fd = open( PORT[ COM_PORT_4 ], O_RDWR | O_NOCTTY | O_SYNC );
    if( 0 > fd )
    {
        perror( PORT[ com_port ] );
        return -1;
    }

    tcgetattr( fd, &_config );

    bzero( &config, sizeof( config ) );

    config.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    cfmakeraw( &config );
    config.c_cc[ VMIN ]  = 0;
    config.c_cc[ VTIME ] = 1;

    tcflush( fd, TCIFLUSH );
    tcsetattr( fd, TCSANOW, &config );

    my_fd = fd;
    opened++;
    return fd;
}


void uart_close( int uart )
{
    opened--;
    if( 0 > opened )
    {
        if( my_fd == uart )
        {
            tcsetattr( uart, TCSANOW, &_config );
            close( uart );
            my_fd = -1;
        }
    }
    return;
}


void uart_flush( void )
{
    if( my_fd > 0 )
    {
        tcflush( my_fd, TCIFLUSH );
    }
    return;
}


int32_t uart_putraw( void *p, size_t len )
{
    if( my_fd < 0 )
    {
        return -1;
    }

    int32_t bytes = 0;
    int32_t tries = 0;
    while( (tries < 16) && (bytes < len) )
    {
        bytes += write( my_fd, p+bytes, len-bytes);
        tries++;
    }
    return bytes;
}


int32_t uart_putchar( char c )
{
    return write( my_fd, &c, 1 );
}


int32_t uart_putstr( const char *str )
{
    return uart_putraw( (void*)str, strlen(str) );
}

int32_t uart_read( void *p, size_t len )
{
    if( 0 > my_fd )
    {
        return -1;
    }

    int ret = 0, retry = 0, i = 0;
    do
    {
        ret = read( my_fd, p+i, len-i );
        i += ret;
        retry++;
    } while( (ret > -1) && (i < len) && (retry < 16) );

    return ret;
}

int32_t uart_new_data( uint32_t ms )
{
    if( my_fd < 0 )
    {
        return -1;
    }

    struct timeval timeout;

    if( ms == 0 )
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
    } 
    else 
    {
        timeout.tv_sec = ms / 1000;
        timeout.tv_usec = (ms % 1000) * 1000;
    }

    fd_set readfds;

    FD_ZERO( &readfds );
    FD_SET( my_fd, &readfds );

    if( select( my_fd + 1, &readfds, NULL, NULL, &timeout ) > 0 ) 
    {
        return 1; 
    }
    else 
    {
        return 0;
    }
    return 0;
}
