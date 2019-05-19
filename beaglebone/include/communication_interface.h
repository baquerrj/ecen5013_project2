/*!
 * @file  communication_interface.h
 * @brief
 *
 *  <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/27/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


#ifndef  _COMMUNICATION_INTERFACE_H_
#define  _COMMUNICATION_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>
#include "common.h"
#include "uart.h"
//#define TIVA_BOARD

#ifdef TIVA_BOARD
#define TIVA_UART   UART_6
#endif

volatile uint8_t comm_type;

#ifdef TIVA_BOARD
static inline void comm_init_uart( void )
{
    uart_config( TIVA_UART, BAUD_921600 );
}

static inline void comm_deinit_uart( int fd ) {}


static inline void comm_send_uart_raw( uint8_t* packet, size_t len )
{
    uart_putraw( TIVA_UART, packet, len );
}

static inline void comm_send_uart( node_message_t *p_comm_object )
{
    uart_putraw( TIVA_UART, (uint8_t*)p_comm_object, sizeof( node_message_t ) );
}

static inline size_t comm_recv_uart( node_message_t *p_comm_object )
{
    return uart_getraw( TIVA_UART, (uint8_t*)p_comm_object, sizeof( node_message_t ) );
}

#else

static inline int comm_init_uart()
{
    return uart_open( COM_PORT_4 );
}

static inline void comm_deinit_uart( int fd )
{
    uart_close( fd );
}

static inline int32_t comm_send_uart( node_message_t *p_comm_object )
{
    return uart_putraw( (void*)p_comm_object,sizeof( node_message_t ) );
}
static inline int32_t comm_send_uart_raw( node_message_t * comm_object, size_t len )
{
    return uart_putraw( (void*)comm_object,len );
}

static inline int32_t comm_recv_uart( node_message_t *comm_object )
{
    int32_t available = uart_new_data( 100 );
    if( available == 1 )
    {
        return uart_read( (void*)comm_object,sizeof( node_message_t ) );
    }
    else
    {
        return available;
    }
}

#endif

#endif   /* _COMMUNICATION_INTERFACE_H_ */
