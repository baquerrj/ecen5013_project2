/*!
 * @file  my_uart.h
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


#ifndef  _MY_UART_H_
#define  _MY_UART_H_

#include <stdlib.h>


/*! Communication Ports */
typedef enum
{
    COM_PORT_1 = 1,
    COM_PORT_2,
    COM_PORT_3,
    COM_PORT_4
} port_e;


/*!
 * @brief   Initialize UART on COM PORT
 */
int uart_open( port_e com_port );


/*!
 * @brief   Close UART on COM PORT
 */
void uart_close( int uart );


/*!
 * @brief   Flush UART
 */
void uart_flush( void );


/*!
 * @brief   Put data on UART
 */
int32_t uart_putraw( void *p, size_t len );


/*!
 * @brief Put char
 */
int32_t uart_putchar( char c );


/*!
 * @brief   Put string
 */
int32_t uart_putstr( const char *str );


/*!
 * @brief   Read from UART
 *
 * @param[in]   p Where to store data
 * @param[in]   len Number of bytes to read
 * @returns number of bytes read
 */
int32_t uart_read( void *p, size_t len );

/*!
 * @brief   Check if data is available on UART
 */
int32_t uart_new_data( uint32_t ms );






#endif   /* _MY_UART_H_ */
