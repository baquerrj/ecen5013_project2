/*!
 * @file	uart.c
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */


#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"

#include "uart.h"

#define UART_CONFIG_NORMAL  ( UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE )

void UART0_config( uart_baud_e baud_rate, uint32_t clock_frequency )
{

    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
    // Enable UART0
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    while( !MAP_SysCtlPeripheralReady( SYSCTL_PERIPH_UART0 ) );

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure( GPIO_PA0_U0RX );
    MAP_GPIOPinConfigure( GPIO_PA1_U0TX );
    MAP_GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    UARTConfigSetExpClk( UART0_BASE, clock_frequency, baud_rate, UART_CONFIG_NORMAL );

    g_pUARTMutex = xSemaphoreCreateMutex();
    UARTEnable( UART0_BASE );
}


void UART0_putstr( const char *str )
{
    while( *str )
    {
        if( *str == '\n' )
            UARTCharPut( UART0_BASE, '\r' );
        UARTCharPut( UART0_BASE, *str++ );
    }
}


void UART0_printf( const char *fmt, ... )
{
    const char *p;
    int i;
    unsigned int u;
    char *s;
    double d;
    va_list argp;
    va_start( argp, fmt );

    p=fmt;
    for( p=fmt; *p!='\0';p++ )
    {
        if( *p != '%' )
        {
            UART0_putchar( *p );
            continue;
        }

        p++;

        switch( *p )
        {
            case 'f' :
            {
                d = va_arg( argp, double );
                if( d < 0 )
                {
                    d = -d;
                    UART0_putchar( '-' );
                }
                i = (int32_t)d;
                UART0_putstr( convert( i, 10 ) );
                UART0_putchar( '.' );
                i = (int32_t)((d -(int32_t)d )*1000);
                UART0_putstr( convert( i, 10 ) );
                break;
            }
            case 'c' :
            {
                i = va_arg( argp, int );
                UART0_putchar( i );
                break;
            }
            case 'd' :
            {
                i = va_arg( argp, int );
                if( i < 0 )
                {
                    i = -i;
                    UART0_putchar( '-' );
                }
                UART0_putstr( convert( i, 10 ) );
                break;
            }
            case 'o':
            {
                i = va_arg( argp, unsigned int );
                UART0_putstr( convert( i, 8 ) );
                break;
            }
            case 's':
            {
                s = va_arg( argp, char * );
                UART0_putstr( s );
                break;
            }
            case 'u':
            {
                u = va_arg( argp, unsigned int );
                UART0_putstr( convert( u, 10 ) );
                break;
            }
            case 'x':
            {
                u = va_arg( argp,unsigned int );
                UART0_putstr( convert( u, 16 ) );
                break;
            }
            case 't':
            {
                u = va_arg( argp, unsigned int );
                uint32_t sec  = (uint32_t)(u / 1000);
                if( 0 != sec )
                {
                    uint32_t msec = (uint32_t)(u % 1000);
                    UART0_putstr( convert( sec, 10 ) );
                    UART0_putchar( '.' );
                    UART0_putstr( convert( msec, 10 ) );
                }
                else
                {
                    UART0_putchar( '.' );
                    UART0_putstr( convert( u, 10 ) );
                }
                break;
            }
            case '%':
            {
                UART0_putchar( '%' );
                break;
            }
        }
    }

    va_end(argp);
}


char* convert( unsigned int num, int base )
{
    static char buf[ 50 ];
    char *ptr;

    ptr=&buf[ sizeof(buf)-1 ];
    *ptr='\0';
    do
    {
        *--ptr="0123456789abcdef"[ num%base ];
        num/=base;
    }while( num!=0 );
    return ( ptr );
}


void UART0_putchar( char ch )
{
    if( '\n' == ch )
    {
        UARTCharPut( UART0_BASE, '\r' );
    }
    UARTCharPut( UART0_BASE, ch );
    return;
}




