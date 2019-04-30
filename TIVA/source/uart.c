/*!
 * @file	uart.c
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */

#include "uart.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"


#define UART_CONFIG_NORMAL  ( UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE )
#define MAX_RETRY 64

extern uint32_t g_sysClock;

const uint32_t UART[ UART_MAX ] = { UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE, UART5_BASE, UART6_BASE };


void uart_config( uart_e uart, uart_baud_e baud_rate )
{
    switch( uart )
    {
        case UART_0:
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

            UARTConfigSetExpClk( UART0_BASE, g_sysClock, baud_rate, UART_CONFIG_NORMAL );

            g_pUARTMutex[ UART_0 ] = xSemaphoreCreateMutex();
            UARTEnable( UART0_BASE );
            break;
        }
        case UART_6:
        {
            // Enable the GPIO Peripheral used by the UART.
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
            // Enable UART3
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
            while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));

            // Configure GPIO Pins for UART mode.
            MAP_GPIOPinConfigure(GPIO_PP0_U6RX);
            MAP_GPIOPinConfigure(GPIO_PP1_U6TX);
            MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);
            UARTConfigSetExpClk( UART6_BASE, g_sysClock, baud_rate, UART_CONFIG_NORMAL );

            g_pUARTMutex[ UART_6 ] = xSemaphoreCreateMutex();
            UARTEnable( UART6_BASE );
            break;
        }
        default:
            /* unsupported */
            break;
    }

}

void uart_putraw( uart_e uart, uint8_t *buf, size_t len )
{
    while( UARTBusy( UART[ uart ] ) );
    while( len-- )
    {
        UARTCharPut( UART[ uart ], *buf++ );
    }
}


uint8_t uart_getraw( uart_e uart, uint8_t *buf, size_t len )
{
    if( !UARTCharsAvail( UART[ uart ] ) )
    {
        return 0;
    }
    uint8_t i = 0;
    uint8_t attempts = 0;
    while( (i<len) && (attempts < MAX_RETRY) )
    {
        int32_t c = UARTCharGetNonBlocking( UART[ uart ] );
        if( c != -1)
        {
            *(buf+i) = c;
            attempts = 0;
            i++;
        }
        else
        {
            attempts++;
        }
    }
    return i;
}


void uart_putstr( uart_e uart, const char *str )
{
    while( *str )
    {
        if( *str == '\n' )
            UARTCharPut( UART[ uart ], '\r' );
        UARTCharPut( UART[ uart ], *str++ );
    }
}

void uart_printf( uart_e uart, const char *fmt, ... )
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
            uart_putchar( uart,  *p );
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
                    uart_putchar( uart,  '-' );
                }
                i = (int32_t)d;
                uart_putstr( uart, convert( i, 10 ) );
                uart_putchar( uart,  '.' );
                i = (int32_t)((d -(int32_t)d )*1000);
                uart_putstr( uart, convert( i, 10 ) );
                break;
            }
            case 'c' :
            {
                i = va_arg( argp, int );
                uart_putchar( uart,  i );
                break;
            }
            case 'd' :
            {
                i = va_arg( argp, int );
                if( i < 0 )
                {
                    i = -i;
                    uart_putchar( uart,  '-' );
                }
                uart_putstr( uart, convert( i, 10 ) );
                break;
            }
            case 'o':
            {
                i = va_arg( argp, unsigned int );
                uart_putstr( uart, convert( i, 8 ) );
                break;
            }
            case 's':
            {
                s = va_arg( argp, char * );
                uart_putstr( uart, ( s ) );
                break;
            }
            case 'u':
            {
                u = va_arg( argp, unsigned int );
                uart_putstr( uart, convert( u, 10 ) );
                break;
            }
            case 'x':
            {
                u = va_arg( argp,unsigned int );
                uart_putstr( uart, convert( u, 16 ) );
                break;
            }
            case 't':
            {
                u = va_arg( argp, unsigned int );
                uint32_t sec  = (uint32_t)(u / 1000);
                if( 0 != sec )
                {
                    uint32_t msec = (uint32_t)(u % 1000);
                    uart_putstr( uart, convert( sec, 10 ) );
                    uart_putchar( uart,  '.' );
                    uart_putstr( uart, convert( msec, 10 ) );
                }
                else
                {
                    uart_putchar( uart,  '.' );
                    uart_putstr( uart, convert( u, 10 ) );
                }
                break;
            }
            case '%':
            {
                uart_putchar( uart,  '%' );
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


void uart_putchar( uart_e uart,  char ch )
{
    if( '\n' == ch )
    {
        UARTCharPut( UART[ uart ], '\r' );
    }
    UARTCharPut( UART[ uart ], ch );
    return;
}




