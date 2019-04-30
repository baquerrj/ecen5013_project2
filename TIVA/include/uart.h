/*!
 * @file	uart.h
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef UART_H_
#define UART_H_

#include "FreeRTOS.h"
#include "semphr.h"

/*!
 * @brief UART Baud rates
 */
typedef enum
{
    BAUD_921600 = 921600,
    BAUD_115200 = 115200,
    BAUD_38400  = 38400,
    BAUD_57200  = 57200,
    BAUD_9600   = 9600
} uart_baud_e;

/*!
 * @brief UART enum
 */
typedef enum
{
    UART_0,
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6,
    UART_MAX
} uart_e;

/*!
 * @brief UART access semaphore
 */
xSemaphoreHandle g_pUARTMutex[ UART_MAX ];


/*!
 * @brief Thread-safe printf macro
 * reference: https://www.freertos.org/FreeRTOS_Support_Forum_Archive/October_2013/freertos_Printf_-like_task_97cc7906j.html
 */
#define printf( fmt, ... )      xSemaphoreTake( g_pUARTMutex[ UART_0 ], portMAX_DELAY ); \
                                uart_printf( UART_0, fmt, ##__VA_ARGS__); \
                                xSemaphoreGive( g_pUARTMutex[ UART_0 ] )
#define puts( str )             xSemaphoreTake( g_pUARTMutex[ UART_0 ], portMAX_DELAY ); \
                                uart_putstr( UART_0, str ); \
                                xSemaphoreGive( g_pUARTMutex[ UART_0 ] )

/*!
 * @brief Put data on UART
 */
void uart_putraw( uart_e, uint8_t* buf, size_t len );

/*!
 * @Brief Get raw data from UART
 */
uint8_t uart_getraw( uart_e uart, uint8_t *buf, size_t len );

/*!
 * @brief Convert value to ASCII
 * reference: uartstio.c
 *
 * @param num - integer value to convert
 * @param base - octal, hex, decimal
 * @returns ASCII representation of num
 */
char* convert( unsigned int num, int base );

/*!
 * @brief Configure UART0 with baud_rate
 *
 * @param baud_rate - Baud Rate to set UART0 to
 * @param clock_frequency - system clock
 * @returns void
 */
void uart_config( uart_e uart, uart_baud_e baud_rate );

/*!
 * @brief Print string to UART0
 *
 * @param str - string to print
 * @returns void
 */
void uart_putstr( uart_e uart, const char *str);

/*!
 * @brief Formats string and outputs to UART
 *
 * @param fmt - format string
 * @returns void
 */
void uart_printf( uart_e uart, const char *fmt, ... );

/*!
 * @brief Print single char to UART0
 *
 * @param ch - character to print
 * @returns void
 */
void uart_putchar( uart_e uart, char ch );

#endif /* UART_H_ */
