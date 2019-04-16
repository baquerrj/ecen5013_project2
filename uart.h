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

#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*!
 * @brief UART Baud rates
 */
typedef enum
{
    BAUD_115200 = 115200,
    BAUD_38400  = 38400,
    BAUD_57200  = 57200,
    BAUD_9600   = 9600,

} uart_baud_e;

/*!
 * @brief UART access semaphore
 */
xSemaphoreHandle g_pUARTMutex;


/*!
 * @brief Thread-safe printf macro
 * reference: https://www.freertos.org/FreeRTOS_Support_Forum_Archive/October_2013/freertos_Printf_-like_task_97cc7906j.html
 */
#define printf( fmt, ... )      xSemaphoreTake( g_pUARTMutex, portMAX_DELAY ); UART0_printf(fmt, ##__VA_ARGS__); xSemaphoreGive( g_pUARTMutex )
#define puts( str )             xSemaphoreTake( g_pUARTMutex, portMAX_DELAY ); UART0_putstr( str ); xSemaphoreGive( g_pUARTMutex )

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
void UART0_config( uart_baud_e baud_rate, uint32_t clock_frequency );

/*!
 * @brief Print string to UART0
 *
 * @param str - string to print
 * @returns void
 */
void UART0_putstr(const char *str);

/*!
 * @brief Formats string and outputs to UART
 *
 * @param fmt - format string
 * @returns void
 */
void UART0_printf( const char *fmt, ... );

/*!
 * @brief Print single char to UART0
 *
 * @param ch - character to print
 * @returns void
 */
void UART0_putchar( char ch );

#endif /* UART_H_ */
