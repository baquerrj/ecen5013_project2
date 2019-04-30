/*!
 * @file	logger_task.h
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef LOGGER_TASK_H_
#define LOGGER_TASK_H_

#include "uart.h"

#define INFO    "[INFO]"
#define ERROR   "[ERROR]"
#define WARNING "[WARNING]"

typedef enum
{
    MSG_BEGIN = 0,
    MSG_TOGGLE_LED,
    MSG_GET_TEMP,
    MSG_TEMP_LOW,
    MSG_TEMP_HIGH,
    MSG_GET_LUX,
    MSG_PROXIMITY,
    MSG_MAX
} log_msg_e;

typedef enum
{
    LOG_ERROR,
    LOG_INFO,
    LOG_WARNING,
    LOG_ALL
} log_level_e;


typedef struct
{
    TickType_t tickcount;
    log_level_e level;
    const char *src;
    char msg[25];
    union data
    {
        float float_data;
        uint32_t toggle_count;
    } data;
} log_msg_t;

extern xSemaphoreHandle g_pUARTMutex[ UART_MAX ];

#define LOG_INFO( p_log ) \
    do{ \
        printf( INFO "[%t][%s]: %s\n", (p_log)->tickcount, (p_log)->src, (p_log)->msg ); \
    }while(0)

#define LOG_ERROR( p_log ) \
    do{ \
        printf( ERROR "[%t][%s]: %s\n", (p_log)->tickcount, (p_log)->src, (p_log)->msg ); \
    }while(0)

#define LOG_WARNING( p_log ) \
    do{ \
        printf( WARNING "[%t][%s]: %s\n", (p_log)->tickcount, (p_log)->src, (p_log)->msg ); \
    }while(0)


xSemaphoreHandle g_LoggerMutex;

#define LOG_TASK_MSG( p_log, fmt, ... ) \
    do{ \
        xSemaphoreTake( g_LoggerMutex, portMAX_DELAY ); \
        snprintf( (p_log)->msg, sizeof( (p_log)->msg), fmt, ##__VA_ARGS__); \
        enqueue( g_pLoggerQueue, p_log, sizeof( p_log ) ); \
        xSemaphoreGive( g_LoggerMutex ); \
    }while(0)


/*!
 * @brief Add logging message to queue
 *
 * @param[in]   queue handle
 * @param[in]   message to queue
 * @param[in]   size of message
 */
void enqueue( QueueHandle_t queue, const log_msg_t *msg_out, size_t size );

/*!
 * @brief Logger Task function
 *
 * @param params - FreeRTOS TaskHandle_t params
 * @returns void
 */
void logger_task( void *params );

/*!
 * @brief Initialize logger task by calling FreeRTOS mechanisms
 *          xTaskCreate and xQueueCreate
 * @oaram void
 * @returns 0 if successful
 */
uint8_t logger_task_init( void );


#endif /* LOGGER_TASK_H_ */
