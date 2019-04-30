/*!
 * @file	logger_task.c
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"

#include "tmp102_task.h"
#include "logger_task.h"

#define LOGGER_QUEUE_ITEMSIZE   (sizeof(log_msg_t))
#define LOGGER_QUEUE_LENGTH     (30)

#define MY_STACK_SIZE 256

volatile uint8_t loggerTaskInitDone = 0;

xQueueHandle g_pLoggerQueue;

xTaskHandle g_pLoggerTask;

void enqueue( QueueHandle_t queue, const log_msg_t *msg_out, size_t size )
{
    if( pdPASS != xQueueSend( queue, (void*)msg_out, portMAX_DELAY ) )
    {
        puts( "[LOGGER] --- MESSAGE QUEUE\n" );
    }
}


void logger_task( void *params )
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    log_msg_t msg_in;

    //LOG_TASK_MESSAGE
    while(1)
    {
        if(xQueueReceive( g_pLoggerQueue , &msg_in, xMaxBlockTime ))
        {
            switch( msg_in.level )
            {
                case LOG_ERROR:
                {
                    LOG_ERROR( &msg_in );
                    break;
                }
                case LOG_INFO:
                {
                    LOG_INFO( &msg_in );
                    break;
                }
                case LOG_WARNING:
                {
                    LOG_WARNING( &msg_in );
                    break;
                }
            }
        }
        else
        {
            puts( "ERROR - LOGGER TASK - QUEUE RECV\n" );
        }
    }
}

uint8_t logger_task_init( void )
{
    /* Creating a Queue required for Logging the msg */
    g_pLoggerQueue = xQueueCreate(LOGGER_QUEUE_LENGTH, LOGGER_QUEUE_ITEMSIZE);

    g_LoggerMutex = xSemaphoreCreateMutex();

    /* Create the task*/
    if(xTaskCreate(logger_task, (const portCHAR *)"LOGGER", MY_STACK_SIZE, NULL,
                       tskIDLE_PRIORITY + PRIO_LOG_TASK, &g_pLoggerTask ) != pdTRUE)
    {
        return 1;
    }
    log_msg_t logger_msg;
    logger_msg.tickcount = 0;
    logger_msg.src = pcTaskGetTaskName( g_pLoggerTask );
    logger_msg.level = LOG_INFO;
    LOG_TASK_MSG( &logger_msg, "INITIALIZED" );

    loggerTaskInitDone = 1;

    return 0;
}
