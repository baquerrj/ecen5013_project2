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

#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"

#include "tmp102_task.h"
#include "logger_task.h"
#include "uart.h"


#define LOGGER_QUEUE_ITEMSIZE   (sizeof(log_msg_t))
#define LOGGER_QUEUE_LENGTH     (30)

#define MY_STACK_SIZE 256

volatile uint8_t loggerTaskInitDone = 0;

xQueueHandle g_pLoggerQueue;

void logger_task( void *params )
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    log_msg_t msg_in;
    while(1)
    {
        if(xQueueReceive( g_pLoggerQueue , &msg_in, xMaxBlockTime ))
        {
            switch( msg_in.type )
            {
                case MSG_GET_TEMP:
                {
                    printf( "(%s): %s ----- TIME: %t ----- TEMP: %f C\n",
                            msg_in.src, msg_in.msg, msg_in.tickcount, msg_in.data.temperature );
                    break;
                }
                case MSG_TOGGLE_LED:
                {
                    printf( "(%s): %s ----- TIME: %t ----- TOGGLE COUNT: %u\n",
                            msg_in.src, msg_in.msg, msg_in.tickcount, msg_in.data.toggle_count );
                    break;
                }
                case MSG_TEMP_LOW:
                case MSG_TEMP_HIGH:
                {
                    printf( "(%s): %s ----- TIME: %t\n",
                            msg_in.src, msg_in.msg, msg_in.tickcount );
                    break;
                }
                default:
                    break;
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

    /* Create the task*/
    if(xTaskCreate(logger_task, (const portCHAR *)"LoggerTask", MY_STACK_SIZE, NULL,
                       tskIDLE_PRIORITY + PRIO_LOG_TASK, NULL ) != pdTRUE)
    {
        return 1;
    }

    loggerTaskInitDone = 1;

    return 0;
}
