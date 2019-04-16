/*!
 * @file	alert_task.c
 *
 * @brief
 *
 *  Created on: Apr 10, 2019
 *      Author: Roberto Baquerizo
 */


#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "FreeRTOS.h"

#include "priorities.h"
#include "uart.h"
#include "alert_task.h"
#include "tmp102_task.h"
#include "logger_task.h"

#define MY_STACK_SIZE   (256)
extern xQueueHandle g_pLoggerQueue;

xTaskHandle g_pAlertTaskHandle;

void alert_task( void *params )
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    BaseType_t xResult;
    uint32_t alert_type = 0;
    static log_msg_t msg_out;
    msg_out.src = pcTaskGetTaskName( g_pAlertTaskHandle );
    while(1)
    {
        /* Wait to be notified of an interrupt */
        xResult = xTaskNotifyWait( pdFALSE,
                                   ULONG_MAX,
                                   &alert_type,
                                   xMaxBlockTime );
        /* We got a notification */
        if( pdPASS == xResult )
        {
            if( alert_type & MSG_TEMP_HIGH )
            {
                msg_out.tickcount = xTaskGetTickCount();
                msg_out.type = MSG_TEMP_HIGH;
                memcpy( msg_out.msg, "HIGH TEMP ALERT!", sizeof( msg_out.msg ) );
            }
            else if( alert_type & MSG_TEMP_LOW )
            {
                msg_out.tickcount = xTaskGetTickCount();
                msg_out.type = MSG_TEMP_LOW;
                memcpy( msg_out.msg, "LOW TEMP ALERT!", sizeof( msg_out.msg ) );
            }
            else
            {
                puts( "ERROR - ALERT TASK - INVALID ALERT\n" );
                return;
            }
            if( pdPASS != xQueueSend( g_pLoggerQueue, &msg_out, xMaxBlockTime ) )
            {
                puts( "ERROR - ALERT TASK - QUEUE SEND\n" );
            }
        }
    }
}

uint8_t alert_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    /* Create the task */
    if( xTaskCreate(alert_task, (const portCHAR *)"AlertTask", MY_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + PRIO_ALERT_TASK, &g_pAlertTaskHandle ) != pdTRUE )
    {
        return 1;
    }

    return 0;
}
