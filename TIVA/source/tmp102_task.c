/*
 * tmp102_task.c
 *
 *  Created on: Apr 8, 2019
 *      Author: rober
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"
#include "timers.h"

#include "uart.h"
#include "led_task.h"
#include "logger_task.h"
#include "tmp102_sensor.h"
#include "tmp102_task.h"
#include "alert_task.h"

#define MY_STACK_SIZE 256

extern xQueueHandle g_pLoggerQueue;

extern xTaskHandle g_pAlertTaskHandle;

TimerHandle_t tmp102_timer_handle;
static log_msg_t log_msg_out;

static float temp = -5.5;

void get_temperature( float *data )
{
    *data = temp;
}

static void init_1hz( void *params )
{

    tmp102_timer_handle = xTimerCreate( "TMP102_TASK", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, tmp102_task_callback );

    if( tmp102_timer_handle == NULL)
    {
        while(1);
    }

    if((xTimerStart(tmp102_timer_handle, 0)) != pdTRUE)
    {
        while(1);
    }

    /* The only work of this task was to initialize a timer and start it. We are suspending the task now*/
    vTaskSuspend(NULL);
}

void tmp102_task_callback( TimerHandle_t timer )
{
    log_msg_out.src = pcTimerGetTimerName( tmp102_timer_handle );
    //Timer handle for 1Hz Temperature Sensor task
    if( tmp102_timer_handle == timer )
    {
        log_msg_out.tickcount = xTaskGetTickCount();
        log_msg_out.level = LOG_INFO;

        tmp102_get_temp( &temp );
        log_msg_out.data.float_data = temp;

        int i = (int32_t)temp;
        if( HIGH_TEMPERATURE < i )
        {
            /* Notify Alert Task of out-of-range temperature */
            xTaskNotify( g_pAlertTaskHandle, MSG_TEMP_HIGH, eSetBits );
            SENSOR_MSG( &log_msg_out, "HIGH TEMP!" );
        }
        else if( LOW_TEMPERATURE > i )
        {
            /* Notify Alert Task of out-of-range temperature */
            xTaskNotify( g_pAlertTaskHandle, MSG_TEMP_LOW, eSetBits );
            SENSOR_MSG( &log_msg_out, "LOW TEMP!" );
        }

        LOG_TASK_MSG( &log_msg_out, "TEMP: %f C", log_msg_out.data.float_data );
    }
}

uint8_t temp_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    if( pdTRUE != xTaskCreate( init_1hz, (const portCHAR *)"TMP102_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_TMP_TASK, NULL ) )
    {
        return 1;
    }

    return 0;
}
