/*!
 * @file	apds9301_task.c
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
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
#include "logger_task.h"
#include "apds9301_sensor.h"
#include "apds9301_task.h"

#define MY_STACK_SIZE   (256)

extern xQueueHandle g_pLoggerQueue;

TimerHandle_t apds9301_timer_handle;


static void init_1hz( void *params )
{
    apds9301_timer_handle = xTimerCreate( "APDS9301_TASK", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, apds9301_task_callback );

    if( apds9301_timer_handle == NULL)
    {
        while(1);
    }

    if((xTimerStart(apds9301_timer_handle, 0)) != pdTRUE)
    {
        while(1);
    }

    /* The only work of this task was to initialize a timer and start it. We are suspending the task now*/
    vTaskSuspend(NULL);
}
void apds9301_task_callback( TimerHandle_t timer )
{
    static log_msg_t msg_out;
    msg_out.src = pcTimerGetTimerName( apds9301_timer_handle );
    float lux = -5.5;
    //Timer handle for 1Hz APDS9301 Sensor task
    if( apds9301_timer_handle == timer )
    {
        msg_out.tickcount = xTaskGetTickCount();
        msg_out.level = LOG_INFO;

        apds9301_get_lux( &lux );
        msg_out.data.float_data = lux;

        LOG_TASK_MSG( &msg_out, "LUX: %f", msg_out.data.float_data );

    }
    return;
}


uint8_t apds9301_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    apds9301_power_on( POWER_ON );

    uint8_t apds9301_id = apds9301_read_id();

    if( APDS9301_ID != apds9301_id )
    {
        printf("ERROR - ID: %x\n", apds9301_id );
        return 1;
    }
    else
    {
        printf("SUCCESS - ID: %x\n", apds9301_id );
    }

    if( pdTRUE != xTaskCreate( init_1hz, (const portCHAR *)"APDS9301_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_APDS9301_TASK, NULL ) )
    {
        return 1;
    }

    return 0;








}
