/*
 *  ======= apds9960_task ========
 *  apds9960_task implementation
 *
 *  Created on: Apr 16, 2019
 *  Author:     rober
 */

#include <stdint.h>
#include <stdbool.h>

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
#include "apds9960_sensor.h"
#include "apds9960_task.h"

#define MY_STACK_SIZE   (512)



extern xQueueHandle g_pLoggerQueue;

TimerHandle_t apds9960_timer_handle;


static void init_1hz( void *params )
{
    apds9960_timer_handle = xTimerCreate( "APDS9960_TASK", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, apds9960_task_callback );

    if( apds9960_timer_handle == NULL)
    {
        while(1);
    }

    if((xTimerStart(apds9960_timer_handle, 0)) != pdTRUE)
    {
        while(1);
    }

    /* The only work of this task was to initialize a timer and start it. We are suspending the task now*/
    vTaskSuspend(NULL);
}
void apds9960_task_callback( TimerHandle_t timer )
{
    static log_msg_t msg_out;
    msg_out.src = pcTimerGetTimerName( apds9960_timer_handle );
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    //Timer handle for 1Hz APDS9960 Sensor task
    if( apds9960_timer_handle == timer )
    {
        msg_out.tickcount = xTaskGetTickCount();
        msg_out.type = MSG_PROXIMITY;
        memcpy( msg_out.msg, "PROXIMITY", sizeof( msg_out.msg ) );

        uint8_t proximity = readProximity();
        printf( "proximity = %u\n", proximity );
        msg_out.data.float_data = (float)proximity;

        //Enqueue the worker queue with a new msg
        if( xQueueSend( g_pLoggerQueue, &msg_out, xMaxBlockTime ) != pdPASS )
        {
            puts("ERROR - APDS9960 SENSOR TASK - QUEUE SEND\n");
        }
    }
    return;
}


uint8_t apds9960_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    uint8_t retVal = apds9960_sensor_init();
    if( 0 != retVal )
    {
        return 1;
    }

    if( pdTRUE != xTaskCreate( init_1hz, (const portCHAR *)"APDS9960_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_APDS9960_TASK, NULL ) )
    {
        return 1;
    }

    return 0;








}
