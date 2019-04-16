/*!
 * @file	led_task.c
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
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

#include "main.h"
#include "led_task.h"
#include "uart.h"
#include "logger_task.h"
#include "tmp102_task.h"

#define MY_STACK_SIZE 256

#define LED_D1_PORT    GPIO_PORTN_BASE
#define LED_D2_PORT    LED_D1_PORT

#define LED_D1_PIN     GPIO_PIN_0
#define LED_D2_PIN     GPIO_PIN_1

extern xQueueHandle g_pLoggerQueue;

TimerHandle_t led_timer_handle;

static void init_10hz( void *params )
{
    led_timer_handle = xTimerCreate( "LED_TASK", pdMS_TO_TICKS(100), pdTRUE, (void*)0, led_task_callback );

    if( led_timer_handle == NULL)
    {
        while(1);
    }

    if((xTimerStart(led_timer_handle, 0)) != pdTRUE)
    {
        while(1);
    }

    /* The only work of this task was to initialize a timer and start it. We are suspending the task now*/
    vTaskSuspend(NULL);
}

void led_task_callback( TimerHandle_t timer )
{
    static log_msg_t msg_out;
    msg_out.src = pcTimerGetTimerName( led_timer_handle );
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);

    if( led_timer_handle == timer )
    {
        static uint32_t toggle_count = 0;
        toggle_count++;

        /* Build the message to send to logger */
        memcpy( msg_out.msg, "TOGGLE_LED", sizeof( msg_out.msg ) );
        msg_out.tickcount = xTaskGetTickCount();
        msg_out.type = MSG_TOGGLE_LED;
        msg_out.data.toggle_count = toggle_count;
        static uint32_t led_d1 = LED_D1_PIN;
        static uint32_t led_d2 = LED_D2_PIN;

        led_d1 ^= (LED_D1_PIN);
        led_d2 ^= (LED_D2_PIN);

        GPIOPinWrite( LED_D1_PORT, LED_D1_PIN, led_d1 );
        GPIOPinWrite( LED_D2_PORT, LED_D2_PIN, led_d2 );

        //Enqueue the worker queue with a new msg
        if( xQueueSend( g_pLoggerQueue, &msg_out, xMaxBlockTime ) != pdPASS )
        {
            puts("ERROR - LED TASK - QUEUE SEND\n");
        }
    }
}

uint8_t led_task_init( void )
{
    configASSERT(loggerTaskInitDone == 1);

    /* Configure the GPIO pins*/
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPION );
    MAP_GPIOPinTypeGPIOOutput(LED_D2_PORT, LED_D2_PIN | LED_D1_PIN);

    if( pdTRUE != xTaskCreate( init_10hz, (const portCHAR *)"LED_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_LED_TASK, NULL ) )
    {
        return 1;
    }

    return 0;
}


