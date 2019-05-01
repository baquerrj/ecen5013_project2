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
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"
#include "timers.h"

#include "max7219.h"
#include "main.h"
#include "led_task.h"
#include "uart.h"
#include "logger_task.h"
#include "tmp102_task.h"

#define MY_STACK_SIZE 256

#define LED_QUEUE_ITEMSIZE   (sizeof(log_msg_t))
#define LED_QUEUE_LENGTH     (30)

#define LED_D1_PORT    GPIO_PORTN_BASE
#define LED_D2_PORT    LED_D1_PORT

#define LED_D1_PIN     GPIO_PIN_0
#define LED_D2_PIN     GPIO_PIN_1

extern xQueueHandle g_pLoggerQueue;
extern TimerHandle_t tmp102_timer_handle;
extern TimerHandle_t apds9301_timer_handle;

TimerHandle_t led_timer_handle;

xTaskHandle g_pLedTaskHandle;

void led_enqueue( QueueHandle_t queue, const log_msg_t *msg_out, size_t size )
{
    if( pdPASS != xQueueSend( queue, (void*)msg_out, portMAX_DELAY ) )
    {
        puts( "[LED] --- MESSAGE QUEUE\n" );
    }
}

static void init_10hz( void *params )
{

    led_timer_handle = xTimerCreate( "TOGGLE_TASK", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, led_task_callback );

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

    if( led_timer_handle == timer )
    {
        static uint32_t toggle_count = 0;
        toggle_count++;

        /* Build the message to send to logger */
        msg_out.tickcount = xTaskGetTickCount();
        msg_out.level = LOG_INFO;
        msg_out.data.toggle_count = toggle_count;
        static uint32_t led_d1 = LED_D1_PIN;
        static uint32_t led_d2 = LED_D2_PIN;

        led_d1 ^= (LED_D1_PIN);
        led_d2 ^= (LED_D2_PIN);

        GPIOPinWrite( LED_D1_PORT, LED_D1_PIN, led_d1 );
        GPIOPinWrite( LED_D2_PORT, LED_D2_PIN, led_d2 );

        LOG_TASK_MSG( &msg_out, "TOGGLE COUNT: %u", msg_out.data.toggle_count );
    }
}

void led_task( void *params )
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
    static log_msg_t msg_in;
    msg_in.src = pcTaskGetTaskName( g_pLedTaskHandle );
    msg_in.level = LOG_WARNING;
    while(1)
    {
        if(xQueueReceive( g_pLedQueue , &msg_in, xMaxBlockTime ) )
        {
            switch( msg_in.level )
            {
                case LOG_WARNING:
                {
                    if( msg_in.src == pcTimerGetTimerName( tmp102_timer_handle ) )
                    {
                        char buf[8];
                        snprintf( buf, 8 * sizeof( char ), "%f", msg_in.data.float_data );
                        maxSegmentString( buf );
                    }
                    else if( msg_in.src == pcTimerGetTimerName( apds9301_timer_handle ) )
                    {
                        char buf[8] = "DARK!";
                        maxSegmentString( buf );
                    }
                }
            }
        }
    }
}
uint8_t led_task_init( void )
{
    configASSERT(loggerTaskInitDone == 1);
    g_pLedQueue = xQueueCreate(LED_QUEUE_LENGTH, LED_QUEUE_ITEMSIZE);

    g_LedMutex = xSemaphoreCreateMutex();

    maxInit( 1, 1, MAX7219_SPI );

    /* Configure the GPIO pins*/
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPION );
    MAP_GPIOPinTypeGPIOOutput(LED_D2_PORT, LED_D2_PIN | LED_D1_PIN);

    if( pdTRUE != xTaskCreate( led_task, (const portCHAR *)"LED_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_LED_TASK, &g_pLedTaskHandle ) )
    {
        return 1;
    }

    if( pdTRUE != xTaskCreate( init_10hz, (const portCHAR *)"TOGGLE_TASK", MY_STACK_SIZE, NULL,
                                 tskIDLE_PRIORITY + PRIO_LED_TASK, NULL ) )
    {
        return 1;
    }

    log_msg_t led_msg;
    led_msg.tickcount = 0;
    led_msg.src = pcTaskGetTaskName( g_pLedTaskHandle );
    led_msg.level = LOG_INFO;
    LOG_TASK_MSG( &led_msg, "INITIALIZED" );

    return 0;
}


