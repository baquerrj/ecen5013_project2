/*
 *  ======= node_comm_task ========
 *  node_comm_task implementation
 *
 *  Created on: Apr 28, 2019
 *  Author:     rober
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>

#include "FreeRTOS.h"

#include "priorities.h"
#include "uart.h"
#include "nrf_module.h"
#include "logger_task.h"
#include "node_comm_task.h"

#define MY_STACK_SIZE   (256)
extern xQueueHandle g_pLoggerQueue;

xTaskHandle g_pNodeCommTaskHandle;

volatile uint8_t count = 0;
//0x54,0x4d,0x52,0x68,0x7C
const uint64_t pipes[6] =
                    { 0xF0F0F0F0D2LL,
                      0xF0F0F0F0E1LL,
                      0xF0F0F0F0E2LL,
                      0xF0F0F0F0E3LL,
                      0xF0F0F0F0F1LL,
                      0xF0F0F0F0F2LL
                    };


typedef enum
{
    ROLE_SENDER = 0,
    ROLE_RECV,
    ROLE_MAX
} role_e;



int8_t comm_init_nrf( void )
{
    if( count )
    {
        count++;
        return 0;
    }

    //nrf_init_test();
    if( 0 != nrf_module_init() )
    {
        return -1;
    }

    nrf_set_channel(1);
    nrf_set_palevel( NRF_PA_MIN );
    nrf_open_writing_pipe( pipes[ 0 ] );
    nrf_open_reading_pipe( 1, pipes[ 1 ] );

    print_details();
    count++;

    //nrf_start_listening();
    return 0;
}

void comm_task( void *params )
{
//    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
//    BaseType_t xResult;
//    uint32_t alert_type = 0;
    static log_msg_t msg_out;
    msg_out.src = pcTaskGetTaskName( g_pNodeCommTaskHandle );
    msg_out.level = LOG_INFO;
    role_e role = ROLE_RECV;
    while( 1 )
    {
        if( ROLE_RECV == role )
        {
            printf( "WAITING TO RECEIVE\n" );
            nrf_start_listening();
            while( !nrf_available() )
            {
                delayMs( 5 );
            }
            msg_out.level = LOG_INFO;
            LOG_TASK_MSG( &msg_out, "NEW DATA AVAILABLE\n" );
            unsigned long data;
            nrf_read( (uint8_t*)&data, sizeof( unsigned long ) );
            LOG_TASK_MSG( &msg_out, "DATA: 0%x\n", data );
            role = ROLE_SENDER;
        }
        else if( ROLE_SENDER == role )
        {
            printf( "PREPARING TO SEND\n" );
            nrf_stop_listening();
            unsigned long tick = 0xDEADBEEF;
            printf( "Sending %u\n", tick );

            uint8_t ok = nrf_write( (uint8_t*)&tick, sizeof( unsigned long ) );
            msg_out.level = LOG_INFO;
            if( ok )
            {
                msg_out.level = LOG_INFO;
                //LOG_TASK_MSG( &msg_out, "TX SUCCESS\n" );
                printf( "TX SUCCESS\n" );
            }
            else
            {
                msg_out.level = LOG_ERROR;
                //LOG_TASK_MSG( &msg_out, "TX FAILED\n" );
                printf( "TX FAILED\n" );
            }
            role = ROLE_RECV;
        }
    }
}

uint8_t node_comm_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    if( 0 != comm_init_nrf() )
    {
        puts( "ERROR - NODE COMM TASK\n" );
        return 1;
    }

    /* Create the task */
    if( xTaskCreate(comm_task, (const portCHAR *)"COMM_TASK", MY_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + PRIO_COMM_TASK, &g_pNodeCommTaskHandle ) != pdTRUE )
    {
        return 1;
    }

    return 0;
}
