/*
 *  ======= node_comm_task ========
 *  node_comm_task implementation
 *
 *  Created on: Apr 28, 2019
 *  Author:     rober
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>

#include "FreeRTOS.h"

#include "priorities.h"
#include "common.h"
#include "uart.h"
#include "communication_interface.h"
#include "nrf_module.h"
#include "logger_task.h"
#include "node_comm_rx_task.h"
#include "node_comm_task.h"


#define MY_STACK_SIZE   (256)
extern xQueueHandle g_pLoggerQueue;

xTaskHandle g_pNodeCommTaskHandle;
static log_msg_t node_comm_log;



void comm_task( void *params )
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(5000);
    BaseType_t xResult;
    uint32_t alert_type = 0;

    CREATE_NODE_MESSAGE( node_msg, BOARD_ID_TIVA, TIVA_MODULE_COMM, BBG_MODULE_LOGGER );
    node_msg.msg_id = NODE_MSG_ID_BOARD_TYPE;
    node_msg.data.filler = 1;
    strncpy( node_msg.message, "Hello!", sizeof(node_msg.message) );
    CALC_CHECKSUM( &node_msg );
    comm_send_uart( &node_msg );

    while( 1 )
    {
        xResult = xTaskNotifyWait( pdFALSE,
                                   ULONG_MAX,
                                   &alert_type,
                                   xMaxBlockTime );
        if( pdPASS == xResult )
        {
            if( alert_type & NODE_MSG_ID_GET_TEMPERATURE )
            {
                node_msg.msg_id = NODE_MSG_ID_SENSOR_STATUS;
                get_temperature( &node_msg.data.sensor_value );
                CALC_CHECKSUM( &node_msg );
                comm_send_uart( &node_msg );
                LOG_TASK_MSG( &node_comm_log, "TX TEMP" );
            }
            else if( alert_type & NODE_MSG_ID_GET_LUX )
            {
                node_msg.msg_id = NODE_MSG_ID_SENSOR_STATUS;
                get_lux( &node_msg.data.sensor_value );
                CALC_CHECKSUM( &node_msg );
                comm_send_uart( &node_msg );
                LOG_TASK_MSG( &node_comm_log, "TX LUX" );
            }
            else if( alert_type & NODE_MSG_ID_GET_CLIENT_BOARD_TYPE )
            {
                node_msg.msg_id = NODE_MSG_ID_GET_CLIENT_BOARD_TYPE;
                strncpy( node_msg.message, "TM4C1924XL", sizeof(node_msg.message) );
                CALC_CHECKSUM( &node_msg );
                comm_send_uart( &node_msg );
                LOG_TASK_MSG( &node_comm_log, "TX ALIVE" );
            }
        }
    }

}

uint8_t node_comm_task_init( void )
{
    configASSERT( loggerTaskInitDone == 1 );

    comm_init_uart();

    node_comm_log.tickcount = 0;
    node_comm_log.src = "COMM_TASK";
    node_comm_log.level = LOG_INFO;
    if( 0 != node_comm_rx_task_init() )
    {
        LOG_TASK_MSG( &node_comm_log, "NODE COMM RX TASK INIT\n" );
        return 1;
    }

    /* Create the task */
    if( xTaskCreate(comm_task, (const portCHAR *)"COMM_TASK", MY_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + PRIO_COMM_TASK, &g_pNodeCommTaskHandle ) != pdTRUE )
    {
        return 1;
    }

    LOG_TASK_MSG( &node_comm_log, "INITIALIZED" );
    return 0;
}
