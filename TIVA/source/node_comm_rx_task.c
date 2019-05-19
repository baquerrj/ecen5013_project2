/*
 *  ======= node_comm_rx_task ========
 *  node_comm_rx_task implementation
 *
 *  Created on: Apr 30, 2019
 *  Author:     rober
 */

#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"

#include "priorities.h"

#include "uart.h"
#include "common.h"
#include "logger_task.h"
#include "communication_interface.h"
#include "node_comm_task.h"
#include "node_comm_rx_task.h"
#include "delay.h"
#define MY_STACK_SIZE   (256)

extern xQueueHandle g_pLoggerQueue;
extern xTaskHandle g_pNodeCommTaskHandle;

TaskHandle_t g_pNodeRxTaskHandle;

/* Create the entry task*/
static void node_receiver_task(void *params)
{
    static node_message_t node_msg_in;
    while(1)
    {
        memset(&node_msg_in, 0 , sizeof(node_msg_in));
        int32_t ret = comm_recv_uart(&node_msg_in);
        if(0 > ret)
        {
            if(node_msg_in.src_brd_id == BOARD_ID_BBG )
            {
                printf("\n*******\n\
                SRCID:%u, SRC_BRDID:%u, DST_ID:%u, MSGID:%u\n\
                MSG:%s\n\
                Checksum:%u ?= %u\n********\n",\
                node_msg_in.src_id, node_msg_in.src_brd_id, node_msg_in.dst_id,node_msg_in.msg_id, node_msg_in.message,node_msg_in.checksum, getCheckSum(&node_msg_in));
                switch( node_msg_in.msg_id )
                {
                    case NODE_MSG_ID_ALIVE:
                    {
                        NOTIFY_COMM_TASK( NODE_MSG_ID_ALIVE );
                        break;
                    }
                    case NODE_MSG_ID_GET_TEMPERATURE:
                    {
                        NOTIFY_COMM_TASK( NODE_MSG_ID_GET_TEMPERATURE );
                        break;
                    }
                    case NODE_MSG_ID_GET_LUX:
                    {
                        NOTIFY_COMM_TASK( NODE_MSG_ID_GET_LUX );
                        break;
                    }
                    case NODE_MSG_ID_GET_SENSOR_INFO:
                    {
                        break;
                    }
                    case NODE_MSG_ID_GET_CLIENT_BOARD_TYPE:
                    {
                        NOTIFY_COMM_TASK( NODE_MSG_ID_GET_CLIENT_BOARD_TYPE );
                        break;
                    }
                    case NODE_MSG_ID_GET_CLIENT_UID:
                    {
                        break;
                    }
                    default:
                        break;
                }
            }
        }
    }
}


uint8_t node_comm_rx_task_init()
{

    /* Create the task*/
    if(xTaskCreate(node_receiver_task, (const portCHAR *)"NODE_RX_TASK", MY_STACK_SIZE, NULL,
                       tskIDLE_PRIORITY + PRIO_COMM_TASK, &g_pNodeRxTaskHandle) != pdTRUE)
    {
        return 1;
    }


    return 0;
}
