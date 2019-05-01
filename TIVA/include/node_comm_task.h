/*!
 * @file	node_comm_task.h
 *
 * @brief
 *
 *  Created on: Apr 28, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef _NODE_COMM_TASK_H_
#define _NODE_COMM_TASK_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "tmp102_task.h"
#include "apds9301_task.h"
#include "uart.h"
#include "communication_interface.h"

//Handy macros
#define NOTIFY_COMM_TASK( MSG_ID )       xTaskNotify(g_pNodeCommTaskHandle, MSG_ID, eSetBits)

#define CREATE_NODE_MESSAGE(name, src_board_id, source_id, dest_id) \
    node_message_t name = { \
                        .src_brd_id = src_board_id, \
                        .src_id = source_id, \
                        .dst_id = dest_id, \
                        .dst_brd_id = BBG_BOARD_ID }

#define NODE_MSG_ID(node_msg,msgid)   node_msg.msg_id = msgid
#define CALC_CHECKSUM(p_node_msg)           do{ (p_node_msg)->checksum = getCheckSum(p_node_msg); }while(0)
#define NDOE_MSG(node_msg,p_str)       strncpy(node_msg.message,p_str,sizeof(node_msg.message))




uint8_t node_comm_task_init( void );


#endif /* _NODE_COMM_TASK_H_ */
