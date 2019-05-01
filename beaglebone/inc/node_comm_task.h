/*!
 * @file  node_comm_task.h
 * @brief
 *
 *  <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/27/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


#ifndef  _NODE_COMM_TASK_H_
#define  _NODE_COMM_TASK_H_

#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <mqueue.h>

#include "common.h"
#include "communication_interface.h"

#define CREATE_NODE_MESSAGE( name, src_board_id, source_id, dest_id ) \
        node_message_t name = { \
            .src_brd_id = src_board_id, \
            .src_id = source_id, \
            .dst_id = dest_id, \
            .dst_brd_id = TIVA_BOARD_ID }

#define CALC_CHECKSUM( p_node_msg )           do{ ( p_node_msg )->checksum = getCheckSum( p_node_msg ); }while( 0 )
#define NODE_MSG( node_msg, p_str )       strncpy( node_msg.message, p_str, sizeof( node_msg.message ) )

/*!
 * @brief   Get message queue FD for communication task
 *
 * @return mqd_t
 */
mqd_t get_node_comm_queue( void );


/*!
 * @brief
 *
 */
static inline void send_get_client_board_info( void )
{
    CREATE_NODE_MESSAGE( node_msg, BOARD_ID_BBG, BBG_MODULE_COMM, TIVA_MODULE_COMM );
    node_msg.msg_id = NODE_MSG_ID_GET_CLIENT_BOARD_TYPE;
    CALC_CHECKSUM( &node_msg );
    comm_send_uart( &node_msg );
}

/*!
 * @brief
 *
 */
static inline void send_get_lux( void )
{
    CREATE_NODE_MESSAGE( node_msg, BOARD_ID_BBG, BBG_MODULE_COMM, TIVA_MODULE_APDS9301 );
    node_msg.msg_id = NODE_MSG_ID_GET_LUX;
    CALC_CHECKSUM( &node_msg );
    comm_send_uart( &node_msg );
}



/*!
 * @brief
 *
 * @param board_id
 */
static inline void send_get_temperature( void )
{
    CREATE_NODE_MESSAGE( node_msg, BOARD_ID_BBG, BBG_MODULE_COMM, TIVA_MODULE_TMP102 );
    node_msg.msg_id = NODE_MSG_ID_GET_TEMPERATURE;
    CALC_CHECKSUM( &node_msg );
    comm_send_uart( &node_msg );
}



/*!
 * @brief
 *
 */
mqd_t node_comm_task_queue_init( void );



/*!
 * @brief
 *
 * @param thread_args
 * @return void*
 */
void* node_comm_task_fn( void *thread_args );


#endif   /* _NODE_COMM_TASK_H_ */
