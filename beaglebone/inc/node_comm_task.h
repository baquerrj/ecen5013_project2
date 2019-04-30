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

#define COMM_CREATE_OBJECT( name, src_board_id, source_id, dest_id )  node_message_t name = { .src_brd_id = src_board_id, .src_id = source_id, .dst_id = dest_id, .dst_brd_id = TIVA_BOARD_ID }
#define COMM_OBJECT_MSGID( node_msg, msgid )   node_msg.msg_id = msgid
#define COMM_DST_BOARD_ID( node_msg, dst_board_id )    node_msg.dst_brd_id = dst_board_id
#define FILL_CHECKSUM( p_node_msg )           do{ ( p_node_msg )->checksum = getCheckSum( p_node_msg ); }while( 0 )
#define COMM_FILL_MSG( node_msg, p_str )       strncpy( node_msg.message, p_str, sizeof( node_msg.message ) )

/*!
 * @brief   Get message queue FD for communication task
 *
 * @return mqd_t
 */
mqd_t get_node_comm_queue( void );



#define POST_MESSAGE_NODE_COMM( p_node_msg, format, ... )  \
    do{ \
        ( strlen( format )>0 ) ? snprintf( ( p_node_msg )->message, sizeof( ( p_node_msg )->message ), format, ##__VA_ARGS__ ): 0;   \
        FILL_CHECKSUM( p_node_msg );   \
        __POST_MESSAGE_NODE_COMM( get_node_comm_queue(), ( p_node_msg ), sizeof( *p_node_msg ), 20 ); \
    }while( 0 )

#define POST_MESSAGE_NODE_COMM_EXIT( format, ... )  \
    do{ \
        node_message_t node_msg;    \
        ( strlen( format )>0 ) ? snprintf( node_msg.message, sizeof( node_msg.message ), format, ##__VA_ARGS__ ):0; \
        COMM_OBJECT_MSGID( node_msg, 0xFF ); \
        COMM_DST_BOARD_ID( node_msg, BBG_BOARD_ID ); \
        __POST_MESSAGE_NODE_COMM( get_node_comm_queue(), &node_msg, sizeof( node_msg ), 20 ); \
    }while( 0 )

/*!
 * @brief
 *
 * @param queue
 * @param node_msg
 * @param node_msg_size
 * @param prio
 */
static inline void __POST_MESSAGE_NODE_COMM( mqd_t queue, const node_message_t *node_msg, size_t node_msg_size, int prio )
{
    if( -1 == mq_send( queue, ( const char* )node_msg, node_msg_size, prio ) )
    {
        LOG_ERROR( "COMM_SEND:MQ_SEND:%s\n", strerror( errno ) );
    }
}



/*!
 * @brief
 *
 * @param board_id
 */
static inline void send_GET_CLIENT_INFO_BOARD_TYPE( uint8_t board_id )
{
    COMM_CREATE_OBJECT( node_msg, BBG_BOARD_ID, BBG_COMM_MODULE, TIVA_COMM_MODULE );
    COMM_OBJECT_MSGID( node_msg, NODE_MSG_ID_GET_CLIENT_BOARD_TYPE );
    COMM_DST_BOARD_ID( node_msg, board_id );
    FILL_CHECKSUM( &node_msg );
    POST_MESSAGE_NODE_COMM( &node_msg, "BBG/Req/BType" );
}

/*!
 * @brief
 *
 * @param board_id
 */
static inline void send_GET_CLIENT_INFO_UID( uint8_t board_id )
{
    COMM_CREATE_OBJECT( node_msg, BBG_BOARD_ID, BBG_COMM_MODULE, TIVA_COMM_MODULE );
    COMM_OBJECT_MSGID( node_msg, NODE_MSG_ID_GET_CLIENT_UID );
    COMM_DST_BOARD_ID( node_msg, board_id );
    FILL_CHECKSUM( &node_msg );
    POST_MESSAGE_NODE_COMM( &node_msg, "BBG/Req/UID" );
}

/*!
 * @brief
 *
 * @param board_id
 */
static inline void send_GET_SENSOR_STATUS( uint8_t board_id, uint8_t src_module_id )
{
    COMM_CREATE_OBJECT( node_msg, BBG_BOARD_ID, src_module_id, TIVA_SENSOR_MODULE );
    COMM_OBJECT_MSGID( node_msg, NODE_MSG_ID_GET_SENSOR_STATUS );
    COMM_DST_BOARD_ID( node_msg, board_id );
    FILL_CHECKSUM( &node_msg );
    POST_MESSAGE_NODE_COMM( &node_msg, "BBG/Req/Distance" );
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
