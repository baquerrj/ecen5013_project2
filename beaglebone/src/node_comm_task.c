/*!
 * @file  node_comm_task.c
 * @brief
 *
 * <+DETAILED+>
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

#include <pthread.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <mqueue.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include "common.h"
#include "node_comm_task.h"
#include "communication_interface.h"
#include "nrf_module.h"

#define NODE_COMM_QUEUE "/node_comm_queue"

typedef enum
{
    ROLE_SENDER = 0,
    ROLE_RECV,
    ROLE_MAX
} role_e;

volatile role_e role = ROLE_SENDER;

static mqd_t node_comm_task_queue;

mqd_t get_node_comm_queue( void )
{
    return node_comm_task_queue;
}

mqd_t node_comm_task_queue_init( void )
{
    mq_unlink( NODE_COMM_QUEUE );
    struct mq_attr attr = {
        .mq_msgsize = sizeof(node_message_t),
        .mq_maxmsg = 128,
        .mq_flags = 0,
        .mq_curmsgs = 0
    };

    int msg_q = mq_open( NODE_COMM_QUEUE, O_CREAT | O_RDWR, 0666, &attr );
    if( 0 > msg_q )
    {
        int errnum = errno;
        LOG_ERROR( "NODE COMM TASK: QUEUE CREATE: %s\n",
                    strerror( errnum ) );
    }
    return msg_q;
}

static void cycle( void )
{
    role = ROLE_SENDER;
    while( 1 )
    {
        if( ROLE_RECV == role )
        {
            LOG_INFO( "WAITING TO RECEIVE\n" );
            nrf_start_listening();
            unsigned long time = __millis();
            while( !nrf_available() )
            {
                delayMs( 5 );
            }
            LOG_INFO( "NEW DATA AVAILABLE\n" );
            unsigned long data;
            nrf_read( &data, sizeof( unsigned long ) );
            LOG_INFO( "DATA: 0%lx\n", data );
            role = ROLE_SENDER;
        }
        else if( ROLE_SENDER == role )
        {
            LOG_INFO( "PREPARING TO SEND\n" );
            nrf_stop_listening();
            unsigned long time = __millis();

            LOG_INFO( "SENDING %lu\n", time );
            uint8_t ok = nrf_write( &time, sizeof( unsigned long ) );

            if( ok )
            {
                LOG_INFO( "TX SUCCESS\n" );
            }
            else
            {
                LOG_ERROR( "TX FAILED\n" );
            }
            //role = ROLE_RECV;
        }
    }
}

void* node_comm_task_fn( void *thread_args )
{
    LOG_INFO( "NODE COMM TASK STARTED\n" );
    node_comm_task_queue = node_comm_task_queue_init();

    if( 0 > node_comm_task_queue )
    {
        LOG_ERROR( "NODE COMM TASK INIT\n" );
        thread_exit( EXIT_INIT );
    }

    if( 0 > comm_init_nrf() )
    {
        LOG_ERROR( "NODE COMM TASK: NRF INIT\n" );
        thread_exit( EXIT_INIT );
    }


    LOG_INFO( "NODE COMM TASK INITIALIZED\n" );
    cycle();

    comm_deinit_nrf();
    thread_exit( 0 );
    return NULL;
}

