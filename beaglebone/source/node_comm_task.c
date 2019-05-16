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
#include "logger.h"
#include "node_comm_task.h"
#include "communication_interface.h"
#include "nrf_module.h"

#define NODE_COMM_QUEUE "/node_comm_queue"

typedef enum
{
    REQ_TEMP,
    REQ_LUX
} req_e;
static timer_t    timerid;
struct itimerspec trigger;

static int uart_fd;

static req_e req = REQ_TEMP;
static mqd_t node_comm_task_queue;

static message_t comm_log = {
   .level      = LEVEL_INFO,
   .timestamp  = {0},
   .id         = MSG_STATUS,
   .src        = TASK_NODE_COMM,
   .msg        = {0}
};


static void dump_message( node_message_t *p )
{
    printf( "SRC BOARD ID: %s\tSRC ID: %s\n", get_board_id_name( p->src_brd_id ), get_bbg_module_name( p->src_id ) );
    printf( "DST BOARD ID: %s\tDST ID: %s\n", get_board_id_name( p->dst_brd_id ), get_tiva_module_name( p->dst_id ) );
    printf( "MSG ID: %s\n", get_message_id_name( p->msg_id ) );
    printf( "MSG: %s\n", p->message );

}

/*!
 * Function:       sig_handler
 * @brief   Signal handler for temperature sensor thread.
 *          On normal operation, we should be receving SIGUSR1/2 signals from watchdog
 *          when prompted to exit. So, we close the message queue and timer this thread owns
 *
 * @param   signo - enum with signal number of signal being handled
 * @return  void
 */
static void sig_handler( int signo )
{
    if( signo == SIGUSR1 )
    {
        LOG_INFO( "NODE COMM TASK: Received SIGUSR1! Exiting...\n");
        mq_close( node_comm_task_queue );
        timer_delete( timerid );
        comm_deinit_uart( uart_fd );
        thread_exit( signo );
    }
    else if( signo == SIGUSR2 )
    {
        LOG_INFO( "NODE COMM TASK: Received SIGUSR2! Exiting...\n");
        mq_close( node_comm_task_queue );
        timer_delete( timerid );
        comm_deinit_uart( uart_fd );
        thread_exit( signo );
    }
    return;
}

void comms_handler( union sigval sig )
{
    static node_message_t node_req_out;
    node_req_out.msg_id = NODE_MSG_ID_GET_BOARD_TYPE;
//    node_req_out.src_brd_id = BOARD_ID_BBG;
//    node_req_out.src_id = BBG_MODULE_COMM;
//    node_req_out.dst_brd_id = BOARD_ID_TIVA;
    switch( node_req_out.msg_id )
    {
        case NODE_MSG_ID_GET_TEMP:
        {
            LOG_TASK_MSG( LEVEL_INFO, "REQ TEMP" );
//            node_req_out.dst_id = TIVA_MODULE_TMP102;
//            node_req_out.msg_id = NODE_MSG_ID_GET_TEMPERATURE;
            send_get_temperature();
            break;
        }
        case NODE_MSG_ID_GET_LUX:
        {
            LOG_TASK_MSG( LEVEL_INFO, "REQ LUX" );
//            node_req_out.dst_id = TIVA_MODULE_APDS9301;
//            node_req_out.msg_id = NODE_MSG_ID_GET_LUX;
            send_get_lux();
            break;
        }
        case NODE_MSG_ID_GET_CLIENT_BOARD_TYPE:
        {
            LOG_TASK_MSG( LEVEL_INFO, "REQ BOARD TYPE" );
//            node_req_out.dst_id = TIVA_MODULE_COMM;
//            node_req_out.msg_id = NODE_MSG_ID_GET_LUX;
            send_get_board_type();
            break;
            
        }
        default:
            break;
    }
//    CALC_CHECKSUM( &node_req_out );
//    comm_send_uart( &node_req_out );
    dump_message( &node_req_out );
}


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

static int8_t startup( void )
{
    uint8_t tiva_detected = 0;
    uint32_t attempts = 0;
    node_message_t node_msg = {0};
    node_message_t node_msg_out = {0};
    while( !tiva_detected && 100 > attempts )
    {
        node_msg_out.dst_id = TIVA_MODULE_COMM;
        node_msg_out.msg_id = NODE_MSG_ID_ALIVE;
        CALC_CHECKSUM( &node_msg_out );
        comm_send_uart( &node_msg_out );
        delayMs( 5 );
        memset( &node_msg, 0, sizeof( node_msg ) );
        comm_recv_uart( &node_msg );
        if( (node_msg.src_brd_id == TIVA_BOARD_ID) && (node_msg.dst_brd_id == BBG_BOARD_ID) )
        {
            LOG_INFO( "TIVA DETECTED\n" );
            dump_message( &node_msg );
            tiva_detected = 1;
        }
        else
        {
            attempts++;
            delayMs( 5 );
            continue;
        }
    }
    if( !tiva_detected || 100 < attempts )
    {
        return -1;
    }
    return 0;
}


static void cycle( void )
{
    node_message_t node_msg_in = {0};
    while( 1 )
    {
        memset( &node_msg_in, 0, sizeof( node_msg_in ) );
        comm_recv_uart( &node_msg_in );
        switch( node_msg_in.msg_id )
        {
            case NODE_MSG_ID_ALIVE:
            {
                break;
            }
            case NODE_MSG_ID_INFO:
            {
                break;
            }
            case NODE_MSG_ID_ERROR:
            {
                break;
            }
            case NODE_MSG_ID_WARNING:
            {
                break;
            }
            case NODE_MSG_ID_PICTURE:
            {
                break;
            }
            case NODE_MSG_ID_OBJECT_DETECTED:
            {
                break;
            }
            case NODE_MSG_ID_BOARD_TYPE:
            {
                break;
            }
            case NODE_MSG_ID_UID:
            {
                break;
            }
            case NODE_MSG_ID_SENSOR_STATUS:
            {
                switch( node_msg_in.src_id )
                {
                    dump_message( &node_msg_in );
                    case TIVA_MODULE_TMP102:
                    {
                        comm_log.id = MSG_STATUS;
                        LOG_TASK_MSG( LEVEL_INFO, &comm_log, "TEMP: %.5f C\n", node_msg_in.data.sensor_value );
                        break;
                    }
                    case TIVA_MODULE_APDS9301:
                    {
                        comm_log.id = MSG_STATUS;
                        LOG_TASK_MSG( LEVEL_INFO, &comm_log, "LUX: %.5f LUX\n",  node_msg_in.data.sensor_value );
                        break;
                    }
                    default:
                        break;
                }
            }
            default:
                break;
        }
    }
}

void* node_comm_task_fn( void *thread_args )
{
    __start_timer();
    LOG_INFO( "NODE COMM TASK STARTED\n" );
    node_comm_task_queue = node_comm_task_queue_init();

    if( 0 > node_comm_task_queue )
    {
        LOG_TASK_MSG( LEVEL_ERROR,  &comm_log, "NODE COMM TASK INIT" );
        LOG_ERROR( "NODE COMM TASK INIT\n" );
        thread_exit( EXIT_INIT );
    }

//    if( 0 > comm_init_nrf() )
//    {
//        LOG_ERROR( "NODE COMM TASK: NRF INIT\n" );
//        thread_exit( EXIT_INIT );
//    }

    signal( SIGUSR1, sig_handler );
    signal( SIGUSR2, sig_handler );

    uart_fd = comm_init_uart();
    if( 0 > uart_fd )
    {
        LOG_TASK_MSG( LEVEL_ERROR, &comm_log, "NOCE COMM TASK UART INIT\n" );
        LOG_ERROR( "NOCE COMM TASK UART INIT\n" );
        thread_exit( EXIT_INIT );
    }

    LOG_TASK_MSG( LEVEL_INFO, &comm_log, "NODE COMM TASK INITIALIZED\n" );
    LOG_INFO( "NODE COMM TASK INITIALIZED\n" );

    uint8_t retries = 100;
    while( retries > 0 && 0 != startup() )
    {
        LOG_ERROR( "TIMED OUT WAITING FOR TIVA. Retrying...\n" );
        retries--;
    }
    if( 0 == retries )
    {
        thread_exit( EXIT_INIT );
    }

    timer_setup( &timerid, &comms_handler );

    timer_start( &timerid, FREQ_1HZ );

    cycle();

//    comm_deinit_nrf();
    comm_deinit_uart( uart_fd );
    thread_exit( 0 );
    return NULL;
}

