/**
 * =================================================================================
 *    @file     common.h
 *    @brief   Defines types and functions common between the threads of the application
 *
 *
 *    @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 *    @internal
 *       Created:  03/09/2019
 *      Revision:  none
 *      Compiler:  gcc
 *  Organization:  University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 * =================================================================================
 */

#ifndef COMMON_H
#define COMMON_H


#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <mqueue.h>
#include <time.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>

/* Common local includes */
#include "led.h"
#include "delay.h"

#define MSG_SIZE 100
#define MAX_MESSAGES 100

#define MICROS_PER_SEC  1000000

#define FREQ_1HZ        (MICROS_PER_SEC)
#define FREQ_2HZ        (MICROS_PER_SEC/2)
#define FREQ_4HZ        (MICROS_PER_SEC/4)
#define FREQ_HALF_HZ    (MICROS_PER_SEC*2)
#define FREQ_QURT_HZ    (MICROS_PER_SEC*4)

#define ERROR   "[ERROR]\n"
#define INFO    "[INFO]\n"
#define SIGNAL  "[SIGNAL]\n"
#define WARNING "[WARNING]\n"

#define FPRINTF(stream, fmt, ...)   \
   do{ \
      fprintf(stream, "[%s]\t[PID:%d]\t[TID:%ld]", get_timestamp(), getpid(), syscall(SYS_gettid)); \
      fprintf(stream, fmt, ##__VA_ARGS__); \
      fflush( stream ); \
   }while(0)

#define LOG_ERROR(fmt, ...)  \
   do{ \
      FPRINTF( stderr, ERROR fmt, ##__VA_ARGS__ ); \
      fflush( stderr ); \
   }while(0)

#define LOG_INFO(fmt, ...)  \
   do{ \
      FPRINTF( stderr, INFO fmt, ##__VA_ARGS__ ); \
      fflush( stdout ); \
   }while(0)

#define LOG_WARNING(fmt, ...)  \
   do{ \
      FPRINTF( stderr, WARNING fmt, ##__VA_ARGS__ ); \
      fflush( stderr ); \
   }while(0)


/*******************************************************************************
 *  Struct to hold thread identifiers for tasks
 ******************************************************************************/

typedef enum {
   TASK_LOGGER = 0,
   TASK_NODE_COMM,
   TASK_WATCHDOG,
   TASK_MAIN,
   TASK_MAX = TASK_MAIN
} task_e;

#define NUM_THREADS         (TASK_MAX)

pthread_t task_id[ TASK_MAX ];

/*! @brief Logging levels */
typedef enum
{
   LEVEL_ERROR,
   LEVEL_WARNING,
   LEVEL_INFO,
   LEVEL_ALL
} log_level_e;



/*******************************************************************************
 *  Defines types of possible messages
 ******************************************************************************/
typedef enum {
   MSG_BEGIN = 0,
   MSG_CLOSE,
   MSG_KILL,
   MSG_STATUS,
   MSG_ALIVE,
   MSG_MAX
} message_e;


/*! @brief Log Message Structure */
typedef struct
{
   log_level_e level;
   char        timestamp[25];
   message_e   id;
   task_e      src;
   char        msg[MSG_SIZE];
} message_t;


/*******************************************************************************
 *
 ******************************************************************************/
typedef struct {
   char *name;
   FILE *fid;
} file_t;

/*******************************************************************************
 *  Exit Enum
 ******************************************************************************/
typedef enum {
   EXIT_BEGIN = 0,
   EXIT_CLEAN = 0,
   EXIT_INIT,
   EXIT_ERROR,
   EXIT_MAX
} exit_e;

extern const char* const task_names[ TASK_MAX ];


/*******************************************************************************
 *  Controller + Remote Node Interface
 ******************************************************************************/
/*! @brief BBG and TIVA Board IDs */


typedef enum
{
    BOARD_ID_BBG    = 0,
    BOARD_ID_TIVA   = 1,
    BOARD_ID_MAX
} board_e;

extern const char* const board_id_names[ BOARD_ID_MAX ];

typedef enum
{
    TIVA_MODULE_TMP102   = 0,
    TIVA_MODULE_APDS9301,
    TIVA_MODULE_CAMERA,
    TIVA_MODULE_COMM,
    TIVA_MODULE_LED,
    TIVA_MODULE_MAX
} tiva_module_e;

extern const char* const tiva_module_names[ TIVA_MODULE_MAX ];

typedef enum
{
    BBG_MODULE_LOGGER = 0,
    BBG_MODULE_COMM,
    BBG_MODULE_MAX
} bbg_module_e;

extern const char* const bbg_module_names[ BBG_MODULE_MAX ];

/*! @brief Message ID for messages between nodes */
typedef enum
{
    NODE_MSG_ID_ALIVE = 0,
    NODE_MSG_ID_INFO,
    NODE_MSG_ID_ERROR,
    NODE_MSG_ID_WARNING,
    NODE_MSG_ID_SENSOR_STATUS,
    NODE_MSG_ID_PICTURE,
    NODE_MSG_ID_OBJECT_DETECTED,
    NODE_MSG_ID_BOARD_TYPE,
    NODE_MSG_ID_UID,

    NODE_MSG_ID_GET_TEMPERATURE,
    NODE_MSG_ID_GET_LUX,
    NODE_MSG_ID_GET_SENSOR_INFO,
    NODE_MSG_ID_GET_CLIENT_BOARD_TYPE,
    NODE_MSG_ID_GET_CLIENT_UID,
    NODE_MSG_ID_MAX
} node_message_e;

extern const char* const node_message_id_names[ NODE_MSG_ID_MAX ];

#define BBG_BOARD_ID        BOARD_ID_BBG
#define TIVA_BOARD_ID       BOARD_ID_TIVA

#define TIVA_TMP102_MODULE      TIVA_MODULE_TMP102
#define TIVA_APDS9301_MODULE    TIVA_MODULE_APDS9301
#define TIVA_CAMERA_MODULE      TIVA_MODULE_CAMERA
#define TIVA_COMM_MODULE        TIVA_MODULE_COMM
#define TIVA_LED_MODULE         TIVA_MODULE_LED

#define BBG_LOGGER_MODULE       BBG_MODULE_LOGGER
#define BBG_COMM_MODULE         BBG_MODULE_COMM

typedef struct
{
    size_t length;
    void* frame;
} camera_packet_t;

/*! @brief Struct defining inter-node communication */
typedef struct
{
    uint8_t src_id;
    uint8_t src_brd_id;
    uint8_t dst_id;
    uint8_t dst_brd_id;
    node_message_e msg_id;
    union
    {
        float float_data;
        float sensor_value;
        camera_packet_t *camera_packet;
        size_t filler;
    } data;
    char message[18];
    uint16_t checksum;
} node_message_t;

static uint16_t getCheckSum( const node_message_t *node_msg )
{
    uint16_t checkSum = 0;
    uint8_t sizeOfPayload = sizeof( node_message_t ) - sizeof( node_msg->checksum );
    uint8_t *p_payload = (uint8_t*)node_msg;
    int i;
    for(i = 0; i < sizeOfPayload; i++)
    {
        checkSum += *( p_payload + i );
    }
    return checkSum;
}

/*!
 * @brief   Verify checksum
 *
 * @param[in]   node_msg message from remote node to verify
 * @returns 1 if a match
 */
static inline uint8_t verifyCheckSum( const node_message_t *node_msg )
{
    return getCheckSum( node_msg ) == node_msg->checksum;
}

/*!
 * @brief   Get string representaion of task requesting logging
 *
 * @param[in]   task_id ID according to task_e enum
 * @return string represenation from task_name array
 */
static inline const char* get_task_name( task_e task_id )
{
   return task_names[ task_id ];
}

/*!
 * @brief   Get string represenation of node message ID
 *
 * @param[in]   msg_id ID of message according to node_message_e enum
 * @return string represenation of message
 */
static inline const char* get_message_id_name( node_message_e msg_id )
{
    return node_message_id_names[ msg_id ];
}

static inline const char* get_board_id_name( board_e board_id )
{
    return board_id_names[ board_id ];
}

static inline const char* get_tiva_module_name( tiva_module_e module_id )
{
    return tiva_module_names[ module_id ];
}

static inline const char* get_bbg_module_name( bbg_module_e module_id )
{
    return bbg_module_names[ module_id ];
}

/*!
 * @brief   Get timestamp and format it as a string
 *
 * @return formatted timestamp
 * <+DETAILED+>
 */
char* get_timestamp( void );


/*!
 * @brief   Common exit point for all threads
 *
 * @param   exit_status - reason for exit (signal number)
 * @return  void
 */
void thread_exit( int exit_status );


/*!
 * @brief   Initializes a timer identified by timer_t id
 *
 * @param   *id   - identifier for new timer
 * @param   *handler - pointer to function to register as the handler for the timer ticks
 * @return  EXIT_CLEAN if successful, otherwise EXIT_INIT
 */
int timer_setup( timer_t *id, void (*timer_handler)(union sigval) );


/*!
 * @brief   Starts the timer with interval usecs
 *
 * @param   *id   - identifier for new timer
 * @param   usecs - timer interval
 * @return  EXIT_CLEAN if successful, otherwise EXIT_INIT
 */
int timer_start( timer_t *id, unsigned long usecs );


#endif /* COMMON_H */
