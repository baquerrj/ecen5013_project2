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

#include <signal.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include <time.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>

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
   TASK_COMM_SENDER,
   TASK_WATCHDOG,
   TASK_MAX
} task_e;

#define NUM_THREADS         (TASK_MAX)

pthread_t task_id[ TASK_MAX ];

/*! @brief Logging levels */
typedef enum
{
   LOG_ERROR,
   LOG_WARNING,
   LOG_INFO,
   LOG_ALL
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

extern const char* const task_name[ TASK_MAX ];


/*******************************************************************************
 *  Controller + Remote Node Interface
 ******************************************************************************/
/*! @brief BBG and TIVA Board IDs */
#define BBG_BOARD_ID        (0x00)
#define TIVA_BOARD_ID       (0x01)

#define TIVA_HEART_BEAT_MODULE  (1)
#define TIVA_SENSOR_MODULE      (2)
#define TIVA_CAMERA_MODULE      (3)
#define TIVA_COMM_MODULE        (4)
#define TIVA_LED_MODULE         (5)

#define BBG_LOGGER_MODULE       (1)
#define BBG_COMM_MODULE         (2)

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

    NODE_MSG_ID_GET_SENSOR_STATUS,
    NODE_MSG_ID_GET_SENSOR_INFO,
    NODE_MSG_ID_GET_CLIENT_BOARD_TYPE,
    NODE_MSG_ID_GET_CLIENT_UID,
    NODE_MSG_ID_MAX
} node_message_e;

extern const char* const node_message_names[ NODE_MSG_ID_MAX ];

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




/*!
 * @brief   Get string representaion of task requesting logging
 *
 * @param[in]   task_id ID according to task_e enum
 * @return string represenation from task_name array
 */
static inline const char* get_task_name( task_e task_id )
{
   return task_name[ task_id ];
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
