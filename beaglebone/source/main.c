/**
 * =================================================================================
 *    @file     main.c
 *    @brief
 *
 *  <+DETAILED+>
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


#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
/** /sys includes */
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "logger.h"
#include "common.h"
#include "watchdog.h"
#include "led.h"

#include "node_comm_task.h"
#include "communication_interface.h"

static message_t main_log = {
   .level      = LEVEL_INFO,
   .timestamp  = {0},
   .id         = MSG_STATUS,
   .src        = TASK_MAIN,
   .msg        = {0}
};


void* (*thread_entry_fn[ NUM_THREADS ])(void *) =
{
    logger_fn,
    node_comm_task_fn,
    watchdog_fn,
};

/*!
 * @brief

 */
static void signal_handler( int signo )
{
   switch( signo )
   {
      case SIGINT:
         LOG_ERROR( "Master caught SIGINT!\n" );
         pthread_kill( task_id[TASK_WATCHDOG], SIGUSR2 );
   }
}

/*!
 * @brief
 *
 */
void turn_off_leds( void )
{
    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    return;
}

/*!
 * @brief
 *
 */
int main( int argc, char *argv[] )
{
   signal( SIGINT, signal_handler );
   static file_t *log;
   if( argc > 1 )
   {
      log = malloc( sizeof( file_t ) );
      log->fid = fopen( argv[1], "w" );
      snprintf( log->name, sizeof( log->name ),  "%s", argv[1] );
      LOG_INFO( "Opened file %s\n", argv[1] );
   }
   else
   {
       log = malloc( sizeof( file_t ) );
       //sprintf( log->name, "%d,controller.log", getpid() );
       snprintf( log->name, sizeof( log->name ), "%d,controller.log", getpid() );
       log->fid = fopen( log->name, "w" );
       LOG_INFO( "Opened file %s\n", log->name );
   }

   LOG_INFO( "Starting Threads!\n" );


   led_on( LED2_BRIGHTNESS );


   set_trigger( LED2_TRIGGER, "timer" );
   set_delay( LED2_DELAYON, 50 );

   /* Attempting to spawn child threads */
   for( uint8_t i = 0; i < TASK_MAX; ++i )
   {
      if( 0 != pthread_create( &task_id[i], NULL, thread_entry_fn[i], (void*)log->fid ) )
      {
         int errnum = errno;
         LOG_ERROR( "Could not create thread %d (%s)\n", i, strerror( errnum ) );
         return 1;
      }
   }
   LOG_INFO( "MAIN: CHILD THREADS INIT\n" );

   pthread_join( task_id[TASK_WATCHDOG], NULL );


   LOG_INFO( "All threads exited! Main thread exiting... " );

   free( log );
   turn_off_leds();
   return 0;
}
