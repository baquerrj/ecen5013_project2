/**
 * =================================================================================
 *    @file    light.h
 *    @brief   APDS9301 Sensor Task
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

#ifndef APDS9301_TASK_H
#define APDS9301_TASK_H

#include "common.h"
#include "apds9301_sensor.h"

#define APDS9301_QUEUE "/apds9301-queue"



/**
 * Function:       get_lux
 * @brief   Returns last lux reading
 *
 * @param   void
 * @return  last_lux_value - last lux reading we have
 */
float get_lux( void );

/**
 * Function:       is_dark
 * @brief   Returns int speciyfing if it is night or day
 *
 * @param   void
 * @return  night - 0 if it is day, 1 if night, i.e. below DARK_THRESHOLD
 */
int is_dark( void );

/**
 * Function:       get_apds9301_queue
 * @brief   Get file descriptor for light sensor thread.
 *          Called by watchdog thread in order to be able to send heartbeat check via queue
 *
 * @param   void
 * @return  temp_queue - file descriptor for light sensor thread message queue
 */
mqd_t get_apds9301_queue( void );

/**
 * Function:       apds9301_queue_init
 * @brief   Initialize message queue for light sensor thread
 *
 * @param   void
 * @return  msg_q - file descriptor for initialized message queue
 */
int apds9301_queue_init( void );

/**
 * Function:       apds9301_fn
 * @brief   Entry point for light sensor processing thread
 *
 * @param   thread_args - void ptr to arguments used to initialize thread
 * @return  NULL  - We don't really exit from this function,
 *                   since the exit point is thread_exit()
 */
void *apds9301_fn( void *thread_args );


#endif /* APDS9301_TASK_H */
