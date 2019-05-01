/*!
 * @file	apds9301_task.h
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef APDS9301_TASK_H_
#define APDS9301_TASK_H_

#include "timers.h"

void get_lux( float *data );

/*!
 * @brief Timer Callback for APDS9301 Sensor Task
 *
 * @param timer - FreeRTOS TimerHandle_t associated with task
 * @returns void
 */
void apds9301_task_callback( TimerHandle_t timer );

/*!
 * @brief Initialize 1Hz Temperature Sensor task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t apds9301_task_init( void );


#endif /* APDS9301_TASK_H_ */
