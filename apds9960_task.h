/*!
 * @file	apds9960_task.h
 *
 * @brief
 *
 *  Created on: Apr 16, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef APDS9960_TASK_H_
#define APDS9960_TASK_H_

#include "timers.h"

/*!
 * @brief Timer Callback for APDS9960 Sensor Task
 *
 * @param timer - FreeRTOS TimerHandle_t associated with task
 * @returns void
 */
void apds9960_task_callback( TimerHandle_t timer );

/*!
 * @brief Initialize 1Hz Temperature Sensor task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t apds9960_task_init( void );



#endif /* APDS9960_TASK_H_ */
