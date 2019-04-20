/*!
 * @file	my_tasks.h
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef LED_TASK_H_
#define LED_TASK_H_

#include "timers.h"
/*!
 * @brief Timer Callback for LED Task
 *
 * @param timer - FreeRTOS TimerHandle_t associated with task
 * @returns void
 */
void led_task_callback( TimerHandle_t timer );

/*!
 * @brief Initialize 10Hz LED task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t led_task_init( void );



#endif /* LED_TASK_H_ */
