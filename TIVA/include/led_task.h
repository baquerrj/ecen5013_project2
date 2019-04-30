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
#include "task.h"


xSemaphoreHandle    g_LedMutex;
xQueueHandle        g_pLedQueue;

/*!
 * @brief Initialize 10Hz LED task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t led_task_init( void );

void led_task_callback( TimerHandle_t timer );


#endif /* LED_TASK_H_ */
