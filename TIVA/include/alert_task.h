/*!
 * @file	alert_task.h
 *
 * @brief
 *
 *  Created on: Apr 10, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef ALERT_TASK_H_
#define ALERT_TASK_H_

#include "task.h"

/*!
 * Threshold for notifying alert task of high temperature
 * 27 Celsius ~ 80 Fahrenheit
 */
#define HIGH_TEMPERATURE   (27)

/*!
 * Threshold for notifying alert task of low temperature
 * 21 Celsius ~ 70 Fahrenheit
 */
#define LOW_TEMPERATURE     (21)

/*!
 * @brief Initialize Temperature Monitor Task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t alert_task_init( void );

#endif /* ALERT_TASK_H_ */
