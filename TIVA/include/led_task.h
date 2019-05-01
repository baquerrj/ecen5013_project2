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

#include "logger_task.h"

xSemaphoreHandle    g_LedMutex;
xQueueHandle        g_pLedQueue;


#define SENSOR_MSG( p_log, fmt, ... ) \
    do{ \
        xSemaphoreTake( g_LedMutex, portMAX_DELAY ); \
        snprintf( (p_log)->msg, sizeof( (p_log)->msg), fmt, ##__VA_ARGS__); \
        led_enqueue( g_pLedQueue, p_log, sizeof( p_log ) ); \
        xSemaphoreGive( g_LedMutex ); \
    }while(0)

/*!
 * @brief Initialize 10Hz LED task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t led_task_init( void );

void led_task_callback( TimerHandle_t timer );

/*!
 * @brief Add logging message to queue
 *
 * @param[in]   queue handle
 * @param[in]   message to queue
 * @param[in]   size of message
 */
void led_enqueue( QueueHandle_t queue, const log_msg_t *msg_out, size_t size );

#endif /* LED_TASK_H_ */
