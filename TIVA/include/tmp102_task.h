/*
 * tmp102_task.h
 *
 *  Created on: Apr 8, 2019
 *      Author: rober
 */

#ifndef TMP102_TASK_H_
#define TMP102_TASK_H_

#include "timers.h"


#define POST_TEMP_WARNING( p_log, fmt, ... ) \
    do{ \
        xSemaphoreTake( g_LedMutex, portMAX_DELAY ); \
        snprintf( (p_log)->msg, sizeof( (p_log)->msg), fmt, ##__VA_ARGS__); \
        enqueue( g_pLedQueue, p_log, sizeof( p_log ) ); \
        xSemaphoreGive( g_LedMutex ); \
    }while(0)

void get_temperature( float *data );


/*!
 * @brief Timer Callback for Temperature Sensor Task
 *
 * @param timer - FreeRTOS TimerHandle_t associated with task
 * @returns void
 */
void tmp102_task_callback( TimerHandle_t timer );

/*!
 * @brief Initialize 1Hz Temperature Sensor task
 *
 * @param void
 * @returns 0 if successful
 */
uint8_t temp_task_init( void );

#endif /* TMP102_TASK_H_ */
