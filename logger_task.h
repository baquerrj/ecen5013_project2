/*!
 * @file	logger_task.h
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef LOGGER_TASK_H_
#define LOGGER_TASK_H_

typedef enum
{
    MSG_BEGIN = 0,
    MSG_TOGGLE_LED,
    MSG_GET_TEMP,
    MSG_TEMP_LOW,
    MSG_TEMP_HIGH,
    MSG_MAX = -1
} log_msg_e;

typedef struct
{
    TickType_t tickcount;
    log_msg_e type;
    const char *src;
    char msg[25];
    union data
    {
        float temperature;
        uint32_t toggle_count;
    } data;
} log_msg_t;


/*!
 * @brief Logger Task function
 *
 * @param params - FreeRTOS TaskHandle_t params
 * @returns void
 */
void logger_task( void *params );

/*!
 * @brief Initialize logger task by calling FreeRTOS mechanisms
 *          xTaskCreate and xQueueCreate
 * @oaram void
 * @returns 0 if successful
 */
uint8_t logger_task_init( void );


#endif /* LOGGER_TASK_H_ */
