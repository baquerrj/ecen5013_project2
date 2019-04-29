/*!
 * @file	delay.h
 *
 * @brief
 *
 *  Created on: Apr 20, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef _DELAY_H_
#define _DELAY_H_


#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

extern uint32_t g_sysClock;

static inline void delayMs( uint32_t ms )
{
    MAP_SysCtlDelay( (g_sysClock/(1000 *3)) * ms );
}

static inline void delayUs( uint32_t us)
{
    MAP_SysCtlDelay( (g_sysClock/(1000000 * 3)) *us );
}

#define __millis()  xTaskGetTickCount()
#endif /* _DELAY_H_ */
