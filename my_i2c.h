/*!
 * @file    my_i2c.h
 *
 * @brief
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */


#ifndef MY_I2C_H_
#define MY_I2C_H_

#include <stdint.h>
#include <stdbool.h>

//#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"

#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/*! @brief Initialize I2C Bus 2
 *
 * @param[in]   void
 * @returns     void
 */
void I2C2_init( void );

/*! @brief Read from register on I2C Bus 2
 *
 * @param[in]   slave_addr
 * @param[in]   reg
 * @returns     8-bit data from register
 */
uint8_t I2C2_read_byte( uint32_t slave_addr, uint8_t reg );

/*! @brief Read from 16-bit register on I2C Bus 2
 *
 * @param[in]   slave_addr
 * @param[in[   reg
 * @returns     16-bit data from register
 */
uint16_t I2C2_read_word( uint32_t slave_addr, uint8_t reg );


/*!@ brief Write to register on I2C Bus 2
 *
 * @param[in]   slave_addr
 * @param[in]   reg
 * @param[in]   data
 */
void I2C2_write( uint32_t slave_addr, uint8_t reg, uint8_t data );

/*! @brief Initialize I2C Bus 0
 *
 * @param[in]   void
 * @returns     void
 */
void I2C0_init( void );

/*! @brief Read from register on I2C Bus 0
 *
 * @param[in]   slave_addr
 * @param[in]   reg
 * @returns     8-bit data from register
 */
uint8_t I2C0_read_byte( uint32_t slave_addr, uint8_t reg );


/*! @brief Read from 16-bit register on I2C Bus 0
 *
 * @param[in]   slave_addr
 * @param[in[   reg
 * @returns     16-bit data from register
 */
uint16_t I2C0_read_word( uint32_t slave_addr, uint8_t reg );

#endif /* MY_I2C_H_ */
