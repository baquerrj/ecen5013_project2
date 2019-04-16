/*!
 * @file	apds9301_sensor.h
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef APDS9301_SENSOR_H_
#define APDS9301_SENSOR_H_

#include <stdint.h>

#define APDS9301_ADDR           (0x39)

#define APDS9301_ID             (0x50)
/** Register adddresses */
#define APDS9301_REG_CMD        (0x80)
#define APDS9301_REG_CNTRL      (0x80)
#define APDS9301_REG_TIME       (0x81)
#define APDS9301_REG_TH_LL      (0x82)
#define APDS9301_REG_TH_LH      (0x83)
#define APDS9301_REG_TH_HL      (0x84)
#define APDS9301_REG_TH_HH      (0x85)
#define APDS9301_REG_INT_CNTRL  (0x86)
#define APDS9301_REG_ID         (0x8A)
#define APDS9301_REG_DL_0       (0x8C)
#define APDS9301_REG_DH_0       (0x8D)
#define APDS9301_REG_DL_1       (0x8E)
#define APDS9301_REG_DH_1       (0x8F)

/** Helpful constants */
#define POWER_ON     (0x03)
#define POWER_OFF    (0x00)
#define CMD_CLEAR_INTR  (1<<5)
#define CMD_WORD_ENBL   (1<<6)

/** Defaults */
#define DEFAULT_GAIN             (0x00)   /** low gain */
#define DEFAULT_INTEGRATION_TIME (0x02)   /** 402ms integration time */
#define DEFAULT_INTERRUPT        (0x00)   /** No interrupts */

#define DARK_THRESHOLD           (50)


/*! @brief Read ADC register and calculate luminosity
 *
 * @param[out]  lux - calculated ambient light luminosity
 */
void apds9301_get_lux( float *lux );

/*! @brief Read contents of control register
 *
 * @returns data from register
 */
uint8_t apds9301_read_control( void );

/*! @brief Read APDS9301 Sensor ID from ID register
 *
 * @returns sensor id
 */
uint8_t apds9301_read_id( void );

/*! @brief Read ADC CH 0 register
 *
 * @returns data from register
 */
uint16_t apds9301_read_data0( void );

/*! @brief Read ADC CH 1 register
 *
 * @returns data from register
 */
uint16_t apds9301_read_data1( void );

/*! @brief Set Power for APDS9301
 *
 * @param[in]   power on or power off the sensor
 */
void apds9301_power_on( uint8_t on );

#endif /* APDS9301_SENSOR_H_ */
