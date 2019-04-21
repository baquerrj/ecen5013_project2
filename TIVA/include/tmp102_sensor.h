/*!
 * @file	tmp102_sensor.h
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef TMP102_SENSOR_H_
#define TMP102_SENSOR_H_


#define TMP102_SLAVE_ADDR           (0x48)


/* Register address */
#define TMP102_REG_TEMPERATURE          (0x00)
#define TMP102_REG_CONFIGURATION        (0x01)
#define TMP102_REG_TLOW                 (0x02)
#define TMP102_REG_THIGH                (0x03)


#define TMP102_CONFIG_SD                (1)
#define TMP102_CONFIG_TM                (1<<1)
#define TMP102_CONFIG_POL               (1<<2)
#define TMP102_CONFIG_EM                (1<<12)
#define TMP102_CONFIG_AL                (1<<13)
#define TMP102_CONFIG_CR(x)             (x<<14)

#define TMP102_CONFIG_FAULTBITS                (3<<3)              /*generates alert after 4 consecutive faults*/
#define TMP102_CONFIG_ONESHOT_CR               (1<7)              /*saves power between conversions when 1*/
/*! @brief Reads temperature data from TMP102 sensor registers
 *          and writes decoded value to input variable
 * @param[out]  temp
 * @returns     void
 */
void tmp102_get_temp( float *temp );


#endif /* TMP102_SENSOR_H_ */
