/*!
 * @file	tmp102_sensor.c
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#include "tmp102_sensor.h"
#include "my_i2c.h"

void tmp102_get_temp( float *temp )
{
    uint16_t temp_raw = I2C2_read_word( TMP102_SLAVE_ADDR, TMP102_REG_TEMPERATURE );

    temp_raw = (temp_raw>>4) & 0xFFF;

    if(temp_raw & 0x800)
    {
        temp_raw = ((~temp_raw) + 1) & 0xFFF;
        *temp = (-1) * (float)temp_raw * 0.0625;
    }
    else
    {
        *temp = ((float)temp_raw)*0.0625;
    }

    return;
}
