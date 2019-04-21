/*!
 * @file	apds9301_sensor.c
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#include "apds9301_sensor.h"
#include "my_i2c.h"
#include <math.h>

void apds9301_get_lux( float *lux )
{
    float ratio = 0;
    uint16_t data0 = 0;
    uint16_t data1 = 0;

    data0 = apds9301_read_data0();

    data1 = apds9301_read_data1();


    if( 0 == data0 )
    {
        ratio = 0.0;
    }
    else
    {
        ratio = (float)data1 / (float)data0;
    }

    if( (0 < ratio) && (0.50 >= ratio) )
    {
        *lux = 0.0304*data0 - 0.062*data0*(pow(ratio, 1.4));
    }
    else if( (0.50 < ratio) && (0.61 >= ratio) )
    {
        *lux = 0.0224*data0 - 0.031*data1;
    }
    else if( (0.61 < ratio) && (0.80 >= ratio) )
    {
        *lux = 0.0128*data0 - 0.0153*data1;
    }
    else if( (0.80 < ratio) && (1.30 >= ratio) )
    {
        *lux = 0.00146*data0 - 0.00112*data1;
    }
    else if( 1.30 < ratio )
    {
        *lux = 0;
    }
    return;
}

uint8_t apds9301_read_control( void )
{
    uint8_t data = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_CNTRL );
    return data;
}

uint8_t apds9301_read_id( void )
{
    uint8_t id = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_ID );
    return id;
}

uint16_t apds9301_read_data0( void )
{
    uint8_t buff[2];
    buff[1] = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_DL_0 );
    buff[0] = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_DH_0 );

    uint16_t data = (buff[0] << 8) | buff[1];
    return data;
}


uint16_t apds9301_read_data1( void )
{
    uint8_t buff[2];
    buff[1] = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_DL_1 );
    buff[0] = I2C2_read_byte( APDS9301_ADDR, APDS9301_REG_DH_1 );

    uint16_t data = (buff[0] << 8) | buff[1];
    return data;
}
void apds9301_power_on( uint8_t on )
{
    I2C2_write( APDS9301_ADDR, APDS9301_REG_CNTRL, on );
    return;
}

