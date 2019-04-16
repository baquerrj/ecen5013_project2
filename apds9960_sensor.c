/*!
 * @file	apds9960_sensor.c
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */


/*!
 *    @file     apds9960_sensor.c
 *    @brief
 *
 *  <+DETAILED+>
 *
 *    @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 *    @internal
 *       Created:  04/11/2019
 *      Revision:  none
 *      Compiler:  gcc
 *  Organization:  University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/*!
 * @file    SparkFun_APDS-9960.cpp
 * @brief   Library for the SparkFun APDS-9960 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright   This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9960 object, call init(), and call the appropriate functions.
 *
 * APDS-9960 current draw tests (default parameters):
 *   Off:                   1mA
 *   Waiting for gesture:   14mA
 *   Gesture in progress:   35mA
 */

#include "apds9960_sensor.h"
#include "my_i2c.h"

#include "uart.h"
/*! @todo fix this */
#define EXIT_CLEAN	0
#define EXIT_ERROR	1
#if 0
static gesture_data_type gesture_data_;

static int gesture_ud_delta_ = 0;
static int gesture_lr_delta_ = 0;
static int gesture_ud_count_ = 0;
static int gesture_lr_count_ = 0;
static int gesture_near_count_ = 0;
static int gesture_far_count_ = 0;
static int gesture_state_ = 0;
static int gesture_motion_ = DIR_NONE;
#endif


uint8_t apds9960_sensor_init( void )
{
   uint8_t retVal = 0;
   if( 0 != retVal )
   {
      printf( "ERROR - Could not initialize I2C Master instance\n" );
      return retVal;
   }
   uint8_t id;

   id = apds9960_read_id();
   if( ( APDS9960_ID_1 != id ) && ( APDS9960_ID_2 != id ) )
   {
      printf( "ERROR - Invalid ID: %u\n", id );
      return 1;
   }
   else
   {
      printf( "ID: %u\n", id );
   }

   setMode( ALL, OFF );



   I2C2_write( APDS9960_I2C_ADDR, APDS9960_ATIME, DEFAULT_ATIME );
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_WTIME, DEFAULT_WTIME);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG1, DEFAULT_CONFIG1);
   setLEDDrive(DEFAULT_LDRIVE);

   setProximityGain(DEFAULT_PGAIN );

   //setAmbientLightGain(DEFAULT_AGAIN) )
   setProximityIntLowThreshold(DEFAULT_PILT);
   setProximityIntHighThreshold(DEFAULT_PIHT);
   //setLightIntLowThreshold(DEFAULT_AILT)
   //setLightIntHighThreshold(DEFAULT_AIHT)
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_PERS, DEFAULT_PERS);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG2, DEFAULT_CONFIG2);
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG3, DEFAULT_CONFIG3);

    /* Set default values for gesture sense registers */
//    if( EXIT_CLEAN != setGestureEnterThresh(DEFAULT_GPENTH) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != setGestureExitThresh(DEFAULT_GEXTH) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GCONF1, DEFAULT_GCONF1) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != setGestureGain(DEFAULT_GGAIN) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != setGestureWaitTime(DEFAULT_GWTIME) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GPULSE, DEFAULT_GPULSE) ) {
//        return EXIT_ERROR;
//    }
//    if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_GCONF3, DEFAULT_GCONF3) ) {
//        return EXIT_ERROR;
//    }

    //setGestureIntEnable(DEFAULT_GIEN);


    setProximityGain(PGAIN_2X);
    enableProximitySensor( 0 );

   uint8_t proximity_gain = getProximityGain();
   if( DEFAULT_PGAIN != proximity_gain )
   {
      printf( "ERROR - PROXIMITY GAIN: [%d] != [%d]\n", DEFAULT_PGAIN, proximity_gain );
      return 1;
   }
   else
   {
      printf( "EXPECTED %d, ACTUAL %d\n", PGAIN_2X, proximity_gain );
   }

   return 0;
}



uint8_t getMode( void )
{
   uint8_t data = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_ENABLE );
   return data;
}


void setMode( uint8_t mode, uint8_t enable )
{
   uint8_t data;
   data = getMode();
   return;


   enable = enable & 0x01;
   if( (mode >= 0) && (mode <= 6) )
   {
      if( enable )
      {
         data |= (1 << mode);
      }
      else
      {
         data &= ~(1 << mode);
      }
   }
   else if( mode == ALL )
   {
      if( enable )
      {
         data = 0x7F;
      }
      else
      {
         data = 0x00;
      }
   }

   I2C2_write( APDS9960_I2C_ADDR, APDS9960_ENABLE, data );

   return;
}

void enablePower( void )
{
   setMode( POWER, 1 );
   return;
}

void disablePower( void )
{
   setMode( POWER, 0 );
   return;
}

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @returns proximity data
 */
uint8_t readProximity( void )
{
    /* Read value from proximity data register */
    uint8_t data = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_PDATA );

    return data;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts EXIT_CLEAN to enable hardware external interrupt on proximity
 * @returns void
 */
void enableProximitySensor(uint8_t interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    setProximityGain(DEFAULT_PGAIN);

    setLEDDrive(DEFAULT_LDRIVE);
    if( interrupts )
    {
        setProximityIntEnable(1);
    }
    else
    {
        setProximityIntEnable(0);
    }
    enablePower();
    setMode(PROXIMITY, 1);

    return;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @returns void
 */
void disableProximitySensor( void )
{

    setProximityIntEnable(0);
    setMode(PROXIMITY, 0);
    return;
}

/*******************************************************************************
 * Gain control
 ******************************************************************************/

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t getProximityGain( void )
{
    /* Read value from CONTROL register */
    uint8_t data = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONTROL );

    /* Shift and mask out PDRIVE bits */
    data = (data >> 2) & 0b00000011;

    return data;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return EXIT_CLEAN if operation successful. EXIT_ERROR otherwise.
 */
void setProximityGain( uint8_t drive )
{
    /* Read value from CONTROL register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONTROL );
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;

    /* Write register value back into CONTROL register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONTROL, val);
    return;
}

/*******************************************************************************
 * @brief Proximity Interrupt Threshold Controls
 ******************************************************************************/
/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t getProximityIntLowThreshold( void )
{
   /* Read value from PILT register */
   uint8_t data = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_PILT );
   return data;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] low threshold value
 */
void setProximityIntLowThreshold( uint8_t threshold )
{
   I2C2_write( APDS9960_I2C_ADDR, APDS9960_PILT, threshold );
   return;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t getProximityIntHighThreshold( void )
{
   /* Read value from PIHT register */
   uint8_t data = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_PIHT );
   return data;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return EXIT_CLEAN if operation successful. EXIT_ERROR otherwise.
 */
void setProximityIntHighThreshold( uint8_t threshold )
{
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_PIHT, threshold ) ;
    return;
}


/*******************************************************************************
 * @brief Interrupt Enable Controls
 *  ******************************************************************************/


/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getProximityIntEnable( void )
{
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_ENABLE);

    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return EXIT_CLEAN if operation successful. EXIT_ERROR otherwise.
 */
void setProximityIntEnable( uint8_t enable )
{
    /* Read value from ENABLE register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_ENABLE );
    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into ENABLE register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_ENABLE, val);


    return;
}

/*******************************************************************************
 * @brief Clear Interrupts
 ******************************************************************************/

/**
 * @brief Clears the proximity interrupt
 *
 * @returns void
 */
void clearProximityInt( void )
{
    I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_PICLEAR );
    return;
}

/*******************************************************************************
 * @brief LED Drive Strength Controls
 ******************************************************************************/

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t getLEDDrive( void )
{
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONTROL );

    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @returns void
 */
void setLEDDrive(uint8_t drive)
{
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONTROL );

    /* Read value from CONTROL register */

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;

    /* Write register value back into CONTROL register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONTROL, val );
 //   if( EXIT_CLEAN != I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONTROL, val) ) {
 //       return EXIT_ERROR;
 //   }

    return;
}


/**
 * @brief Get the current LED boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t getLEDBoost( void )
{
    /* Read value from CONFIG2 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG2 );
    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @returns void
 */
void setLEDBoost( uint8_t boost )
{
    /* Read value from CONFIG2 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG2 );


    /* Set bits in register to given value */
    boost &= 0b00000011;
    boost = boost << 4;
    val &= 0b11001111;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG2, val );

}


/*******************************************************************************
 * @brief Proxmity Photodiode Select
 ******************************************************************************/
/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t getProxGainCompEnable( void )
{
    /* Read value from CONFIG3 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG3 );


    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0b00000001;

    return val;
}

/*!
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @returns void
 */
 void setProxGainCompEnable(uint8_t enable)
{
    /* Read value from CONFIG3 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG3 );
    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into CONFIG3 register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG3, val );
    return;
}


/*!
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t getProxPhotoMask( void )
{
    /* Read value from CONFIG3 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG3 );

    /* Mask out photodiode enable mask bits */
    val &= 0b00001111;

    return val;
}


void setProxPhotoMask(uint8_t mask)
{
    /* Read value from CONFIG3 register */
    uint8_t val = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_CONFIG3 );

    /* Set bits in register to given value */
    mask &= 0b00001111;
    val &= 0b11110000;
    val |= mask;

    /* Write register value back into CONFIG3 register */
    I2C2_write( APDS9960_I2C_ADDR, APDS9960_CONFIG3, val);


    return;
}


uint8_t apds9960_read_id( void )
{
   uint8_t id = I2C2_read_byte( APDS9960_I2C_ADDR, APDS9960_ID );
   return id;
}



