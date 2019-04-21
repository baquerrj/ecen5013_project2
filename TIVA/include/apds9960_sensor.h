/*!
 * @file	apds9960_sensor.h
 *
 * @brief
 *
 *  Created on: Apr 15, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef APDS9960_SENSOR_H_
#define APDS9960_SENSOR_H_

/*!
 *    @file     apds9960_task.h
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

/**
 * @file    SparkFun_APDS-9960.h
 * @brief   Library for the SparkFun APDS-9960 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright   This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9960 object, call init(), and call the appropriate functions.
 */

#include <stdint.h>


/* Debug */
#define DEBUG                   0

/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR       0x39

/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT   10
#define GESTURE_SENSITIVITY_1   50
#define GESTURE_SENSITIVITY_2   20

/* Error code for returned values */
//#define ERROR                   0xFF

/* Acceptable device IDs */
#define APDS9960_ID_1           0xAB
#define APDS9960_ID_2           0x9C

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9960 register addresses */
#define APDS9960_ENABLE         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL        0x8F
#define APDS9960_CONFIG2        0x90
#define APDS9960_ID             0x92
#define APDS9960_STATUS         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PDATA          0x9C
#define APDS9960_POFFSET_UR     0x9D
#define APDS9960_POFFSET_DL     0x9E
#define APDS9960_CONFIG3        0x9F
#define APDS9960_GPENTH         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF

/* Bit fields */
#define APDS9960_PON            0b00000001
#define APDS9960_AEN            0b00000010
#define APDS9960_PEN            0b00000100
#define APDS9960_WEN            0b00001000
#define APSD9960_AIEN           0b00010000
#define APDS9960_PIEN           0b00100000
#define APDS9960_GEN            0b01000000
#define APDS9960_GVALID         0b00000001

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3

/* Gesture wait time values */
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

/* Direction definitions */
enum {
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */
enum {
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};

/* Container for gesture data */
typedef struct gesture_data_type {
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
    uint8_t index;
    uint8_t total_gestures;
    uint8_t in_threshold;
    uint8_t out_threshold;
} gesture_data_type;



/*!
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
uint8_t apds9960_sensor_init( void );



/*!
 * Function:       apds9960_getMode
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
uint8_t getMode( void );


/*!
 * Function:       apds9960_setMode
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
void setMode( uint8_t mode, uint8_t enable );


/*!
 * @brief
 *
 * @param  <+name+> <+description+>
 * @return <+description+>
 * <+detailed+>
 */
void enablePower( void );

/*!
 * @brief
 *
 * @param  <+name+> <+description+>
 * @return <+description+>
 * <+detailed+>
 */
void disablePower( void );

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @returns proximity data
 */
uint8_t readProximity( void );

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts EXIT_CLEAN to enable hardware external interrupt on proximity
 * @returns void
 */
void enableProximitySensor(uint8_t interrupts);

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @returns void
 */
void disableProximitySensor( void );

/*****************************************************************
 * Gain Control
 *****************************************************************/

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t getAmbientLightGain( void );

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 */
void setAmbientLightGain( uint8_t drive );

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t getGestureGain( void );

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 */
void setGestureGain( uint8_t gain );

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
void setProximityGain( uint8_t drive );


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
uint8_t getProximityGain( void );

/*******************************************************************************
 * @brief Light Interrupt Threshold Controls
 ******************************************************************************/
/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @returns threshold current low threshold stored on the APDS-9960
 */
uint16_t getLightIntLowThreshold( void );

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 */
void setLightIntLowThreshold( uint16_t threshold );

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @returns threshold current low threshold stored on the APDS-9960
 */
uint16_t getLightIntHighThreshold( void );

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 */
void setLightIntHighThreshold( uint16_t threshold );

/*******************************************************************************
 * @brief Proximity Interrupt Threshold Controls
 ******************************************************************************/
/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t getProximityIntLowThreshold( void );

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] low threshold value
 */
void setProximityIntLowThreshold( uint8_t threshold );

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t getProximityIntHighThreshold( void );

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return EXIT_CLEAN if operation successful. EXIT_ERROR otherwise.
 */
void setProximityIntHighThreshold( uint8_t threshold );

/*******************************************************************************
 * @brief Interrupt Enable Controls
 ******************************************************************************/
/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getAmbientLightIntEnable( void );

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 */
void setAmbientLightIntEnable( uint8_t enable );


/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getProximityIntEnable( void );


/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 */
void setProximityIntEnable( uint8_t enable );

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getGestureIntEnable( void );

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 */
void setGestureIntEnable( uint8_t enable );


/*******************************************************************************
 * @brief Clear Interrupts
 ******************************************************************************/
/**
 * @brief Clears the ambient light interrupt
 *
 */
void clearAmbientLightInt( void );

/**
 * @brief Clears the proximity interrupt
 *
 */
void clearProximityInt( void );

/*******************************************************************************
 * @brief Ambient Light Controls
 ******************************************************************************/
/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 */
uint16_t readAmbientLight( void );

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 */
uint16_t readRedLight( void );

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 */
uint16_t readGreenLight( void );

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 */
uint16_t readBlueLight( void );
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
uint8_t getLEDDrive( void );


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
void setLEDDrive(uint8_t drive);

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
uint8_t getLEDBoost( void );

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
void setLEDBoost( uint8_t boost );

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t getProxGainCompEnable( void );

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
 uint8_t getProxPhotoMask( void );


/*!
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 */
void setProxPhotoMask(uint8_t mask);

/*!
 * Function:       apds9960_read_id
 * @brief   Read APDS9960 ID Register
 *
 * @returns sensor id
 */
uint8_t apds9960_read_id( void );

/*!
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @returns void
 */
void setProxGainCompEnable(uint8_t enable);

/*!
 * @brief Gets the entry proximity threshold for gesture sensing
 */
uint8_t getGestureEnterThresh( void );

/*!
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in]  threshold proximity value needed to start gesture mode
 */
void setGestureEnterThresh( uint8_t threshold );

/*!
 * @brief Gets the exit proximity threshold for gesture sensing
 */
uint8_t getGestureExitThresh( void );

/*!
 * @brief Sets the exit proximity threshold for gesture sensing
 */
void setGestureExitThresh( uint8_t threshold );

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t getGestureWaitTime( void );

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
void setGestureWaitTime( uint8_t time );

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
void setGestureLEDDrive( uint8_t drive );


#endif /* APDS9960_SENSOR_H_ */
