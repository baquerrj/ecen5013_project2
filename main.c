/**
 * @file    main.c
 *
 * @brief   TIVA TM4C1294XL FreeRTOS Application consisting of three tasks:
 *              1) Temperature Task gets temperature reading from TMP102 external sensor at 1Hz
 *              2) LED task toggles LEDs D1 and D2 at 10Hz
 *              3) Logger Task receives messages from temperature and LED tasks and logs to UART
 *
 * Built off from TIVA uart_echo example and freetos_demo for ek-tm4c123gxl
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"

#include "my_i2c.h"
#include "uart.h"

#include "logger_task.h"
#include "led_task.h"
#include "tmp102_task.h"
#include "alert_task.h"

uint32_t g_sysClock = CLOCK_FREQ;

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


int main( void )
{
    /* Initialize system clock to 120MHz */
    g_sysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), g_sysClock);

    /* Configure peripherals */
    UART0_config( BAUD_115200, g_sysClock );    /* Configure UART0 with Baud Rate 115200 */

    I2C2_init();  /* Configure I2C Bus 2 for use with TMP102 sensor */


    if( 0 != logger_task_init() )
    {
        puts( "ERROR - LOGGER TASK INIT\n" );
        while(1);
    }

    if( 0 != temp_task_init() )
    {
        puts( "ERROR - TEMPERATURE TASK INIT\n" );
        while(1);
    }

    if( 0 != led_task_init() )
    {
        puts( "ERROR - LED TASK INIT\n" );
        while(1);
    }

    if( 0 != alert_task_init() )
    {
        puts( "ERROR - ALERT TASK INIT\n" );
        while(1);
    }

    puts( "SUCCESS - All tasks initialized. Starting scheduler...\n" );

    vTaskStartScheduler();

    puts( "ERROR - Scheduler Exited\n" );
    while(1);

	return 0;
}





















