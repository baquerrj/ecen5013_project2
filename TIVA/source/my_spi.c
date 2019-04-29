#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

#include "my_spi.h"

const uint32_t SPI[4] = {SSI0_BASE, SSI1_BASE, SSI2_BASE, SSI3_BASE};

const uint32_t SPI_SYSCTL[4] = {SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1, SYSCTL_PERIPH_SSI2, SYSCTL_PERIPH_SSI3};


void spi_init( spi_e spi )
{
    if( SPI_0 == spi )
    {
        MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
        GPIOPinConfigure( GPIO_PA2_SSI0CLK );
        GPIOPinConfigure( GPIO_PA3_SSI0FSS );
        GPIOPinConfigure( GPIO_PA4_SSI0XDAT0 );
        GPIOPinConfigure( GPIO_PA5_SSI0XDAT1 );

        //      PA5 - SSI0Tx
        //      PA4 - SSI0Rx
        //      PA3 - SSI0Fss
        //      PA2 - SSI0CLK
        GPIOPinTypeSSI( GPIO_PORTA_BASE, GPIO_PIN_5 |
                                         GPIO_PIN_4 |
                                         GPIO_PIN_3 |
                                         GPIO_PIN_2 );
    }
    else if( SPI_1 == spi )
    {
        MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
        MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );
        GPIOPinConfigure( GPIO_PB5_SSI1CLK );
        GPIOPinConfigure( GPIO_PB4_SSI1FSS );
        GPIOPinConfigure( GPIO_PE4_SSI1XDAT0 );
        GPIOPinConfigure( GPIO_PE5_SSI1XDAT1 );

        //      PE4 - SSI0Tx
        //      PE5 - SSI0Rx
        //      PB4 - SSI0Fss
        //      PB5 - SSI0CLK
        GPIOPinTypeSSI( GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_4 );
        GPIOPinTypeSSI( GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4 );
    }
    else if( SPI_2 == spi )
    {
        MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
        GPIOPinConfigure( GPIO_PD3_SSI2CLK );
        GPIOPinTypeGPIOOutput( GPIO_PORTD_BASE, GPIO_PIN_2 );
        GPIOPinConfigure( GPIO_PD1_SSI2XDAT0 );
        GPIOPinConfigure( GPIO_PD0_SSI2XDAT1 );

        //      PD1 - SSI0Tx
        //      PD0 - SSI0Rx
        //      PD2 - SSI0Fss
        //      PD3 - SSI0CLK
        GPIOPinTypeSSI( GPIO_PORTD_BASE, GPIO_PIN_0 |
                                         GPIO_PIN_1 |
                                         GPIO_PIN_3 );
    }
    SSIEnable( SPI[ spi ] );
    return;
}


void spi_write_packet( spi_e spi, const uint8_t* p, size_t length )
{
    uint8_t i=0;
    while ( i<length )
    {
        spi_flush_rx( spi );
        spi_write_byte( spi, *(p+i) );
        ++i;
    }
    spi_flush_rx( spi );
}

void spi_read_packet( spi_e spi, uint8_t* p, size_t length )
{
    uint8_t i=0;
    while ( i<length )
    {
        spi_write_byte( spi, 0xFF );
        *(p+i) = spi_read_byte( spi );
        ++i;
    }

}
