/*!
 * @file    my_i2c.c
 *
 * @brief
 *
 *  reference: https://www.digikey.com/eewiki/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
 *
 *
 *  Created on: Apr 8, 2019
 *      Author: Roberto Baquerizo
 */


#include "my_i2c.h"

void I2C0_init( void )
{
    /* Enable I2C Bus 0 */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C0 );

    /* Reset I2C Bus 0 */
    SysCtlPeripheralReset( SYSCTL_PERIPH_I2C0 );

    /* Enable GPIO Peripheral containing I2C0 (PB2 and PB3) */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );

    /* Configure the pin muxing for I2C2 functions on port N4 and N5 */
    GPIOPinConfigure( GPIO_PB3_I2C0SDA );
    GPIOPinConfigure( GPIO_PB2_I2C0SCL );

    /* Select the I2C function for these pins */
    GPIOPinTypeI2CSCL( GPIO_PORTB_BASE, GPIO_PIN_2 );
    GPIOPinTypeI2C(    GPIO_PORTB_BASE, GPIO_PIN_3 );


    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
    return;
}

void I2C2_init( void )
{
    /* Enable I2C Bus 2 */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C2 );

    /* Reset I2C Bus 2 */
    SysCtlPeripheralReset( SYSCTL_PERIPH_I2C2 );

    /* Enable GPIO Peripheral containing I2C2 (PN5 and PN4) */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPION );

    /* Configure the pin muxing for I2C2 functions on port N4 and N5 */
    GPIOPinConfigure( GPIO_PN4_I2C2SDA );
    GPIOPinConfigure( GPIO_PN5_I2C2SCL );

    /* Select the I2C function for these pins */
    GPIOPinTypeI2CSCL( GPIO_PORTN_BASE, GPIO_PIN_5 );
    GPIOPinTypeI2C(    GPIO_PORTN_BASE, GPIO_PIN_4 );


    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
    return;
}

uint8_t I2C0_read_byte(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    uint8_t data = I2CMasterDataGet(I2C0_BASE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    return data;
}

uint8_t I2C2_read_byte(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C2_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    //return data pulled from the specified register
    uint8_t data = I2CMasterDataGet(I2C2_BASE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    return data;
}

uint16_t I2C0_read_word( uint32_t slave_addr, uint8_t reg )
{
    uint8_t buff[2] = {0,0};

    I2CMasterSlaveAddrSet( I2C0_BASE, slave_addr, false );

    I2CMasterDataPut( I2C0_BASE, reg );

    I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START );

    while( I2CMasterBusy(I2C0_BASE) );

    I2CMasterSlaveAddrSet( I2C0_BASE, slave_addr, true );

    I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START );

    while( I2CMasterBusy(I2C0_BASE) );

    buff[0] = (uint8_t)I2CMasterDataGet( I2C0_BASE );

    I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH );

    while( I2CMasterBusy(I2C0_BASE) );

    buff[1] = (uint8_t)I2CMasterDataGet( I2C0_BASE );

    uint16_t data = (buff[0] << 8) | buff[1];
    return data;
}

uint16_t I2C2_read_word( uint32_t slave_addr, uint8_t reg )
{
    uint8_t buff[2] = {0,0};

    I2CMasterSlaveAddrSet( I2C2_BASE, slave_addr, false );

    I2CMasterDataPut( I2C2_BASE, reg );

    I2CMasterControl( I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START );

    while( I2CMasterBusy(I2C2_BASE) );

    I2CMasterSlaveAddrSet( I2C2_BASE, slave_addr, true );

    I2CMasterControl( I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START );

    while( I2CMasterBusy(I2C2_BASE) );

    buff[0] = (uint8_t)I2CMasterDataGet( I2C2_BASE );

    I2CMasterControl( I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH );

    while( I2CMasterBusy(I2C2_BASE) );

    buff[1] = (uint8_t)I2CMasterDataGet( I2C2_BASE );

    uint16_t data = (buff[0] << 8) | buff[1];
    return data;
}

void I2C0_write( uint32_t slave_addr, uint8_t reg, uint8_t data )
{
    I2CMasterSlaveAddrSet( I2C0_BASE, slave_addr, false );
    I2CMasterDataPut( I2C0_BASE, reg );
    I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START );
    while( I2CMasterBusy( I2C0_BASE ) );

    I2CMasterDataPut( I2C0_BASE, data );
    I2CMasterControl( I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );

    while( I2CMasterBusy( I2C0_BASE ) );
    return;
}


void I2C2_write( uint32_t slave_addr, uint8_t reg, uint8_t data )
{
    I2CMasterSlaveAddrSet( I2C2_BASE, slave_addr, false );
    I2CMasterDataPut( I2C2_BASE, reg );
    I2CMasterControl( I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START );
    while( I2CMasterBusy( I2C2_BASE ) );

    I2CMasterDataPut( I2C2_BASE, data );
    I2CMasterControl( I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );

    while( I2CMasterBusy( I2C2_BASE ) );
    return;
}

