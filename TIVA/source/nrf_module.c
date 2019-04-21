/*
 *  ======= nordic_module ========
 *  nrf_module implementation
 *
 *  Created on: Apr 20, 2019
 *  Author:     rober
 */


#include "nrf_module.h"

#include "my_spi.h"
#include "nRF240L.h"
#include "delay.h"

#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"

volatile uint8_t tx_configured = 0;
volatile uint8_t rx_configured = 0;

volatile uint8_t transmitted = 0;
volatile uint8_t received = 0;
volatile uint8_t retry_error = 0;

static uint8_t interrupts_enabled = 0;

extern uint32_t g_sysClock;

void nordic_interrupt_handler( void );

static interrupt_handler_t user_handler;


int8_t nrf_module_init( uint8_t enable_interrupts, interrupt_handler_t handler )
{
    spi_clock_init( SPI_1 );
    spi_init( SPI_1 );
    delayMs( 1 );

    MAP_SysCtlPeripheralEnable( NORDIC_CE_SYSCTL_PORT );
    GPIOPinTypeGPIOOutput( NORDIC_CE_PORT, NORDIC_CE_PIN );
    GPIOPinWrite( NORDIC_CE_PORT, NORDIC_CE_PIN, 0 );

    MAP_SysCtlPeripheralEnable( NORDIC_CSN_SYSCTL_PORT );
    GPIOPinTypeGPIOOutput( NORDIC_CSN_PORT, NORDIC_CSN_PIN );
    GPIOPinWrite( NORDIC_CSN_PORT, NORDIC_CSN_PIN, NORDIC_CSN_PIN );

    MAP_SysCtlPeripheralEnable( NORDIC_IRQ_SYSCTL_PORT );

    GPIOIntDisable( NORDIC_IRQ_PORT,0xFFFF );
    GPIOPinTypeGPIOInput( NORDIC_IRQ_PORT,NORDIC_IRQ_PIN );
    GPIOIntUnregister( NORDIC_IRQ_PORT );
    GPIOIntClear( NORDIC_IRQ_PORT,0xFFFF );
    GPIOIntTypeSet( NORDIC_IRQ_PORT, NORDIC_IRQ_PIN, GPIO_LOW_LEVEL );

    if( enable_interrupts )
    {
        interrupts_enabled = 1;
        user_handler = handler;
        GPIOIntRegister( NORDIC_IRQ_PORT, nordic_interrupt_handler );
        GPIOIntDisable( NORDIC_IRQ_PORT,0xFFFF );
        GPIOIntEnable( NORDIC_IRQ_PORT,NORDIC_IRQ_PIN );
    }
    else
    {
        interrupts_enabled = 0;
    }
    return 0;
}

void nrf_module_setup( nrf_data_rate_e data_rate, nrf_power_e power )
{
    //Clearing all interrupts
    nrf_write_status( 0 );
    //Disabling all interrupts and init in power down tx mode
    nrf_write_config( 0x78 );
    nrf_write_rf_ch( 44 );
    nrf_write_rf_setup(( power<<1) | (data_rate<<3) | 1 );
    nrf_write_register( 0x03, 0x03 );
    delayMs( 1 );
}

void nrf_module_disable( void )
{
    interrupts_enabled = 0;
    uint8_t config = nrf_read_config();
    nrf_write_config( config & ~NORDIC_CONFIG_PWR_UP( 1 )  );
    spi_disable( SPI_1 );
    GPIOIntClear( NORDIC_IRQ_PORT,NORDIC_IRQ_PIN );
    GPIOIntUnregister( NORDIC_IRQ_PORT );
}

uint8_t nrf_read_register( uint8_t reg )
{
    uint8_t readValue = 0;

    //CSN High to low for new command
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,reg );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    spi_write_byte( SPI_1,0xFF );
    readValue = spi_read_byte( SPI_1 );

    //Marking the end of transaction by CSN high
    nrf_chip_disable();

    return readValue;
}


void nrf_write_register( uint8_t reg, uint8_t value )
{
    //CSN High to low for new command
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,reg | 0x20 );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    spi_write_byte( SPI_1,value );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO

    //Marking the end of transaction by CSN high
    nrf_chip_disable();
}

void nrf_write_status( uint8_t statusValue )
{
    nrf_write_register( NORDIC_STATUS_REG, statusValue );
}

uint8_t nrf_read_status( void )
{
    uint8_t readValue = 0;

    //CSN High to low for new command
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,NORDIC_NOP );
    readValue = spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO

    //Marking the end of transaction by CSN high
    nrf_chip_disable();

    return readValue;
}

void nrf_write_config( uint8_t configValue )
{
    nrf_write_register( NORDIC_CONFIG_REG, configValue );
}

uint8_t nrf_read_config( void )
{
    return nrf_read_register( NORDIC_CONFIG_REG );
}

uint8_t nrf_read_rf_setup( void )
{
    return nrf_read_register( NORDIC_RF_SETUP_REG );
}

void nrf_write_rf_setup( uint8_t rfSetupValue )
{
    nrf_write_register( NORDIC_RF_SETUP_REG, rfSetupValue );
}

uint8_t nrf_read_rf_ch( void )
{
    return nrf_read_register( NORDIC_RF_CH_REG );
}

void nrf_write_rf_ch( uint8_t channel )
{
    nrf_write_register( NORDIC_RF_CH_REG, channel );
}

void nrf_write_En_AA( uint8_t data )
{
    nrf_write_register( NORDIC_EN_AA_REG, data );
}

uint8_t nrf_read_En_AA( void )
{
    return nrf_read_register( NORDIC_EN_AA_REG );
}

void nrf_write_setup_retry( uint8_t data )
{
    nrf_write_register( NODIC_SETUP_RETR_REG, data );
}

uint8_t nrf_read_setup_retry( void )
{
    return nrf_read_register( NODIC_SETUP_RETR_REG );
}

void nrf_read_tx_addr( uint8_t *address )
{
    uint8_t i = 0;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,NORDIC_TX_ADDR_REG );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    while( i < NORDIC_TX_ADDR_LEN )
    {
        spi_write_byte( SPI_1, 0xFF );    //Dummy to get the data
        *( address+i) = spi_read_byte(SPI_1 );
        i++;
    }

    nrf_chip_disable();
}

void nrf_write_tx_addr( uint8_t * tx_addr )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,NORDIC_TX_ADDR_REG | 0x20 );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    spi_write_packet( SPI_1,tx_addr,NORDIC_TX_ADDR_LEN );
    spi_flush_rx( SPI_1 );

    nrf_chip_disable();
}

void nrf_read_rx_pipe_addr( uint8_t pipe_num, uint8_t *address )
{
    if( pipe_num > 5 )
        return;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,(NORDIC_RX_ADDR_P0_REG + pipe_num) );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    size_t ADDR_LEN = NORDIC_TX_ADDR_LEN;
    pipe_num > 2 ? ADDR_LEN = 1: 0;
    spi_read_packet( SPI_1, address, ADDR_LEN );

    nrf_chip_disable();
}

void nrf_write_rx_pipe_addr( uint8_t pipe_num, uint8_t *rx_addr )
{
    if( pipe_num > 5 )
        return;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1,(NORDIC_RX_ADDR_P0_REG + pipe_num) | 0x20 );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    size_t ADDR_LEN = NORDIC_TX_ADDR_LEN;
    pipe_num > 1 ? ADDR_LEN = 1: 0;
    spi_write_packet( SPI_1,rx_addr,ADDR_LEN );
    spi_flush_rx( SPI_1 );

    nrf_chip_disable();
}


uint8_t nrf_read_fifo_status(  void  )
{
    return nrf_read_register( NORDIC_FIFO_STATUS_REG );
}

void nrf_flush_tx_fifo( void )
{
    nrf_write_command( NORDIC_TXFIFO_FLUSH_CMD );
}

void nrf_flush_rx_fifo( void )
{
    nrf_write_command( NORDIC_RXFIFO_FLUSH_CMD );
}

void nrf_enable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NORDIC_EN_RXADDR_REG );
    nrf_write_register( NORDIC_EN_RXADDR_REG, ret | (1<<rx_pipe_number)  );

}
void nrf_disable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NORDIC_EN_RXADDR_REG );
    nrf_write_register( NORDIC_EN_RXADDR_REG, ret & (~(1<<rx_pipe_number))  );
}

static void nrf_mode_configure( nrf_mode_e mode, uint8_t rx_pipe_number, uint8_t addr[5], uint8_t payload_size )
{
    if( mode < 2 )
    {
        nrf_radio_disable();
        uint8_t configureRead = nrf_read_config();

        if( mode == NRF_MODE_TX )
        {
            tx_configured = 1;
            configureRead &= ~( NORDIC_CONFIG_TX_DS_INT( 1 ) );// | NORDIC_CONFIG_MAX_RT_INT(1)  );
            nrf_flush_tx_fifo();
            nrf_write_En_AA( 0 );
            nrf_write_setup_retry( 0 );
            nrf_write_tx_addr( addr );
            nrf_write_rx_pipe_addr( rx_pipe_number, addr );
            nrf_enable_rx_pipe( rx_pipe_number );
            nrf_write_register(( NORDIC_RX_PW_P0_REG), payload_size );
            nrf_write_config( configureRead | NORDIC_CONFIG_PWR_UP( 1 )  );
            delayMs( 2 );
        }
        else
        {
            rx_configured = 1;
            configureRead |= NORDIC_CONFIG_PWR_UP( 1 ) | NORDIC_CONFIG_PRIM_RX( 1 );
            configureRead &= ~( NORDIC_CONFIG_RX_DR_INT( 1 ) );
            nrf_flush_rx_fifo();
            nrf_enable_rx_pipe( rx_pipe_number );
            nrf_write_rx_pipe_addr( rx_pipe_number, addr );
            nrf_write_register( (NORDIC_RX_PW_P0_REG + rx_pipe_number), payload_size );
            nrf_write_config( configureRead );
            nrf_radio_enable();
        }

        delayMs( 2 );
        printf("NORDIC Configured in %s mode\n", (( mode)?"RX MODE":"tx MODE") );

    }
    else
    {
        printf("INVALID MODE\n");
    }
}

void nrf_open_read_pipe( uint8_t rx_pipe_number, uint8_t rx_addr[5], uint8_t payload_size )
{
    nrf_mode_configure( NRF_MODE_RX, rx_pipe_number, rx_addr, payload_size );
}

void nrf_open_write_pipe( uint8_t tx_addr[5] )
{
    nrf_mode_configure( NRF_MODE_TX, 0, tx_addr, 5 );
}

void nrf_close_write_pipe( void )
{
    tx_configured = 0;
    uint8_t configureRead = nrf_read_config();
    configureRead |= ( NORDIC_CONFIG_TX_DS_INT( 1 ) | NORDIC_CONFIG_MAX_RT_INT( 1 ) );
    nrf_write_config( configureRead );
    nrf_disable_rx_pipe( 0 );
}

void nrf_close_read_pipe( uint8_t rx_pipe_number )
{
    nrf_radio_disable();
    rx_configured = 0;
    uint8_t configureRead = nrf_read_config();
    configureRead |= NORDIC_CONFIG_RX_DR_INT( 1 );
    nrf_write_config( configureRead );
    nrf_disable_rx_pipe( rx_pipe_number );
}

void nrf_write_tx_payload( uint8_t *data, uint8_t len )
{
    nrf_chip_disable();
    nrf_chip_enable();
    spi_write_byte( SPI_1, NORDIC_W_TXPAYLD_CMD );
    spi_read_byte( SPI_1 ); //used to clear the previously value in the RX FIFO

    spi_write_packet( SPI_1,data, len );  //loading the FIFO with data  before enabling the CE pin
    spi_flush_rx( SPI_1 );
    nrf_chip_disable();
}

void nrf_tx_pulse( void )
{
    nrf_radio_enable();
    delayUs( 20 );
    nrf_radio_disable();
}

uint8_t nrf_transmit_data( uint8_t *data, uint8_t len, uint8_t toRXMode )
{
    if(tx_configured)
    {
        uint8_t configureRead = nrf_read_config();
        configureRead &= ~NORDIC_CONFIG_PRIM_RX( 1 );
        nrf_write_config( configureRead );
        configureRead = nrf_read_config();
        delayUs( 130 );

        nrf_radio_disable();

        nrf_write_tx_payload( data, len );

        nrf_tx_pulse();

        printf("Data written");

        if( interrupts_enabled )
        {
            while( transmitted == 0 && retry_error == 0 );    //wait till tx data is transmitted from FIFO
            if( retry_error )
            {
                retry_error = 0;
                printf("Data Retry Error\n");
            }
            else
            {
                transmitted = 0; printf("Data Transmitted\n");
            }
        }
        else
        {
            uint8_t status = 0;
            while( !(( NORDIC_STATUS_TX_DS_MASK | NORDIC_STATUS_MAX_RT_MASK) & status) )
            {
                status = nrf_read_status();
            }
            nrf_write_status( NORDIC_STATUS_TX_DS_MASK | NORDIC_STATUS_MAX_RT_MASK | NORDIC_STATUS_MAX_RT_MASK );
        }

        if( toRXMode )
        {
            configureRead &= ~( NORDIC_CONFIG_PRIM_RX( 1 ) );
            nrf_write_config( configureRead );
            nrf_flush_rx_fifo();
            nrf_radio_enable();
        }

    }
    else
    {
        printf("tx mode not configured");
    }
    return 0;
}

void nrf_read_rx_payload( uint8_t *data, uint8_t len )
{
    nrf_chip_enable();

    spi_write_byte( SPI_1, NORDIC_R_RXPAYLD_CMD );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    spi_read_packet( SPI_1,data,len );
    spi_flush( SPI_1 );

    nrf_chip_disable();
}

uint8_t nrf_read_data( uint8_t *data, uint8_t len )
{
    if( rx_configured )
    {
        nrf_radio_enable();
        uint8_t val = nrf_read_fifo_status();
        val = nrf_read_config();
        //Status reg if data is available
        if( interrupts_enabled )
        {
            while(received == 0)    //wait till RX data in FIFO
            {
                val = nrf_read_fifo_status();
            }
            received = 0;
        }
        else
        {
            uint8_t status = 0;
            while( !( NORDIC_STATUS_RX_DR_MASK & status ) )
            {
                status = nrf_read_status();
            }
        }
        printf("Data received");

        nrf_read_rx_payload( data, len );

        printf("Data read");
    }
    else
    {
        printf("RX mode not configured");
    }
    return 0;
}


void nrf_init_test( void )
{
    nrf_module_init( 0, nordic_interrupt_handler );
    nrf_module_setup( NRF_DR_1Mbps, NRF_POWER_LOW );
    delayMs( 100 );

    printf( "SPI Initialized\n");
    printf("Nordic Initialized\n");
    printf("Nordic Test\n");
    nrf_write_status( 0 );
    uint8_t sendValue = 0x08;
    uint8_t readValue = 0;
    nrf_write_config( sendValue );
    readValue = nrf_read_config();
    printf("Recv: 0x%x\n",readValue);
    if( readValue == sendValue )
    {
        printf("Write/Read Config Value Matched\n");
        printf("Sent: 0x%x\n",sendValue);
        printf("Recv: 0x%x\n",readValue);
    }

    delayMs( 5 );

    nrf_write_register( NORDIC_STATUS_REG,0 );
    sendValue = 44;
    nrf_write_rf_ch( sendValue );
    readValue = nrf_read_rf_ch();
    if( readValue == sendValue )
    {
        printf("Write/Read RF CH Value Matched\n");
        printf("Sent: 0x%x\n",sendValue);
        printf("Recv: 0x%x\n",readValue);
    }

    sendValue = 0x07 ;
    nrf_write_rf_setup( sendValue );
    readValue = nrf_read_rf_setup();
    if( readValue == sendValue )
    {
        printf("Write/Read RF Setup Value Matched\n");
        printf("Sent: 0x%x\n",sendValue);
        printf("Recv: 0x%x\n",readValue);
    }

//    nrf_write_register( 0x03, 3 );

//    uint8_t sendAddr[5] = {0xBA,0x56,0xBA,0x56,0xBA};
//    uint8_t sendAddr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
//    printf("tx ADDRESSES SET: 0x%x%x%x%x%x\n",sendAddr[0],sendAddr[1],sendAddr[2],sendAddr[3],sendAddr[4]);
//    nrf_write_TX_ADDR( sendAddr );
//    uint8_t readAddr[5];
//    nrf_read_TX_ADDR( readAddr );
//    printf("tx ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);
//
//    nrf_read_rx_p0_addr( readAddr );
//    printf("RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);
//
//    nrf_write_rx_p0_addr( sendAddr );
//    nrf_read_rx_p0_addr( readAddr );
//    printf("RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);




//    nrf_Mode_t mode = NRF_MODE_RX;
//    printf("Configuring nrf in %d mode",mode);
//    nrf_mode_configure( mode );
//    uint8_t Data[2] = {0};
//    nrf_read_data( Data,2 );
//    printf("Nordic Data Recvd: 0x%x, 0x%x", Data[0],Data[1]);

    uint8_t sendAddr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    nrf_open_write_pipe( sendAddr );
    printf("Configuring nrf in tx mode");
    uint8_t readAddr[5];
    nrf_read_tx_addr( readAddr );
    printf( " tx ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);

    printf( " RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);

    nrf_read_rx_pipe_addr( 0, readAddr );
    printf( " RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);

    uint8_t Data[5] = {0x55,0xBB,0xBB,0xBB,0xBB};
    nrf_transmit_data( Data,5, false );
    printf("Nordic Data Sent: 0x%x, 0x%x", Data[0],Data[1]);

    printf("Nordic Test End\n");

    nrf_module_disable();
}

void nordic_interrupt_handler( void )
{
    MAP_IntMasterDisable();
        uint32_t int_status = GPIOIntStatus( NORDIC_IRQ_PORT, false );
        if( int_status & NORDIC_IRQ_PIN )
        {
            GPIOIntClear( NORDIC_IRQ_PORT, NORDIC_IRQ_PIN );
            uint8_t nrf_int_reason = nrf_read_status();
            if( nrf_int_reason & NORDIC_STATUS_TX_DS_MASK )
            {
                nrf_write_status( nrf_int_reason | NORDIC_STATUS_TX_DS_MASK );
                transmitted = 1;
            }
            if( nrf_int_reason & NORDIC_STATUS_RX_DR_MASK )
            {
                nrf_write_status( nrf_int_reason | NORDIC_STATUS_RX_DR_MASK );
                nrf_flush_rx_fifo();
                user_handler();
                received = 1;
                printf("nrf RX Complete\n");
            }
            if( nrf_int_reason & NORDIC_STATUS_MAX_RT_MASK )
            {
                nrf_write_status( nrf_int_reason | NORDIC_STATUS_MAX_RT_MASK );
                nrf_flush_tx_fifo();
                user_handler();
                retry_error = 1;
            }
        }
        MAP_IntMasterEnable();
}
