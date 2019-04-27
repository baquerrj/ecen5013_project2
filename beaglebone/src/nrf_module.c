/*!
 * @file  nrf_module.c
 * @brief
 *
 * <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/21/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#include "nrf_module.h"


/*! Color coding for NRF pins
 *
 * VDD  Red
 * GND  Black
 * CSN  Brown
 * CE   Purple
 * MOSI Orange
 * SCK  Green
 * IRQ  Gray
 * MISO Yellow
 */


volatile uint8_t tx_configured = 0;
volatile uint8_t rx_configured = 0;

volatile uint8_t transmitted = 0;
volatile uint8_t received = 0;
volatile uint8_t retry_error = 0;

static uint8_t interrupts_enabled = 0;

void nordic_interrupt_handler( void *args );

static interrupt_handler_t user_handler;

#define NORDIC_CE_PIN_MRAA      (73)    // purple
#define NORDIC_CSN_PIN_MRAA     (71)    // brown
#define NORDIC_IRQ_PIN_MRAA     (69)    // gray

mraa_gpio_context NRF_CSN_GPIO = 0;
mraa_gpio_context NRF_CE_GPIO = 0;
static mraa_gpio_context NRF_IRQ_GPIO = 0;

uint8_t pipe0_reading_address = 0;

int8_t nrf_gpio_init( void )
{
    if( SPI_0 != spi_init( SPI_0 ) )
    {
        return -1;
    }
    delayMs( 1 );

    NRF_CSN_GPIO = mraa_gpio_init( NORDIC_CSN_PIN_MRAA );
    if( NULL == NRF_CSN_GPIO )
    {
        return -1;
    }

    NRF_CE_GPIO = mraa_gpio_init( NORDIC_CE_PIN_MRAA );
    if( NULL == NRF_CE_GPIO )
    {
        mraa_gpio_close( NRF_CSN_GPIO );
        return -1;
    }

    NRF_IRQ_GPIO = mraa_gpio_init( NORDIC_IRQ_PIN_MRAA );
    if( NULL == NRF_IRQ_GPIO )
    {
        mraa_gpio_close( NRF_CSN_GPIO );
        mraa_gpio_close( NRF_CE_GPIO );
        return -1;
    }

    mraa_result_t retVal;
    retVal = mraa_gpio_dir( NRF_CSN_GPIO, MRAA_GPIO_OUT );
    if( retVal != MRAA_SUCCESS )
    {
        mraa_gpio_close( NRF_IRQ_GPIO );
        mraa_gpio_close( NRF_CE_GPIO );
        mraa_gpio_close( NRF_CSN_GPIO );
        return -1;
    }

    retVal = mraa_gpio_dir( NRF_CE_GPIO, MRAA_GPIO_OUT );
    if( retVal != MRAA_SUCCESS )
    {
        mraa_gpio_close( NRF_IRQ_GPIO );
        mraa_gpio_close( NRF_CE_GPIO );
        mraa_gpio_close( NRF_CSN_GPIO );
        return -1;
    }

    retVal = mraa_gpio_dir( NRF_IRQ_GPIO, MRAA_GPIO_IN );
    if( retVal != MRAA_SUCCESS )
    {
        mraa_gpio_close( NRF_IRQ_GPIO );
        mraa_gpio_close( NRF_CE_GPIO );
        mraa_gpio_close( NRF_CSN_GPIO );
        return -1;
    }
    return 1;
}




uint8_t nrf_set_datarate( nrf_data_rate_e speed )
{
    uint8_t result = 0;
    uint8_t setup = nrf_read_register( NORDIC_REG_RF_SETUP );

    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH) );

    if( NRF_DR_2MBPS == speed )
    {
        setup |= _BV( RF_DR_HIGH );
    }

    nrf_write_register( NORDIC_REG_RF_SETUP, setup );

    uint8_t check = nrf_read_register( NORDIC_REG_RF_SETUP );
    if( setup == check )
    {
        result = 1;
    }
    else
    {
        printf( "ERROR - READ 0%x - EXPECTED 0%x\n", check, setup );
    }
    return result;
}

void nrf_power_up( void )
{
    uint8_t cfg = nrf_read_register( NORDIC_REG_CONFIG );
    if( !(cfg & _BV(NORDIC_MASK_PWR_UP)) )
    {
        nrf_write_register( NORDIC_REG_CONFIG, cfg | _BV(NORDIC_MASK_PWR_UP) );
        delayMs( 5 );
    }
    return;
}


void nrf_power_off( void )
{
    interrupts_enabled = 0;
    uint8_t config = nrf_read_config();
    nrf_write_config( config & ~(NORDIC_CONFIG_PWR_UP( 1 )) );
    return;
}


void nrf_set_retries( uint8_t delay, uint8_t count )
{
    nrf_write_register( NORDIC_REG_SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) << ARC );
    return;
}

uint8_t nrf_get_retries( void )
{
    return nrf_read_register( NORDIC_REG_SETUP_RETR );
}

void nrf_set_crc_length( nrf_crc_e length )
{
    uint8_t config = nrf_read_register( NORDIC_REG_CONFIG ) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

    // switch uses RAM (evil!)
    if ( length == NRF_CRC_DISABLED )
    {
        // Do nothing, we turned it off above.
    }
    else if ( length == NRF_CRC_8 )
    {
        config |= _BV(EN_CRC);
    }
    else
    {
        config |= _BV(EN_CRC);
        config |= _BV( CRCO );
    }
    nrf_write_register( NORDIC_REG_CONFIG, config ) ;
}


nrf_crc_e nrf_get_crc_length( void )
{
    nrf_crc_e result = NRF_CRC_DISABLED;
    uint8_t config = nrf_read_register( NORDIC_REG_CONFIG ) & ( _BV(CRCO) | _BV(EN_CRC)) ;

    if ( config & _BV(EN_CRC ) )
    {
        if ( config & _BV(CRCO) )
        {
            result = NRF_CRC_16;
        }
        else
        {
            result = NRF_CRC_8;
        }
    }
    return result;
}

void nrf_set_palevel( nrf_pa_level_e level )
{
    uint8_t setup = nrf_read_register( NORDIC_REG_RF_SETUP ) ;
    setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

    // switch uses RAM (evil!)
    if ( level == NRF_PA_MAX )
    {
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
    }
    else if ( level == NRF_PA_HIGH )
    {
        setup |= _BV(RF_PWR_HIGH) ;
    }
    else if ( level == NRF_PA_LOW )
    {
        setup |= _BV(RF_PWR_LOW);
    }
    else if ( level == NRF_PA_MIN )
    {
        // nothing
    }
    else if ( level == NRF_PA_ERROR )
    {
        // On error, go to maximum PA
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
    }

    nrf_write_register( NORDIC_REG_RF_SETUP, setup ) ;
}

nrf_pa_level_e nrf_get_palevel( void )
{
    nrf_pa_level_e result = NRF_PA_ERROR ;
    uint8_t power = nrf_read_register(NORDIC_REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

    // switch uses RAM (evil!)
    if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
    {
        result = NRF_PA_MAX ;
    }
    else if ( power == _BV(RF_PWR_HIGH) )
    {
        result = NRF_PA_HIGH ;
    }
    else if ( power == _BV(RF_PWR_LOW) )
    {
        result = NRF_PA_LOW ;
    }
    else
    {
        result = NRF_PA_MIN ;
    }
    return result ;
}

void print_status(uint8_t status)
{
  printf( "STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n" ,
           status,
           (status & _BV(NORDIC_MASK_RX_DR))?1:0,
           (status & _BV(NORDIC_MASK_TX_DS))?1:0,
           (status & _BV(NORDIC_MASK_MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0x07),
           (status & _BV(TX_FULL))?1:0
          );
}

void nrf_start_listening( void )
{
    uint8_t cfg = nrf_read_register( NORDIC_REG_CONFIG );
    nrf_write_register( NORDIC_REG_CONFIG, _BV(PWR_UP)  |
                                           _BV(PRIM_RX) |
                                           cfg );
    nrf_write_register( NORDIC_REG_STATUS, _BV(RX_DR) |
                                           _BV(TX_DS) |
                                           _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if( pipe0_reading_address )
  {
      nrf_write_packet( NORDIC_REG_RX_ADDR_P0, (uint8_t*)(&pipe0_reading_address), 5 );
  }

  // Flush buffers
  nrf_flush_rx();
  nrf_flush_tx();

  // Go!
  nrf_radio_enable();

  // wait for the radio to come up (130us actually only needed)
  delayUs(130);
}


void nrf_stop_listening( void )
{
    nrf_radio_disable();
    nrf_flush_tx();
    nrf_flush_rx();
    return;
}

uint8_t nrf_module_init( void )
{
    uint8_t setup = 0;

    if( SPI_0 != spi_init( SPI_0 ) )
    {
        return -1;
    }

    setup = nrf_gpio_init();

    delayMs( 5 );


    nrf_write_register( NORDIC_REG_SETUP_RETR, (0b0101 << ARD) | (0b1111 << ARC));
//    nrf_write_register( NORDIC_REG_CONFIG, 0x0C );

    nrf_set_palevel( NRF_PA_MAX );

    nrf_set_datarate( NRF_DR_1MBPS );

    nrf_set_crc_length( NRF_CRC_16 );

    nrf_write_register( NORDIC_REG_DYNPD, 0 );
    nrf_write_register( NORDIC_REG_STATUS, _BV(NORDIC_MASK_STATUS_RX_DR) |
                                           _BV(NORDIC_MASK_STATUS_TX_DS) |
                                           _BV(NORDIC_MASK_STATUS_MAX_RT) );

    nrf_set_channel(76);

    nrf_flush_rx();
    nrf_flush_tx();

    nrf_power_up();

 //   nrf_write_register( NORDIC_REG_CONFIG, (nrf_read_register(NORDIC_REG_CONFIG)) & ~_BV(NORDIC_MASK_CONFIG_PRIM_RX) );

   // print_status( nrf_read_status() );

//    uint8_t read = nrf_read_register( NORDIC_REG_CONFIG );
//    printf( "CONFIG READ 0x%x\n", read );
//    read = nrf_read_register( NORDIC_REG_RF_SETUP );
//    printf( "RF SETUP READ 0x%x\n", read );
//    read = nrf_read_register( NORDIC_REG_RF_CH );
//    printf( "RF CH READ 0x%x\n", read );
    return ( setup != 0 && setup != 0xFF );
}

int8_t nrf_setup_interrupts( interrupt_handler_t handler )
{
    interrupts_enabled = 1;
    user_handler = handler;
    uint8_t retVal = mraa_gpio_isr( NRF_IRQ_GPIO, MRAA_GPIO_EDGE_FALLING,
            &nordic_interrupt_handler, NULL );
    if( MRAA_SUCCESS != retVal )
    {
        mraa_gpio_close( NRF_IRQ_GPIO );
        mraa_gpio_close( NRF_CE_GPIO );
        mraa_gpio_close( NRF_CSN_GPIO );
        interrupts_enabled = 0;
        return -1;
    }
    return 1;
}

void nrf_module_setup( nrf_data_rate_e data_rate, nrf_power_e power )
{
    //Clearing all interrupts
    nrf_write_status( 0 );
    nrf_write_config( 0x78 );
    nrf_set_channel( 44 );
    nrf_write_rf_setup(( power<<1) | (data_rate<<3) | 1 );
    nrf_write_register( 0x03, 0x03 );
    delayMs( 1 );
}


void nrf_write_status( uint8_t statusValue )
{
    nrf_write_register( NORDIC_REG_STATUS, statusValue );
}

uint8_t nrf_read_status( void )
{
    uint8_t readValue = 0;

    //CSN High to low for new command
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_0, NORDIC_NOP );
    readValue = spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO

    //Marking the end of transaction by CSN high
    nrf_chip_disable();

    return readValue;
}

void nrf_write_config( uint8_t configValue )
{
    nrf_write_register( NORDIC_REG_CONFIG, configValue );
}

uint8_t nrf_read_config( void )
{
    return nrf_read_register( NORDIC_REG_CONFIG );
}

uint8_t nrf_read_rf_setup( void )
{
    return nrf_read_register( NORDIC_REG_RF_SETUP );
}

void nrf_write_rf_setup( uint8_t rfSetupValue )
{
    nrf_write_register( NORDIC_REG_RF_SETUP, rfSetupValue );
}

uint8_t nrf_get_channel( void )
{
    return nrf_read_register( NORDIC_REG_RF_CH );
}

void nrf_set_channel( uint8_t channel )
{
    nrf_write_register( NORDIC_REG_RF_CH, channel );
}

void nrf_write_En_AA( uint8_t data )
{
    nrf_write_register( NORDIC_REG_EN_AA, data );
}

uint8_t nrf_read_En_AA( void )
{
    return nrf_read_register( NORDIC_REG_EN_AA );
}

void nrf_write_setup_retry( uint8_t data )
{
    nrf_write_register( NORDIC_REG_SETUP_RETR, data );
}

uint8_t nrf_read_setup_retry( void )
{
    return nrf_read_register( NORDIC_REG_SETUP_RETR );

}

void nrf_read_tx_addr( uint8_t *address )
{
    uint8_t i = 0;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_0,NORDIC_REG_TX_ADDR );
    spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO
    while( i < NORDIC_TX_ADDR_LEN )
    {
        spi_write_byte( SPI_0, 0xFF );    //Dummy to get the data
        *( address+i) = spi_read_byte(SPI_0 );
        i++;
    }

    nrf_chip_disable();
}

void nrf_write_tx_addr( uint8_t * tx_addr )
{
    nrf_chip_disable();
    nrf_chip_enable();
    spi_write_byte( SPI_0,NORDIC_REG_TX_ADDR | 0x20 );
    spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO
    spi_write_packet( SPI_0,tx_addr,NORDIC_TX_ADDR_LEN );
    spi_flush_rx( SPI_0 );

    nrf_chip_disable();
}

void nrf_read_rx_pipe_addr( uint8_t pipe_num, uint8_t *address )
{
    if( pipe_num > 5 )
        return;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_0,(NORDIC_REG_RX_ADDR_P0 + pipe_num) );
    spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO
    size_t ADDR_LEN = NORDIC_TX_ADDR_LEN;
    pipe_num > 2 ? ADDR_LEN = 1: 0;
    spi_read_packet( SPI_0, address, ADDR_LEN );

    nrf_chip_disable();
}

void nrf_write_rx_pipe_addr( uint8_t pipe_num, uint8_t *rx_addr )
{
    if( pipe_num > 5 )
        return;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_0,(NORDIC_REG_RX_ADDR_P0 + pipe_num) | 0x20 );
    spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO
    size_t ADDR_LEN = NORDIC_TX_ADDR_LEN;
    pipe_num > 1 ? ADDR_LEN = 1: 0;
    spi_write_packet( SPI_0,rx_addr,ADDR_LEN );
    spi_flush_rx( SPI_0 );

    nrf_chip_disable();
}


uint8_t nrf_read_fifo_status(  void  )
{
    return nrf_read_register( NORDIC_REG_FIFO_STATUS );
}

void nrf_flush_tx( void )
{
    nrf_write_command( NORDIC_CMD_FLUSH_TX );
}

void nrf_flush_rx( void )
{
    nrf_write_command( NORDIC_CMD_FLUSH_RX );
}

void nrf_enable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NORDIC_REG_EN_RXADDR );
    nrf_write_register( NORDIC_REG_EN_RXADDR, ret | (1<<rx_pipe_number)  );

}
void nrf_disable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NORDIC_REG_EN_RXADDR );
    nrf_write_register( NORDIC_REG_EN_RXADDR, ret & (~(1<<rx_pipe_number))  );
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
            nrf_flush_tx();
            nrf_write_En_AA( 0 );
            nrf_write_setup_retry( 0 );
            nrf_write_tx_addr( addr );
            nrf_write_rx_pipe_addr( rx_pipe_number, addr );
            nrf_enable_rx_pipe( rx_pipe_number );
            nrf_write_register(( NORDIC_REG_RX_PW_P0 ), payload_size );
            nrf_write_config( configureRead | NORDIC_CONFIG_PWR_UP( 1 )  );
            delayMs( 2 );
        }
        else
        {
            rx_configured = 1;
            configureRead |= NORDIC_CONFIG_PWR_UP( 1 ) | NORDIC_CONFIG_PRIM_RX( 1 );
            configureRead &= ~( NORDIC_CONFIG_RX_DR_INT( 1 ) );
            nrf_flush_rx();
            nrf_enable_rx_pipe( rx_pipe_number );
            nrf_write_rx_pipe_addr( rx_pipe_number, addr );
            nrf_write_register( (NORDIC_REG_RX_PW_P0 + rx_pipe_number), payload_size );
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
    spi_write_byte( SPI_0, NORDIC_CMD_W_TXPAYLD );
    spi_read_byte( SPI_0 ); //used to clear the previously value in the RX FIFO

    spi_write_packet( SPI_0,data, len );  //loading the FIFO with data  before enabling the CE pin
    spi_flush_rx( SPI_0 );
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
            while( !(( NORDIC_MASK_STATUS_TX_DS | NORDIC_MASK_STATUS_MAX_RT ) & status ) )
            {
                status = nrf_read_status();
            }
            nrf_write_status( NORDIC_MASK_STATUS_TX_DS  |
                              NORDIC_MASK_STATUS_MAX_RT |
                              NORDIC_MASK_STATUS_MAX_RT );
        }

        if( toRXMode )
        {
            configureRead &= ~( NORDIC_CONFIG_PRIM_RX( 1 ) );
            nrf_write_config( configureRead );
            nrf_flush_rx();
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

    spi_write_byte( SPI_0, NORDIC_CMD_R_RXPAYLD );
    spi_read_byte( SPI_0 );   //used to clear the previously value in the RX FIFO
    spi_read_packet( SPI_0,data,len );
    //spi_flush( SPI_0 );

    nrf_chip_disable();
}

uint8_t nrf_read_data( uint8_t *data, uint8_t len )
{
    if( rx_configured )
    {
        nrf_radio_enable();
        nrf_read_fifo_status();
        nrf_read_config();
        uint8_t val = 0;
        //Status reg if data is available
        if( interrupts_enabled )
        {
            while( received == 0 )    //wait till RX data in FIFO
            {
                val = nrf_read_fifo_status();
            }
            received = 0;
        }
        else
        {
            uint8_t status = 0;
            while( !( NORDIC_MASK_STATUS_RX_DR & status ) )
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
    uint8_t setup = nrf_module_init();
    if( setup == 0 || setup == 0xFF )
    {
        printf( "ERROR - SETUP: 0x%x\n", setup );
    }
    printf( "SPI Initialized\n");
    printf("Nordic Initialized\n");
    delayMs( 100 );

    uint8_t readValue = 0;
    printf("Nordic Test\n");
    readValue = nrf_read_config();
    printf("Recv: 0x%x\n",readValue);
    nrf_write_status( 0 );
    uint8_t sendValue = 0x08;
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

    nrf_write_register( NORDIC_REG_STATUS, 0 );
    sendValue = 44;
    nrf_set_channel( sendValue );
    readValue = nrf_get_channel();
    printf( "RF REG RECV: 0x%x\n", readValue );
    if( readValue == sendValue )
    {
        printf("Write/Read RF CH Value Matched\n");
        printf("Sent: 0x%x\n",sendValue);
        printf("Recv: 0x%x\n",readValue);
    }

//   sendValue = 0x07 ;
//   nrf_write_rf_setup( sendValue );
//   readValue = nrf_read_rf_setup();
//   if( readValue == sendValue )
//   {
//       printf("Write/Read RF Setup Value Matched\n");
//       printf("Sent: 0x%x\n",sendValue);
//       printf("Recv: 0x%x\n",readValue);
//   }
//
//   uint8_t sendAddr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
//   nrf_open_write_pipe( sendAddr );
//   printf("Configuring nrf in tx mode");
//   uint8_t readAddr[5];
//   nrf_read_tx_addr( readAddr );
//   printf( " tx ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);
//
//   printf( " RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);
//
//   nrf_read_rx_pipe_addr( 0, readAddr );
//   printf( " RX ADDRESSES GET: 0x%x%x%x%x%x\n",readAddr[0],readAddr[1],readAddr[2],readAddr[3],readAddr[4]);
//
//   uint8_t Data[5] = {0x55,0xBB,0xBB,0xBB,0xBB};
//   nrf_transmit_data( Data,5, false );
//   printf("Nordic Data Sent: 0x%x, 0x%x", Data[0],Data[1]);


    nrf_stop_listening();
    printf( "NRF STOP LISTENING\n" );
    nrf_power_off();
    printf( "NRF POWERED OFF\n" );
    spi_disable( SPI_0 );
    printf( "SPI0 DISABLED\n" );
    mraa_result_t status = mraa_gpio_close(NRF_IRQ_GPIO);
    if( MRAA_SUCCESS != status )
    {
        mraa_result_print( status );
    }
    status = mraa_gpio_close(NRF_CE_GPIO);
    if( MRAA_SUCCESS != status )
    {
        mraa_result_print( status );
    }

    status = mraa_gpio_close(NRF_CSN_GPIO);
    if( MRAA_SUCCESS != status )
    {
        mraa_result_print( status );
    }

}

void nordic_interrupt_handler( void *args )
{
    uint8_t reason = nrf_read_status();
    if( NORDIC_MASK_STATUS_TX_DS & reason )
    {
        nrf_write_status( reason | NORDIC_MASK_STATUS_TX_DS );
        transmitted = 1;
        printf( "NRF TX COMPLETED\n" );
    }
    if( NORDIC_MASK_STATUS_RX_DR & reason )
    {
        nrf_write_status( reason | NORDIC_MASK_STATUS_RX_DR );
        nrf_flush_rx();
        user_handler();
        received = 1;
        printf( "NRF RX COMPLETED\n" );
    }
    if( reason & NORDIC_MASK_STATUS_MAX_RT )
    {
        nrf_write_status( reason | NORDIC_MASK_STATUS_MAX_RT );
        nrf_flush_rx();
        user_handler();
        retry_error = 1;
        printf( "NRF TX RETRU ERROR\n" );
    }
}
