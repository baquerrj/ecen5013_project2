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

static uint8_t dynamic_payloads_enabled;
static uint8_t interrupts_enabled = 0;

static uint8_t ack_payload_available;
static uint8_t ack_payload_length;
static uint8_t payload_size = 32;

uint8_t pipe0_reading_address = 0;

extern uint32_t g_sysClock;

void nordic_interrupt_handler( void );

static interrupt_handler_t user_handler;


#if 1

/*!
* @brief - Enable the chip select connection to Nordic
* @return void
**/
void nrf_chip_enable( void )
{
    GPIOPinWrite( NRF_CSN_PORT, NRF_CSN_PIN, 0 );
    delayUs(50);
}

/*!
* @brief - Disable the chip select connection to Nordic
* @return void
**/
void nrf_chip_disable( void )
{
    GPIOPinWrite( NRF_CSN_PORT, NRF_CSN_PIN, NRF_CSN_PIN );
}

/*!
* @brief - Enable TX/RX from the Nordic module
* @return void
**/
void nrf_radio_enable( void )
{
    GPIOPinWrite( NRF_CE_PORT, NRF_CE_PIN, NRF_CE_PIN );
}

/*!
* @brief - Disable TX/RX from the Nordic module
* @return void
**/
void nrf_radio_disable( void )
{
    GPIOPinWrite( NRF_CE_PORT, NRF_CE_PIN, 0 );
}


/*!
 * @brief Send command to NRF module
 *
 * @param[in] command
 */
void nrf_write_command( uint8_t command )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, command );
    spi_read_byte( SPI_1 );

    nrf_chip_disable();
    return;
}

/*!
 * @brief - Read a register from the nrf module
 * @param - reg uint8_t
 * @return uint8_t
 */
uint8_t nrf_read_register( uint8_t reg )
{
   uint8_t data = 0;

   nrf_chip_disable();
   nrf_chip_enable();

   spi_write_byte( SPI_1, reg );
   spi_read_byte( SPI_1 );
   spi_write_byte( SPI_1, NRF_NOP );
   data = spi_read_byte( SPI_1 );

   nrf_chip_disable();
   return data;
}


/*!
 * @brief
 *
 * @param  <+NAME+> <+DESCRIPTION+>
 * @return <+DESCRIPTION+>
 * <+DETAILED+>
 */
uint8_t nrf_read_packet( uint8_t reg, uint8_t *buf, uint8_t len )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, R_REGISTER | (REGISTER_MASK & reg) );
    while( len-- )
    {
        *buf++ = spi_read_byte( SPI_1 );
    }
    nrf_chip_disable();
    return 0;
}

/*!
 * @brief - Write to a register from the nrf module
 * @param - reg uint8_t
 * @param - value uint8_t
 * @return void
 */
void nrf_write_register( uint8_t reg, uint8_t value )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, W_REGISTER | (REGISTER_MASK & reg) );
    spi_read_byte( SPI_1 );
    spi_write_byte( SPI_1, value );
    spi_read_byte( SPI_1 );

    nrf_chip_disable();
    return;
}


uint8_t nrf_write_packet( uint8_t reg, const uint8_t *buf, uint8_t len )
{
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, W_REGISTER | (REGISTER_MASK & reg) );
    while( len-- )
    {
        spi_write_byte( SPI_1, *buf++ );
    }

    nrf_chip_disable();
    return 0;
}


#endif


int8_t nrf_setup_interrupts( interrupt_handler_t handler )
{
    interrupts_enabled = 1;
    user_handler = handler;
    GPIOIntRegister( NRF_IRQ_PORT, nordic_interrupt_handler );
    GPIOIntDisable( NRF_IRQ_PORT, 0xFFFF );
    GPIOIntEnable( NRF_IRQ_PORT, NRF_IRQ_PIN );
    return 0;
}

uint8_t nrf_module_init( void )
{
    spi_clock_init( SPI_1 );
    spi_init( SPI_1 );
    delayMs( 1 );

 //   nrf_gpio_init();

    MAP_SysCtlPeripheralEnable( NRF_CE_SYSCTL_PORT );
    GPIOPinTypeGPIOOutput( NRF_CE_PORT, NRF_CE_PIN );
    GPIOPinWrite( NRF_CE_PORT, NRF_CE_PIN, 0 );

    MAP_SysCtlPeripheralEnable( NRF_CSN_SYSCTL_PORT );
    GPIOPinTypeGPIOOutput( NRF_CSN_PORT, NRF_CSN_PIN );
    GPIOPinWrite( NRF_CSN_PORT, NRF_CSN_PIN, NRF_CSN_PIN );

    MAP_SysCtlPeripheralEnable( NRF_IRQ_SYSCTL_PORT );

    GPIOIntDisable( NRF_IRQ_PORT,0xFFFF );
    GPIOPinTypeGPIOInput( NRF_IRQ_PORT,NRF_IRQ_PIN );
    GPIOIntUnregister( NRF_IRQ_PORT );
    GPIOIntClear( NRF_IRQ_PORT,0xFFFF );
    GPIOIntTypeSet( NRF_IRQ_PORT, NRF_IRQ_PIN, GPIO_LOW_LEVEL );

//    if( enable_interrupts )
//    {
//        interrupts_enabled = 1;
//        user_handler = handler;
//        GPIOIntRegister( NRF_IRQ_PORT, nordic_interrupt_handler );
//        GPIOIntDisable( NRF_IRQ_PORT,0xFFFF );
//        GPIOIntEnable( NRF_IRQ_PORT,NRF_IRQ_PIN );
//    }
//    else
//    {
//        interrupts_enabled = 0;
//    }
    delayMs( 5 );

    nrf_write_register( NRF_REG_SETUP_RETR, (0b0101 << ARD) | (0b1111 << ARC));
//    nrf_write_register( NRF_REG_CONFIG, 0x0C );

    nrf_set_palevel( NRF_PA_MAX );

    nrf_set_datarate( NRF_DR_1MBPS );

    nrf_set_crc_length( NRF_CRC_16 );

    nrf_write_register( NRF_REG_DYNPD, 0 );
    nrf_write_register( NRF_REG_STATUS, NRF_MASK_STATUS_RX_DR |
                                        NRF_MASK_STATUS_TX_DS |
                                        NRF_MASK_STATUS_MAX_RT );

    nrf_set_channel(76);

    nrf_flush_rx();
    nrf_flush_tx();

    nrf_power_on();



    return 0;
}

uint8_t nrf_set_datarate( nrf_data_rate_e speed )
{
    uint8_t result = 0;
    uint8_t setup = nrf_read_register( NRF_REG_RF_SETUP );

    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH) );

    if( NRF_DR_2MBPS == speed )
    {
        setup |= _BV( RF_DR_HIGH );
    }

    nrf_write_register( NRF_REG_RF_SETUP, setup );

    uint8_t check = nrf_read_register( NRF_REG_RF_SETUP );
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

void nrf_power_on( void )
{
    uint8_t cfg = nrf_read_register( NRF_REG_CONFIG );
    if( !(cfg & _BV(NRF_PWR_UP)) )
    {
        nrf_write_register( NRF_REG_CONFIG, cfg | _BV(NRF_PWR_UP) );
        delayMs( 5 );
    }
    return;
}


void nrf_power_off( void )
{
    interrupts_enabled = 0;
    uint8_t config = nrf_read_register( NRF_REG_CONFIG );
    nrf_write_register( NRF_REG_CONFIG, config & ~_BV(NRF_PWR_UP) );
    return;
}


void nrf_set_retries( uint8_t delay, uint8_t count )
{
    nrf_write_register( NRF_REG_SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) << ARC );
    return;
}

uint8_t nrf_get_retries( void )
{
    return nrf_read_register( NRF_REG_SETUP_RETR );
}

void nrf_set_crc_length( nrf_crc_e length )
{
    uint8_t config = nrf_read_register( NRF_REG_CONFIG ) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

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
    nrf_write_register( NRF_REG_CONFIG, config ) ;
}


nrf_crc_e nrf_get_crc_length( void )
{
    nrf_crc_e result = NRF_CRC_DISABLED;
    uint8_t config = nrf_read_register( NRF_REG_CONFIG ) & ( _BV(CRCO) | _BV(EN_CRC)) ;

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
    uint8_t setup = nrf_read_register( NRF_REG_RF_SETUP ) ;
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

    nrf_write_register( NRF_REG_RF_SETUP, setup ) ;
}

nrf_pa_level_e nrf_get_palevel( void )
{
    nrf_pa_level_e result = NRF_PA_ERROR ;
    uint8_t power = nrf_read_register(NRF_REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

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

void nrf_start_listening( void )
{
    uint8_t config = nrf_read_register( NRF_REG_CONFIG );
    nrf_write_register( NRF_REG_CONFIG, _BV(NRF_PWR_UP)  |
                                        _BV(NRF_PRIM_RX) |
                                        config );
    nrf_write_register( NRF_REG_STATUS, _BV(NRF_RX_DR) |
                                        _BV(NRF_TX_DS) |
                                        _BV(NRF_MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if( pipe0_reading_address )
  {
      nrf_write_packet( NRF_REG_RX_ADDR_P0, (uint8_t*)(&pipe0_reading_address), 5 );
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



void nrf_write_status( uint8_t statusValue )
{
    nrf_write_register( NRF_REG_STATUS, statusValue );
}

uint8_t nrf_read_status( void )
{
    uint8_t readValue = 0;

    //CSN High to low for new command
    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, NRF_NOP );
    readValue = spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO

    //Marking the end of transaction by CSN high
    nrf_chip_disable();

    return readValue;
}

void nrf_write_config( uint8_t configValue )
{
    nrf_write_register( NRF_REG_CONFIG, configValue );
}

uint8_t nrf_read_config( void )
{
    return nrf_read_register( NRF_REG_CONFIG );
}

uint8_t nrf_read_rf_setup( void )
{
    return nrf_read_register( NRF_REG_RF_SETUP );
}

void nrf_write_rf_setup( uint8_t rfSetupValue )
{
    nrf_write_register( NRF_REG_RF_SETUP, rfSetupValue );
}

uint8_t nrf_get_channel( void )
{
    return nrf_read_register( NRF_REG_RF_CH );
}

void nrf_set_channel( uint8_t channel )
{
    nrf_write_register( NRF_REG_RF_CH, channel );
}

void nrf_write_En_AA( uint8_t data )
{
    nrf_write_register( NRF_REG_EN_AA, data );
}

uint8_t nrf_read_En_AA( void )
{
    return nrf_read_register( NRF_REG_EN_AA );
}

void nrf_write_setup_retry( uint8_t data )
{
    nrf_write_register( NRF_REG_SETUP_RETR, data );
}

uint8_t nrf_read_setup_retry( void )
{
    return nrf_read_register( NRF_REG_SETUP_RETR );

}


void nrf_read_tx_addr( uint8_t *address )
{
    uint8_t i = 0;

    nrf_chip_disable();
    nrf_chip_enable();

    spi_write_byte( SPI_1, NRF_REG_TX_ADDR );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    while( i < NRF_TX_ADDR_LEN )
    {
        spi_write_byte( SPI_1, 0xFF );    //Dummy to get the data
        *( address+i) = spi_read_byte( SPI_1 );
        i++;
    }

    nrf_chip_disable();
}

void nrf_write_tx_addr( uint8_t * tx_addr )
{
    nrf_chip_disable();
    nrf_chip_enable();
    spi_write_byte( SPI_1, NRF_REG_TX_ADDR | 0x20 );
    spi_read_byte( SPI_1 );   //used to clear the previously value in the RX FIFO
    spi_write_packet( SPI_1, tx_addr, NRF_TX_ADDR_LEN );
    spi_flush_rx( SPI_1 );

    nrf_chip_disable();
}


void nrf_flush_tx( void )
{
    nrf_write_command( NRF_CMD_FLUSH_TX );
}

void nrf_flush_rx( void )
{
    nrf_write_command( NRF_CMD_FLUSH_RX );
}

void nrf_enable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NRF_REG_EN_RXADDR );
    nrf_write_register( NRF_REG_EN_RXADDR, ret | (1<<rx_pipe_number)  );

}
void nrf_disable_rx_pipe( uint8_t rx_pipe_number )
{
    if( rx_pipe_number > 5 )
        return;
    uint8_t ret = nrf_read_register( NRF_REG_EN_RXADDR );
    nrf_write_register( NRF_REG_EN_RXADDR, ret & (~(1<<rx_pipe_number))  );
}


static const uint8_t child_pipe[] =
{
  NRF_REG_RX_ADDR_P0, NRF_REG_RX_ADDR_P1, NRF_REG_RX_ADDR_P2, NRF_REG_RX_ADDR_P3, NRF_REG_RX_ADDR_P4, NRF_REG_RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  NRF_REG_RX_PW_P0, NRF_REG_RX_PW_P1, NRF_REG_RX_PW_P2, NRF_REG_RX_PW_P3, NRF_REG_RX_PW_P4, NRF_REG_RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void nrf_open_writing_pipe( uint64_t address )
{
    nrf_write_packet( NRF_REG_RX_ADDR_P0, (uint8_t*)(&address), 5 );
    nrf_write_packet( NRF_REG_TX_ADDR, (uint8_t*)(&address), 5 );

    nrf_write_register( NRF_REG_RX_PW_P0, payload_size );
    return;
}


void nrf_open_reading_pipe( uint8_t number, uint64_t address )
{
    if( 0 == number )
    {
        pipe0_reading_address = address;
    }
    if( number <= 6 )
    {
        if( number < 2 )
        {
            nrf_write_packet( child_pipe[ number ], (uint8_t*)(&address), 5 );
        }
        else
        {
            nrf_write_packet( child_pipe[ number ], (uint8_t*)(&address), 1 );
        }

        nrf_write_register( child_payload_size[ number ], payload_size );

        uint8_t read = nrf_read_register( NRF_REG_EN_RXADDR );
        nrf_write_register( NRF_REG_EN_RXADDR, read | _BV(child_pipe_enable[ number ]) );
    }
    return;
}


uint8_t nrf_read_payload( uint8_t *buf, uint8_t len )
{
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : (payload_size - len);

    nrf_chip_enable();
    spi_write_byte( SPI_1, NRF_CMD_R_RX_PAYLOAD );
    while( len-- )
    {
        *buf++ = spi_read_byte( SPI_1 );
    }
    while( blank_len-- )
    {
        spi_read_byte( SPI_1 );
    }
    nrf_chip_disable();
    return 0;
}

uint8_t nrf_write_payload( uint8_t *buf, uint8_t len )
{
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : (payload_size - len);

    nrf_chip_disable();
    nrf_chip_enable();
    spi_write_byte( SPI_1, NRF_CMD_W_TX_PAYLOAD );
    while( len-- )
    {
        spi_write_byte( SPI_1, *buf++ );
    }
    while( blank_len-- )
    {
        spi_write_byte( SPI_1, 0 );
    }

    nrf_chip_disable();

    return 0;
}

void nrf_tx_pulse( void )
{
    nrf_radio_enable();
    delayUs( 20 );
    nrf_radio_disable();
}

void nrf_start_write( uint8_t *buf, uint8_t len )
{
    // Transmitter power-up
    uint8_t config = nrf_read_register( NRF_REG_CONFIG );
    nrf_write_register( NRF_REG_CONFIG, ( config | _BV(NRF_PWR_UP) ) & ~_BV(NRF_PRIM_RX) );
    delayUs(150);

    // Send the payload
    nrf_write_payload( buf, len );

    nrf_tx_pulse();
}

uint8_t nrf_write( uint8_t *buf, uint8_t len )
{
    uint8_t result = 0;

    nrf_start_write( buf, len );

    uint8_t observe_tx;
    uint8_t status = 0;
    uint32_t timeout = 500;
    while( (timeout > 0) && !( status & ( _BV(NRF_TX_DS) | _BV(NRF_MAX_RT) )) )
    {
        status = nrf_read_packet( NRF_REG_OBSERVE_TX, &observe_tx, 1 );
        timeout--;
    }
    uint8_t tx_ok = 0;
    uint8_t tx_fail = 0;
    what_happened( &tx_ok, &tx_fail, &ack_payload_available );

    result = tx_ok;
    if( result )
    {
        printf( "TX SUCCESS\n" );
    }
    else
    {
        printf( "TX FAIL\n" );
    }

    if( ack_payload_available )
    {
        ack_payload_length = nrf_get_dynamic_payload_size();
        printf( "[AckPacket]/\n%d", ack_payload_length );
    }

    nrf_power_off();

    nrf_flush_tx();

    return result;
}

uint8_t nrf_get_dynamic_payload_size( void )
{
    return nrf_read_register( NRF_CMD_RXPAYLD_W );
}

uint8_t nrf_read( uint8_t *buf, uint8_t len )
{
    nrf_read_payload( buf, len );

    return nrf_read_register( NRF_REG_FIFO_STATUS ) & _BV( RX_EMPTY );
}

void what_happened( uint8_t *tx_ok, uint8_t *tx_fail, uint8_t *rx_ready )
{
    uint8_t status = nrf_read_register( NRF_REG_STATUS );

    *tx_ok = status & _BV(NRF_TX_DS);
    *tx_fail = status & _BV(NRF_MAX_RT);
    *rx_ready  = status & _BV(NRF_RX_DR);
    return;
}

void nrf_toggle_features( void )
{
    nrf_chip_enable();
    spi_write_byte( SPI_1, ACTIVATE );
    spi_write_byte( SPI_1, 0x78 );
    nrf_chip_disable();
    return;
}


void nrf_init_test( void )
{
    //nrf_module_init( 0, nordic_interrupt_handler );
    //nrf_module_setup( NRF_DR_1Mbps, NRF_POWER_LOW );
    nrf_module_init();
    printf( "SPI Initialized\n");
    printf( "Nordic Initialized\n" );
    delayMs( 100 );


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

    nrf_write_register( NRF_REG_STATUS, 0 );
    sendValue = 44;
    nrf_set_channel( sendValue );
    readValue = nrf_get_channel();
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
    spi_disable( SPI_1 );
    printf( "SPI_1 DISABLED\n" );

    GPIOIntClear( NRF_IRQ_PORT, NRF_IRQ_PIN );
    GPIOIntUnregister( NRF_IRQ_PORT );
    printf("Nordic Test End\n");

}

void nordic_interrupt_handler( void )
{
    MAP_IntMasterDisable();
        uint32_t int_status = GPIOIntStatus( NRF_IRQ_PORT, false );
        if( int_status & NRF_IRQ_PIN )
        {
            GPIOIntClear( NRF_IRQ_PORT, NRF_IRQ_PIN );
            uint8_t nrf_int_reason = nrf_read_status();
            if( nrf_int_reason & NRF_MASK_STATUS_TX_DS )
            {
                nrf_write_status( nrf_int_reason | NRF_MASK_STATUS_TX_DS );
                transmitted = 1;
            }
            if( nrf_int_reason & NRF_MASK_STATUS_RX_DR )
            {
                nrf_write_status( nrf_int_reason | NRF_MASK_STATUS_RX_DR );
                nrf_flush_rx();
                user_handler();
                received = 1;
                printf("nrf RX Complete\n");
            }
            if( nrf_int_reason & NRF_MASK_STATUS_MAX_RT )
            {
                nrf_write_status( nrf_int_reason | NRF_MASK_STATUS_MAX_RT );
                nrf_flush_tx();
                user_handler();
                retry_error = 1;
            }
        }
        MAP_IntMasterEnable();
}
