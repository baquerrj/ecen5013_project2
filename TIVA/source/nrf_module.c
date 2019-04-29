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

void nrf_module_deinit( void )
{
    nrf_stop_listening();
    printf( "NRF STOP LISTENING\n" );
    nrf_power_off();
    printf( "NRF POWERED OFF\n" );
    spi_disable( NRF_SPI );
    printf( "SPI_1 DISABLED\n" );

    GPIOIntClear( NRF_IRQ_PORT, NRF_IRQ_PIN );
    GPIOIntUnregister( NRF_IRQ_PORT );
}

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
    spi_clock_init( NRF_SPI );
    spi_init( NRF_SPI );
    delayMs( 1 );

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


    delayMs( 5 );

    nrf_write_register( NRF_REG_SETUP_RETR, (0b0101 << ARD) | (0b1111 << ARC));

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
    uint8_t config = nrf_read_config();
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
    uint8_t cfg = nrf_read_register( NRF_REG_CONFIG );
    nrf_write_register( NRF_REG_CONFIG, _BV(NRF_PWR_UP)  |
                                        _BV(NRF_PRIM_RX) |
                                        cfg );
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

    spi_write_byte( NRF_SPI, NRF_NOP );
    readValue = spi_read_byte( NRF_SPI );   //used to clear the previously value in the RX FIFO

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

    spi_write_byte( NRF_SPI,NRF_REG_TX_ADDR );
    spi_read_byte( NRF_SPI );   //used to clear the previously value in the RX FIFO
    while( i < NRF_TX_ADDR_LEN )
    {
        spi_write_byte( NRF_SPI, 0xFF );    //Dummy to get the data
        *( address+i) = spi_read_byte(NRF_SPI );
        i++;
    }

    nrf_chip_disable();
}

void nrf_write_tx_addr( uint8_t * tx_addr )
{
    nrf_chip_disable();
    nrf_chip_enable();
    spi_write_byte( NRF_SPI,NRF_REG_TX_ADDR | 0x20 );
    spi_read_byte( NRF_SPI );   //used to clear the previously value in the RX FIFO
    spi_write_packet( NRF_SPI,tx_addr,NRF_TX_ADDR_LEN );
    spi_flush_rx( NRF_SPI );

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
    nrf_write_packet( NRF_REG_RX_ADDR_P0, (uint8_t*)&address, 5 );
    nrf_write_packet( NRF_REG_TX_ADDR, (uint8_t*)&address, 5 );
    nrf_write_register( NRF_REG_RX_PW_P0, payload_size );

#ifdef NRF_DEBUG
    uint64_t tx_addr = 0;
    nrf_read_packet( NRF_REG_TX_ADDR, (uint8_t*)&tx_addr, 5 );
    LOG_INFO( "TX_ADDR: 0x%llX\n", tx_addr );

    uint64_t rx_addr_p0 = 0;
    nrf_read_packet( NRF_REG_RX_ADDR_P0, (uint8_t*)&rx_addr_p0, 5 );
    LOG_INFO( "RX_ADDR_P0: 0x%llX\n", rx_addr_p0 );

    uint8_t pl = nrf_read_register( NRF_REG_RX_PW_P0 );
    LOG_INFO( "NRF_REG_RX_PW_P0: 0x%X\n", pl );
#endif
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
    spi_write_byte( NRF_SPI, NRF_CMD_R_RX_PAYLOAD );
    while( len-- )
    {
        *buf++ = spi_read_byte( NRF_SPI );
    }
    while( blank_len-- )
    {
        spi_read_byte( NRF_SPI );
    }
    nrf_chip_disable();
    return 0;
}

uint8_t nrf_write_payload( const void *buf, uint8_t len, const uint8_t writeType )
{
    uint8_t status = 0;
    uint8_t *data = (uint8_t*)buf;
//    uint8_t blank_len = dynamic_payloads_enabled ? 0 : (payload_size - len);

    nrf_chip_enable();
    spi_write_byte( NRF_SPI, W_TX_PAYLOAD_NO_ACK );
//    nrf_write_command( W_TX_PAYLOAD );
//    status = spi_write_byte( NRF_SPI, W_TX_PAYLOAD );
    spi_read_byte( NRF_SPI );
    spi_write_packet( NRF_SPI, data, len );
    nrf_chip_disable();

    return status;
}

void nrf_tx_pulse( void )
{
    nrf_radio_enable();
    delayUs( 20 );
    nrf_radio_disable();
}

void nrf_start_write( void *buf, uint8_t len )
{
    // Transmitter power-up
    uint8_t config = nrf_read_register( NRF_REG_CONFIG );
    nrf_write_register( NRF_REG_CONFIG, ( config | _BV(NRF_PWR_UP) ) & ~_BV(NRF_PRIM_RX) );
    delayUs(150);

    // Send the payload
    //nrf_write_payload( buf, len );
    nrf_write_payload( buf, len, 0 );
    nrf_tx_pulse();
}

void nrf_start_fast_write( const void *buf, uint8_t len, const uint8_t multicast, uint8_t startTx )
{
    nrf_write_payload( buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD );
    if( startTx )
    {
        nrf_radio_enable();
    }
}

uint8_t nrf_write( void *buf, uint8_t len )
{
    // Start writing
    //nrf_start_write( buf, len );
    nrf_start_fast_write( buf, len, 1, 1 );

    uint32_t sent_at = __millis();
    uint8_t status = 0;
    uint32_t timeout = 500;
    do
    {
        status = nrf_read_register( NRF_REG_STATUS );
        print_status( status );
    }
    while( ! (status & (_BV(NRF_TX_DS) | _BV(NRF_MAX_RT))) && (__millis() - sent_at < timeout ) );
    printf( "Sent at %u\n", sent_at );
    nrf_radio_disable();

    status = nrf_read_status();
    print_status( status );
    nrf_write_register( NRF_REG_STATUS, NRF_MASK_RX_DR |
                                        NRF_MASK_TX_DS |
                                        NRF_MASK_MAX_RT );

    // Max retries exceeded
    if( status & NRF_MASK_MAX_RT )
    {
        printf( "MAX RETIRES\n" );
        nrf_flush_tx();
        return 0;
    }

    return 1;
}

uint8_t nrf_available( void )
{
    uint8_t status = nrf_read_register( NRF_REG_STATUS );

    uint8_t result = (status & NRF_MASK_RX_DR);

    if( result )
    {
        nrf_write_register( NRF_REG_STATUS, NRF_MASK_RX_DR );

        if( status & NRF_MASK_TX_DS )
        {
            nrf_write_register( NRF_REG_STATUS, NRF_MASK_TX_DS );
        }
    }
    return result;
}

uint8_t nrf_get_dynamic_payload_size( void )
{
    return nrf_read_register( NRF_CMD_RXPAYLD_W );
}

uint8_t nrf_read( void *buf, uint8_t len )
{
    nrf_read_payload( (uint8_t*)buf, len );

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
    spi_write_byte( NRF_SPI, ACTIVATE );
    spi_write_byte( NRF_SPI, 0x73 );
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

    nrf_write_register( NRF_REG_STATUS, 0 );
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

    uint64_t addr = 0xE7E7E7E7E7;
    uint8_t* buf = (uint8_t*)&addr;
    printf( "TX ADDRESS EXPECT: 0x%x%x%x%x%x\n", *buf, *(buf+1), *(buf+2), *(buf+3), *(buf+4)  );
    nrf_open_writing_pipe( addr );
    uint64_t raddr = 0;
    nrf_read_packet( NRF_REG_TX_ADDR, (uint8_t*)&raddr, 5 );
    buf = (uint8_t*)&raddr;
    printf( "TX ADDRESS GET: 0x%x%x%x%x%x\n", *buf, *(buf+1), *(buf+2), *(buf+3), *(buf+4)  );

    nrf_module_deinit();
}

void print_status(uint8_t status)
{
  printf( "STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n" ,
           status,
           (status & _BV(NRF_RX_DR))?1:0,
           (status & _BV(NRF_TX_DS))?1:0,
           (status & _BV(NRF_MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0x07),
           (status & _BV(TX_FULL))?1:0
          );
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
