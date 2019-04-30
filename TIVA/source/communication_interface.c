/*!
 * @file  communication_interface.c
 * @brief
 *
 * <+DETAILED+>
 *
 * @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 * @internal
 *       Created:    04/27/2019
 *       Revision:   none
 *       Compiler:   gcc
 *  Organization:    University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#include "communication_interface.h"

void nrf_interrupt_handler( void )
{
}
const uint64_t pipes[6] =
					{ 0xF0F0F0F0D2LL,
                      0xF0F0F0F0E1LL,
                      0xF0F0F0F0E2LL,
                      0xF0F0F0F0E3LL,
                      0xF0F0F0F0F1LL,
                      0xF0F0F0F0F2LL
					};

volatile uint8_t count = 0;

int8_t comm_init_nrf( void )
{
    if( count )
    {
        count++;
        return 0;
    }

    //nrf_init_test();
    if( 0 != nrf_module_init() )
    {
        return -1;
    }

    nrf_set_channel(1);
    nrf_set_palevel( NRF_PA_MIN );
    nrf_open_writing_pipe( pipes[ 0 ] );
    nrf_open_reading_pipe( 1, pipes[ 1 ] );

    print_details();
    count++;

    //nrf_start_listening();
    return 0;
}

void comm_deinit_nrf( void )
{
    count--;
    if( count )
    {
        return;
    }

//    nrf_close_read_pipe( 1 );
//    nrf_close_write_pipe();

    nrf_module_deinit();
    return;
}
