/*!
 * @file  delay.h
 * @brief 
 *
 *  <+DETAILED+>
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


#ifndef  _DELAY_H_
#define  _DELAY_H_

#include <stdint.h>
#include <unistd.h>


static inline void delayMs( uint32_t ms )
{
    usleep( 1000 * ms );
}

static inline void delayUs( uint32_t us )
{
    usleep( us );
}

#endif   /* _DELAY_H_ */
