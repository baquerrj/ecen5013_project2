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
#include <sys/time.h>

static struct timeval start, end;
static long mtime, seconds, useconds;
static inline void delayMs( uint32_t ms )
{
    usleep( 1000 * ms );
}

static inline void delayUs( uint32_t us )
{
    usleep( us );
}

static inline void __start_timer( void )
{
    gettimeofday( &start, NULL );
}

static inline long __millis( void )
{
    gettimeofday( &end, NULL );
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;

    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    return mtime;
}
#endif   /* _DELAY_H_ */
