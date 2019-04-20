/*!
 * @file	main.h
 *
 * @brief
 *
 *  Created on: Apr 10, 2019
 *      Author: Roberto Baquerizo
 */

#ifndef MAIN_H_
#define MAIN_H_


#define CLOCK_FREQ      (120000000)
#define FREQ_1HZ        (1000)
#define FREQ_2HZ        (FREQ_1HZ/2)
#define FREQ_3HZ        (FREQ_1HZ/3)
#define FREQ_5HZ        (FREQ_1HZ/5)
#define FREQ_10HZ       (FREQ_1HZ/10)
#define FREQ_HALF_HZ    (FREQ_1HZ*2)
#define FREQ_QUARTER_HZ (FREQ_1HZ*4)



#define REQ_LED_TASK        (FREQ_QUARTER_HZ)
#define FREQ_TMP102_TASK    (FREQ_1HZ)
#define FREQ_APDS9301_TASK  (FREQ_1HZ)

#endif /* MAIN_H_ */
