/*
 * software_timer.h
 *
 *  Created on: Aug 21, 2023
 *      Author: MINH
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#define TIMER_CYCLE	10

extern int s27delayFlag;
extern int timer0Flag;
extern int timer1Flag;

void set_s27delay_timer();
void set_timer0(int milisecond);
void set_timer1(int milisecond);

void timer_run();


#endif /* INC_SOFTWARE_TIMER_H_ */
