/*
 * software_timer.c
 *
 *  Created on: Aug 21, 2023
 *      Author: MINH
 */
#include "software_timer.h"

int s27delayFlag = 0;
int s27delayCount = 0;

int timer0Flag = 0;
int timer0Count = 0;

int timer1Flag = 0;
int timer1Count = 0;

void set_s27delay_timer(){
	s27delayFlag = 1;
	s27delayCount = 1000; //10 second
}

void set_timer0(int milisecond){
	timer0Flag = 0;
	timer0Count = milisecond/TIMER_CYCLE;
}

void set_timer1(int milisecond){
	timer1Flag = 0;
	timer1Count = milisecond/TIMER_CYCLE;
}

void timer_run(){
	s27delayCount--;
	if(s27delayCount <= 0){
		s27delayFlag = 0;
	}

	timer0Count--;
	if(timer0Count <= 0){
		timer0Flag = 1;
	}

	timer1Count--;
	if(timer1Count <= 0){
		timer1Flag = 1;
	}
}

