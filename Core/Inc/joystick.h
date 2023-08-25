/*
 * button.h
 *
 *  Created on: Aug 20, 2023
 *      Author: MINH
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "main.h"

#define NUMBER_OF_JOY 	4 //left, right, center is used
#define JOY_CTR 		0
#define JOY_LEFT 		1
#define JOY_RIGHT 		2
#define USER_BUTTON		3

#define JOY_CTR_PIN		JOY_CTR_Pin
#define JOY_LEFT_PIN	JOY_A_Pin
#define JOY_RIGHT_PIN	JOY_D_Pin

#define NORMAL_STATE	1 // button of joystick is pull up
#define PRESSED_STATE	0

#define TIME_OUT		100

int is_joy_pressed(int joystick);

void get_joystick();

#endif /* INC_JOYSTICK_H_ */
