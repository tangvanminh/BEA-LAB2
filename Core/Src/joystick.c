/*
 * button.c
 *
 *  Created on: Aug 20, 2023
 *      Author: MINH
 */

#include <joystick.h>

// 3 temp variable for debounce
int temp1[NUMBER_OF_JOY] = {NORMAL_STATE};
int temp2[NUMBER_OF_JOY] = {NORMAL_STATE};
int temp3[NUMBER_OF_JOY] = {NORMAL_STATE};

int joy_flag[NUMBER_OF_JOY] = {0};

// timeout variable for long press
int timeOutForKeyPress = TIME_OUT;

// state of joys
int joy_state[NUMBER_OF_JOY] = {NORMAL_STATE};

int is_joy_pressed(int joystick){
	if(joy_flag[joystick] == 1){
		joy_flag[joystick] = 0;
		return 1;
	}
	return 0;
}

void get_joystick(){
	//left joy
	temp3[JOY_LEFT] = temp2[JOY_LEFT];
	temp2[JOY_LEFT] = temp1[JOY_LEFT];
	temp1[JOY_LEFT] = HAL_GPIO_ReadPin(GPIOC, JOY_LEFT_PIN);

	if((temp3[JOY_LEFT] == temp2[JOY_LEFT]) && (temp2[JOY_LEFT] == temp1[JOY_LEFT])){
		if(joy_state[JOY_LEFT] != temp3[JOY_LEFT]){
			joy_state[JOY_LEFT] = temp3[JOY_LEFT];

			if(joy_state[JOY_LEFT] == PRESSED_STATE){
				timeOutForKeyPress = TIME_OUT;
				joy_flag[JOY_LEFT] = 1;
				joy_flag[JOY_CTR] = 0;
				joy_flag[JOY_RIGHT] = 0;
			}
		}else{
			timeOutForKeyPress--;
			if(timeOutForKeyPress == 0){
				joy_state[JOY_LEFT] = NORMAL_STATE;
			}
		}
	}

	//right joy
	temp3[JOY_RIGHT] = temp2[JOY_RIGHT];
	temp2[JOY_RIGHT] = temp1[JOY_RIGHT];
	temp1[JOY_RIGHT] = HAL_GPIO_ReadPin(GPIOC, JOY_RIGHT_PIN);

	if((temp3[JOY_RIGHT] == temp2[JOY_RIGHT]) && (temp2[JOY_RIGHT] == temp1[JOY_RIGHT])){
		if(joy_state[JOY_RIGHT] != temp3[JOY_RIGHT]){
			joy_state[JOY_RIGHT] = temp3[JOY_RIGHT];

			if(joy_state[JOY_RIGHT] == PRESSED_STATE){
				timeOutForKeyPress = TIME_OUT;
				joy_flag[JOY_RIGHT] = 1;
				joy_flag[JOY_LEFT] = 0;
				joy_flag[JOY_CTR] = 0;
			}
		}else{
			timeOutForKeyPress--;
			if(timeOutForKeyPress == 0){
				joy_state[JOY_RIGHT] = NORMAL_STATE;
			}
		}
	}

	// center joy
	temp3[JOY_CTR] = temp2[JOY_CTR];
	temp2[JOY_CTR] = temp1[JOY_CTR];
	temp1[JOY_CTR] = HAL_GPIO_ReadPin(GPIOC, JOY_CTR_PIN);

	if((temp3[JOY_CTR] == temp2[JOY_CTR]) && (temp2[JOY_CTR] == temp1[JOY_CTR])){
		if(joy_state[JOY_CTR] != temp3[JOY_CTR]){
			joy_state[JOY_CTR] = temp3[JOY_CTR];

			if(joy_state[JOY_CTR] == PRESSED_STATE){
				timeOutForKeyPress = TIME_OUT;
				joy_flag[JOY_CTR] = 1;
				joy_flag[JOY_RIGHT] = 0;
				joy_flag[JOY_LEFT] = 0;
			}
		}else{
			timeOutForKeyPress--;
			if(timeOutForKeyPress == 0){
				joy_state[JOY_CTR] = NORMAL_STATE;
			}
		}
	}
}
