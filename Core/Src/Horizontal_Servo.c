/*
 * Vertical_Servo.c
 *
 *  Created on: Apr 14, 2025
 *      Author: jhfjf
 */
#include <stdint.h>
#include "main.h"
#include "tim.h"  // 타이머 핸들이 선언된 헤더 파일
#include "Horizontal_Servo.h"

// 타이머 핸들이 tim.c에 선언되어 있으므로 extern으로 가져옵니다
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

//0 degree to pi/2 degree
void set_Ankle_Hori_Servo(uint8_t wheelNumber, float angle_deg){
	//htim4
	uint32_t channel;
	uint32_t pulse;

	if(angle_deg < 0) angle_deg = 0;
	else if(angle_deg > 180) angle_deg = 180;

    switch(wheelNumber) {
        case 1:
            channel = TIM_CHANNEL_1;
            break;
        case 2:
            channel = TIM_CHANNEL_2;
            break;
        case 3:
            channel = TIM_CHANNEL_3;
            break;
        case 4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return; // 잘못된 휠 번호
    }

    if(wheelNumber == 3) {
        pulse = 480 + (uint32_t)(10 * (angle_deg / 0.9));
    } else {
        pulse = 500 + (uint32_t)(10 * (angle_deg / 0.9));
    }
	__HAL_TIM_SET_COMPARE(&htim4, channel, pulse);
}
void init_servo_set()
{
	  char a = 1;
	  set_Ankle_Hori_Servo(a, 90);
	  set_Hip_Hori_Servo(a, 0);
	  set_Ankle_Hori_Servo(a+1, 90);
	  set_Hip_Hori_Servo(a+1, 0);
	  set_Ankle_Hori_Servo(a+2, 90);
	  set_Hip_Hori_Servo(a+2, 0);
	  set_Ankle_Hori_Servo(a+3, 90);
	  set_Hip_Hori_Servo(a+3, 0);
}
void set_Hip_Hori_Servo(uint8_t legNumber, float angle_deg){
	//htim3
	uint32_t channel;
	uint32_t pulse;

	if(angle_deg < 0) angle_deg = 0;
	else if(angle_deg > 180) angle_deg = 180;

    switch(legNumber) {
        case 1:
            channel = TIM_CHANNEL_1;
            break;
        case 2:
            channel = TIM_CHANNEL_2;
            break;
        case 3:
            channel = TIM_CHANNEL_3;
            break;
        case 4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return; // 잘못된 휠 번호
    }

    if(legNumber == 3 || legNumber == 1) {
        pulse = 560 + (uint32_t)(10 * (angle_deg / 0.9));
    } else {
        pulse = 500 + (uint32_t)(10 * (angle_deg / 0.9));
    }

	__HAL_TIM_SET_COMPARE(&htim3, channel, pulse);
}
