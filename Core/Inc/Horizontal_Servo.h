/*
 * Vertical_Servo.h
 *
 *  Created on: Apr 14, 2025
 *      Author: jhfjf
 */

#ifndef INC_HORIZONTAL_SERVO_H_
#define INC_HORIZONTAL_SERVO_H_

#include "main.h"  // htim3, htim4 등의 정의를 위해 필요

void set_Ankle_Hori_Servo(uint8_t wheelNumber, float angle_deg);
void set_Hip_Hori_Servo(uint8_t legNumber, float angle_deg);

#endif /* INC_HORIZONTAL_SERVO_H_ */
