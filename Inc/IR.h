/*
 * IR_configure.h
 *
 *  Created on: Jun 15, 2025
 *      Author: javie
 */

#ifndef IR_H_
#define IR_H_

#include "stm32f051x8.h"
#include "chassis.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t signal_pin;
} IRSensor;

typedef struct {
	IRSensor OutRight;
	IRSensor CenterRight;
	IRSensor Center;
	IRSensor CenterLeft;
	IRSensor OutLeft;

    // PID variables
    float previous_error;
    float integral;

    float Kp;
    float Ki;
    float Kd;
} LineFollower;

void IR_Init(IRSensor* IRSensor);
bool IR_GetState(IRSensor* sensor);
void LineFollower_Init(LineFollower* LineFollower);
void LineFollower_GetStates(LineFollower* LineFollower, volatile bool* LineFollowerStatesArray);
void computeErrors(volatile bool sensorStates[5], int weights[5], float* error, int* total);
void LineFollower_FollowLine(LineFollower* LineFollower, CHASSIS* chassis, float forward_velocity);

#endif /* IR_H_ */
