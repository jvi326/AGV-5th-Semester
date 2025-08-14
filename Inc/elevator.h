/*
 * motor_controller_Hbridge.h
 *
 *  Created on: Apr 27, 2025
 *      Author: javie
 */

#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "stm32f051x8.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int dirPin1; // Recommended 3
    int dirPin2; // Recommended 2
    bool inverted; // 0 = normal, 1 = polarization is inverted
    float currentSpeed;
} ELEVATOR;

// Initialization
void Elevator_Init(ELEVATOR* motor, int dir1, int dir2);

// Control functions
void elevator_SetSpeed(ELEVATOR* motor, float speed);
void elevator_Invert(ELEVATOR* motor, bool inverted);	// input: 0 for normal, 1 for inverted
void elevator_Forward(ELEVATOR* motor);
void elevator_Reverse(ELEVATOR* motor);

// Status functions
bool elevator_IsInverted(const ELEVATOR* motor);  // Returns true if motor is inverted

#endif /* MOTOR_CONTROLER_H_ */
