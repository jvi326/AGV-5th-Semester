/*
 * motor_controller_Hbridge.h
 *
 *  Created on: Apr 27, 2025
 *      Author: javie
 */

#ifndef MOTOR_CONTROLER_H
#define MOTOR_CONTROLER_H

#include "stm32f051x8.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int dirPin1; // Recommended 6 or 10
    int dirPin2; // Recommended 7 or 11
    int pwmPin;	// Use 8 or 9
    int brakeRelayPin; // Recommended 5 or 12
    float currentSpeed;  //Range -1.0 to 1.0
    bool inverted; // 0 = normal, 1 = polarization is inverted
    bool brakeEnabled; // 0 = coastMode, 1 = brakeMode
} MotorController;

// Initialization
void Motor_Init(MotorController* motor, int dir1, int dir2, int pwmpin, int brakepin);

// Control functions
void Motor_Invert(MotorController* motor, bool inverted);	// input: 0 for normal, 1 for inverted
void Motor_SetSpeed(MotorController* motor, float speed);  // -1.0 to 1.0
void Motor_Forward(MotorController* motor);
void Motor_Reverse(MotorController* motor);
void Motor_BrakeMode(MotorController* motor);
void Motor_CoastMode(MotorController* motor);

// Status functions
bool Motor_IsInverted(const MotorController* motor);  // Returns true if motor is inverted
bool Motor_BrakeEnabled(const MotorController* motor);  // Returns true if brake is enabled
float Motor_GetCurrentSpeed(const MotorController* motor); // Get current speed

#endif /* MOTOR_CONTROLER_H_ */
