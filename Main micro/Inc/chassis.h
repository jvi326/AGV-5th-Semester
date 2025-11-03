/*
 * chassis.h
 *
 *  Created on: Apr 28, 2025
 *      Author: javie
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "stm32f051x8.h"
#include "motor_controller.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
MotorController wheelLeft;
MotorController wheelRight;
float safeFactorBackwardsSpeed; //Range 0 to 1.0, recommended values below 0.5
float maxSpeed; //Range 0 to 1.0
float currentAdvanceSpeed; //Range -1.0 backWards to 1.0 Front
float currentTurnSpeed; //Range -1.0 LeftTurn to 1.0 RightTurn
float currentLeftWheelSpeed; //Current speed for left wheel
float currentRightWheelSpeed; //Current speed for right wheel
bool advanceInverted; // 0 = normal, 1 = front is at the back
bool turnInverted; // 0 = normal, 1 = turn directions are inverted
bool brakeEnabled; // 0 = coastMode, 1 = breakMode
} CHASSIS;

extern volatile CHASSIS agv;

// Structure to hold numbers extracted from USART_IRQ, either as float or integer
typedef union {
    int i;
    float f;
} Numeros;

extern bool justEnteredLineMode;
extern int lineFollowerMode;

extern float forward_speed_LF; // Choose based on desired speed (0.0 to 1.0)

extern float temp_P;
extern float temp_I;
extern float temp_D;

//Init functions
void Init_Chassis(CHASSIS* AGV_Chassis, MotorController wheelLeft, MotorController wheelRight);

//Control functions
void reset_ChassisSpeeds(CHASSIS* AGV_Chassis);
void apply_CurrentSpeedsToMotors(CHASSIS* AGV_Chassis);
void apply_CurrentSpeedsToMotors_noBrake_if_0(CHASSIS* AGV_Chassis);
void set_SafeFactorBackwards(CHASSIS* AGV_Chassis, float safeFactor);
void set_MaxSpeed(CHASSIS* AGV_Chassis, float maxSpeed);
void set_AdvanceSpeed(CHASSIS* AGV_Chassis, float advanceSpeed);
void set_TurnSpeed(CHASSIS* AGV_Chassis, float turnSpeed);
void set_AdvanceInverted(CHASSIS* AGV_Chassis, bool invert);
void set_TurnInverted(CHASSIS* AGV_Chassis, bool invert);
void set_BrakeMode(CHASSIS* AGV_Chassis);
void set_CoastMode(CHASSIS* AGV_Chassis);
void stop_Chassis(CHASSIS* AGV_Chassis);
void pause_Chassis(CHASSIS* AGV_Chassis);
void pause_Chassis_with_STOP(CHASSIS* AGV_Chassis);
void SoftStart_Chassis(CHASSIS* chassis, float targetAdvance, float targetTurn, float rampTimeMs, float stepDelayMs);

//Status functions
float get_CurrentChassisAdvanceSpeed(const CHASSIS* AGV_C);
float get_CurrentChassisTurnSpeed(const CHASSIS* AGV_C);
bool get_AdvanceInverted(const CHASSIS* AGV_C);
bool get_TurnInverted(const CHASSIS* AGV_C);
bool get_BrakeEnabled(const CHASSIS* AGV_C);

//General function
void decideDir(CHASSIS* AGV_Chassis,volatile Numeros* numeros, uint8_t count);

#endif /* CHASSIS_H_ */
