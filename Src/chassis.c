/*
 * chassis.c
 *
 *  Created on: May 4, 2025
 *      Author: javie
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f051x8.h"
#include "motor_controller.h"
#include "chassis.h"

static void calculateWheelSpeeds(CHASSIS* AGV_Chassis){
	//Selecting real max Speed
	float speedLimit;
	if (AGV_Chassis->advanceInverted) {
	    // Inverted logic: treat positive speed as backward
	    speedLimit = (AGV_Chassis->currentAdvanceSpeed >= 0) ?
	                 AGV_Chassis->safeFactorBackwardsSpeed :
	                 AGV_Chassis->maxSpeed;
	} else {
	    // Normal logic: treat negative speed as backward
	    speedLimit = (AGV_Chassis->currentAdvanceSpeed <= 0) ?
	                 AGV_Chassis->safeFactorBackwardsSpeed :
	                 AGV_Chassis->maxSpeed;
	}

	//Calculations for Advance Speed and Turn Speed
	float baseSpeed = AGV_Chassis->currentAdvanceSpeed * speedLimit;
	float turnEffect = AGV_Chassis->currentTurnSpeed * speedLimit;

	//Final calculations for speed
	AGV_Chassis->currentLeftWheelSpeed = baseSpeed - turnEffect;
	AGV_Chassis->currentRightWheelSpeed = baseSpeed + turnEffect;

	//Normalize result between range [-1.0, 1.0]
	float maxMagnitude = fmaxf(fabsf(AGV_Chassis->currentLeftWheelSpeed), AGV_Chassis->currentRightWheelSpeed);

	if (maxMagnitude > 1.0){
		float scaleFactor = 1/maxMagnitude;
		AGV_Chassis->currentLeftWheelSpeed *= scaleFactor;
		AGV_Chassis->currentRightWheelSpeed *= scaleFactor;
	}
}

//Init functions
void Init_Chassis(CHASSIS* AGV_Chassis, MotorController wheelLeft, MotorController wheelRight){

	AGV_Chassis->wheelLeft = wheelLeft;
	AGV_Chassis->wheelRight = wheelRight;

	AGV_Chassis->safeFactorBackwardsSpeed = 0.5;
	AGV_Chassis->maxSpeed = 0.8;
	AGV_Chassis->currentAdvanceSpeed = 0;
	AGV_Chassis->currentTurnSpeed = 0;

	AGV_Chassis->currentLeftWheelSpeed = 0;
	AGV_Chassis->currentRightWheelSpeed = 0;

	AGV_Chassis->advanceInverted = 0;
	AGV_Chassis->turnInverted = 0;
	AGV_Chassis->brakeEnabled = 1;

	//Initially set the motors with brakes
	Motor_BrakeMode(&AGV_Chassis->wheelLeft);
	Motor_BrakeMode(&AGV_Chassis->wheelRight);
}

//Control functions
void reset_ChassisSpeeds(CHASSIS* AGV_Chassis){
	AGV_Chassis->safeFactorBackwardsSpeed = 0.5f;
	AGV_Chassis->maxSpeed = 0.8f;
	AGV_Chassis->currentAdvanceSpeed = 0;
	AGV_Chassis->currentTurnSpeed = 0;
}

//Control functions
void apply_CurrentSpeedsToMotors(CHASSIS* AGV_Chassis){
	calculateWheelSpeeds(AGV_Chassis);
	Motor_SetSpeed(&AGV_Chassis->wheelLeft, AGV_Chassis->currentLeftWheelSpeed);
	Motor_SetSpeed(&AGV_Chassis->wheelRight, AGV_Chassis->currentRightWheelSpeed);
}
void set_SafeFactorBackwards(CHASSIS* AGV_Chassis, float safeFactor){
	safeFactor = (safeFactor < 0) ? 0 : (safeFactor > 1.0) ? 1.0 : safeFactor;
	AGV_Chassis->safeFactorBackwardsSpeed = safeFactor;
}

void set_MaxSpeed(CHASSIS* AGV_Chassis, float maxSpeed){
	maxSpeed = (maxSpeed < 0) ? 0 : (maxSpeed > 1.0) ? 1.0 : maxSpeed;
	AGV_Chassis->maxSpeed = maxSpeed;
}

void set_AdvanceSpeed(CHASSIS* AGV_Chassis, float advanceSpeed){
	if (AGV_Chassis->advanceInverted == 1) {
		advanceSpeed = -advanceSpeed;
	}
	advanceSpeed = (advanceSpeed < -1.0) ? -1.0 : (advanceSpeed > 1.0) ? 1.0 : advanceSpeed;
	AGV_Chassis->currentAdvanceSpeed = advanceSpeed;
}

void set_TurnSpeed(CHASSIS* AGV_Chassis, float turnSpeed){
	if (AGV_Chassis->turnInverted == 1) {
		turnSpeed = -turnSpeed;
	}
	turnSpeed = (turnSpeed < -1.0) ? -1.0 : (turnSpeed > 1.0) ? 1.0 : turnSpeed;
	AGV_Chassis->currentTurnSpeed = turnSpeed;
}

void set_AdvanceInverted(CHASSIS* AGV_Chassis, bool invert){
	AGV_Chassis->advanceInverted = invert;
}
void set_TurnInverted(CHASSIS* AGV_Chassis, bool invert){
	AGV_Chassis->turnInverted = invert;
}

void set_BrakeMode(CHASSIS* AGV_Chassis){
	Motor_BrakeMode(&AGV_Chassis->wheelLeft);
	Motor_BrakeMode(&AGV_Chassis->wheelRight);
	AGV_Chassis->brakeEnabled = 1;
}
void set_CoastMode(CHASSIS* AGV_Chassis){
	Motor_CoastMode(&AGV_Chassis->wheelLeft);
	Motor_CoastMode(&AGV_Chassis->wheelRight);
	AGV_Chassis->brakeEnabled = 0;
}

void stop_Chassis(CHASSIS* AGV_Chassis){
	set_AdvanceSpeed(AGV_Chassis, 0);
	set_TurnSpeed(AGV_Chassis, 0);
	apply_CurrentSpeedsToMotors(AGV_Chassis);
}

void pause_Chassis(CHASSIS* AGV_Chassis) {
	// Apply speed 0 to motors withouth altering the current speeds
	Motor_SetSpeed(&AGV_Chassis->wheelLeft, 0);
	Motor_SetSpeed(&AGV_Chassis->wheelRight, 0);
}

//Status functions
float get_CurrentChassisAdvanceSpeed(const CHASSIS* AGV_Chassis){
	return AGV_Chassis->currentAdvanceSpeed;
}
float get_CurrentChassisTurnSpeed(const CHASSIS* AGV_Chassis){
	return AGV_Chassis->currentTurnSpeed;
}
bool get_AdvanceInverted(const CHASSIS* AGV_Chassis){
	return AGV_Chassis->advanceInverted;
}
bool get_TurnInverted(const CHASSIS* AGV_Chassis){
	return AGV_Chassis->turnInverted;
}
bool get_BrakeEnabled(const CHASSIS* AGV_Chassis){
	return AGV_Chassis->brakeEnabled;
}

void SoftStart_Chassis(CHASSIS* chassis, float targetAdvance, float targetTurn, float rampTimeMs, float stepDelayMs) {
    int steps = (int)(rampTimeMs / stepDelayMs);
    for (int i = 1; i <= steps; i++) {
        float scale = (float)i / steps;
        float adv = targetAdvance * scale;
        float turn = targetTurn * scale;

        set_AdvanceSpeed(chassis, adv);
        set_TurnSpeed(chassis, turn);
        apply_CurrentSpeedsToMotors(chassis);
    }
}






