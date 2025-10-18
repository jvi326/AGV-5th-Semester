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

void apply_CurrentSpeedsToMotors_noBrake_if_0(CHASSIS* AGV_Chassis){
	calculateWheelSpeeds(AGV_Chassis);
	Motor_SetSpeed_noBreak_if_0(&AGV_Chassis->wheelLeft, AGV_Chassis->currentLeftWheelSpeed);
	Motor_SetSpeed_noBreak_if_0(&AGV_Chassis->wheelRight, AGV_Chassis->currentRightWheelSpeed);
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

void decideDir(CHASSIS* AGV_Chassis, volatile Numeros* numeros, uint8_t count) {

    // Asegura que no hay elementos extras para la decisión
    if (count < 4) return;

    if (numeros[0].i == 0) {
        // Apagar todo si el primer valor es 0
    	stop_Chassis(AGV_Chassis);
    } else if (numeros[0].i == 1) {
        if (numeros[1].i == 0) {
        	stop_Chassis(AGV_Chassis);
        	set_CoastMode(AGV_Chassis);
        } else if (numeros[1].i == 1) {
            float avance = numeros[2].f;
            float giro = numeros[3].f;


            // Validación del rango permitido para avance
            if (avance >= -1.0 && avance <= 1.0) {
                set_AdvanceSpeed(AGV_Chassis, avance);
            } else {
                // Si el valor está fuera de rango, detener
                set_AdvanceSpeed(AGV_Chassis, 0);
            }
            if (giro >= -1.0 && giro <= 1.0) {
				set_TurnSpeed(AGV_Chassis, giro);
			} else {
				// Si el valor está fuera de rango, detener
				set_TurnSpeed(AGV_Chassis, 0);
			}
            apply_CurrentSpeedsToMotors(AGV_Chassis);

        } else if (numeros[1].i == 2) {
        	lineFollowerMode = 1;
        	justEnteredLineMode = 1;

        	float avance = numeros[2].f;


			// Validación del rango permitido para avance
			if (avance >= -1.0 && avance <= 1.0) {
				forward_speed_LF = avance;
			} else {
				forward_speed_LF = 0;
			}
        }
    } else if (numeros[1].i == 3) {
    	lineFollowerMode = 1;
    	justEnteredLineMode = 1;

    	float avance = numeros[2].f;

    	temp_P = numeros[3].f;
    	temp_I = numeros[4].f;
    	temp_D = numeros[5].f;


		// Validación del rango permitido para avance
		if (avance >= -1.0 && avance <= 1.0) {
			forward_speed_LF = avance;
		} else {
			forward_speed_LF = 0;
		}

    } else {
        // Cualquier otro caso detiene el sistema
    	stop_Chassis(AGV_Chassis);
    }

    if ((numeros[1].i != 2) & (numeros[1].i != 3)) lineFollowerMode = 0;
}





