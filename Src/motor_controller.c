#include "motor_controller.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define PWM_TIMER_PRESCALER (8000 - 1)   // 8MHz / 8 = 1MHz
#define PWM_TIMER_PERIOD    1000 - 1  // 1MHz / 1000 = 1kHz PWM

// Set brake relay state
void Brake_ON(MotorController* motor) {
	GPIOC->ODR &= ~(1 << motor->brakeRelayPin); // Relay OPEN (brake resistor active)
}

void Brake_OFF(MotorController* motor) {
	GPIOC->ODR |= (1 << motor->brakeRelayPin); // Relay CLOSED (normal operation)
}

// Set PWM duty cycle (0-1000)
static void setPWM(MotorController* motor, uint16_t value) {
    value = (value > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : value;

    if (motor->pwmPin == 8) {
        TIM3->CCR3 = value;
    } else if (motor->pwmPin == 9) {
        TIM3->CCR4 = value;
    }
}

// Set motor direction (true = forward)
static void setDirection(MotorController* motor, bool forward) {
    if (motor->inverted) {
        forward = !forward;
    }

    if (forward) {
    	GPIOC->ODR &= ~(1<< motor->dirPin1);
    	GPIOC->ODR |= (1<< motor->dirPin2);
    } else {
    	GPIOC->ODR &= ~(1<< motor->dirPin2);
    	GPIOC->ODR |= (1<< motor->dirPin1);
    }
}

// Set motor to no direction
static void setNoDirection(MotorController* motor) {
	GPIOC->ODR &= ~(1<< motor->dirPin1);
	GPIOC->ODR &= ~(1<< motor->dirPin2);
}

//Initializes peripherals in correspondent mode
//Peripherals C
//PWM should be either 8 or 9
//Directions peripherals (6, 7) or (10, 11) are suggested
//Brake peripherals 5 or 12 are suggested
void Motor_Init(MotorController* motor, int dir1, int dir2, int pwmpin, int brakepin){
	// Validate PWM pin (must be 8 or 9 for TIM3)
	if (pwmpin != 8 && pwmpin != 9) return;
	if ((dir1 > 15) || (dir2 > 15) || (brakepin > 15)) return; // Prevent invalid pins

    motor->dirPin1 = dir1;
    motor->dirPin2 = dir2;
    motor->pwmPin = pwmpin;
    motor->brakeRelayPin = brakepin;
    motor->currentSpeed = 0.0;
    motor->inverted = false;
    motor->brakeEnabled = false;

	// Enable GPIOC clock
	RCC->AHBENR |= (1 << 19);
	//Enable TIM3
	RCC->APB1ENR |= (1<<1);

	//Configure brake peripherals
	GPIOC->MODER &= ~( 3 << (2 * brakepin));
	GPIOC->MODER |= ( 1 << (2 * brakepin));

	GPIOC->ODR &= ~( 1 << (brakepin)); // Turn on motor brakes

	//Configure direction and pwm peripherals
	GPIOC->MODER &= ~(    (3 << (2 * dir1)) | (3 << (2 * dir2)) | (3 << (2 * pwmpin))    ); // Clean registers to be used
	GPIOC->MODER |=  (    (1 << (2 * dir1)) | (1 << (2 * dir2)) | (2 << (2 * pwmpin))   ); // Set direction peripherals as outputs and PWM as alternate function

	GPIOC->ODR &= ~(1 << (dir1) | 1 << (dir2)); // Turn off direction for motor controllers

	//AF0, in AFRH works for PC8 or greater
	GPIOC->AFR[1] &= ~(0xF << ((pwmpin-8)*4));

    TIM3->PSC = 47;
    TIM3->ARR = PWM_TIMER_PERIOD;
    TIM3->CR1 |= 1<<7; //Enable autoreload preload

	if (pwmpin == 8) {
		TIM3->CCR3 = 0; // Turn off completly PWM
		TIM3->CCMR2 |= 3<<5; //Mode 1 Bits 6:5 110
		TIM3->CCMR2 |= 1<<3; //Bit 3 Output compare preload enable
		TIM3->CCER |= 1<<8; //Enable CC3 output
	} else {
		TIM3->CCR4 = 0; // Turn off completly PWM
		TIM3->CCMR2 |= 3<<13; //Mode 1 Bits 6:5 110
		TIM3->CCMR2 |= 1<<11; //Bit 3 Output compare preload enable
		TIM3->CCER |= 1<<12; //Enable CC4 output
	}

	TIM3->EGR |= 1<<0; // Force register updates
	TIM3->CR1 |= 1<<0; //Enable counter enable
}

//Selects inverted state and changes it globally
void Motor_Invert(MotorController* motor, bool inverted) {
    motor->inverted = inverted;
}

//Sets speed for the motor
void Motor_SetSpeed(MotorController* motor, float speed) {
    // Constrain speed
    speed = (speed < -1.0) ? -1.0 : (speed > 1.0) ? 1.0 : speed;
    motor->currentSpeed = speed;

    if (fabsf(speed) < 0.001) {  // Near zero
        setPWM(motor, 0);
        setNoDirection(motor);
        Motor_BrakeMode(motor);
    } else {
    	Motor_CoastMode(motor);
        setDirection(motor, (speed > 0));
        setPWM(motor, (uint16_t)(fabsf(speed) * PWM_TIMER_PERIOD));
    }
}

// Set full forward speed
void Motor_Forward(MotorController* motor) {
    Motor_SetSpeed(motor, 1.0);
}

// Set full reverse speed
void Motor_Reverse(MotorController* motor) {
    Motor_SetSpeed(motor, -1.0);
}

// Enable brake mode (resistor absorbs current)
void Motor_BrakeMode(MotorController* motor) {
    motor->brakeEnabled = true;
     Brake_ON(motor);
}

// Disable brake mode (coast freely)
void Motor_CoastMode(MotorController* motor) {
    motor->brakeEnabled = false;
    Brake_OFF(motor);
}

// Status functions
bool Motor_IsInverted(const MotorController* motor) {
    return motor->inverted;
}

bool Motor_BrakeEnabled(const MotorController* motor) {
    return motor->brakeEnabled;
}

float Motor_GetCurrentSpeed(const MotorController* motor) {
    return motor->currentSpeed;
}
