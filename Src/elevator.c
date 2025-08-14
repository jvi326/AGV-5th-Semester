#include "elevator.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


// Set motor direction (true = forward)
static void elevator_setDirection(ELEVATOR* motor, bool forward) {
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
static void elevator_setNoDirection(ELEVATOR* motor) {
	GPIOC->ODR &= ~(1<< motor->dirPin1);
	GPIOC->ODR &= ~(1<< motor->dirPin2);
}

//Sets speed for the motor
void elevator_SetSpeed(ELEVATOR* motor, float speed) {
    // Constrain speed
    speed = (speed < -1.0) ? -1.0 : (speed > 1.0) ? 1.0 : speed;
    motor->currentSpeed = speed;

    if (fabsf(speed) < 0.001) {  // Near zero
    	elevator_setNoDirection(motor);
    } else {
    	elevator_setDirection(motor, (speed > 0));
    }
}

//Initializes peripherals in correspondent mode
//Peripherals C
//Directions peripherals 3 and 2 are suggested
void Elevator_Init(ELEVATOR* motor, int dir1, int dir2){
	if ((dir1 > 15) || (dir2 > 15)) return; // Prevent invalid pins

    motor->dirPin1 = dir1;
    motor->dirPin2 = dir2;
    motor->inverted = false;

	// Enable GPIOC clock
	RCC->AHBENR |= (1 << 19);
	//Enable TIM3
	RCC->APB1ENR |= (1<<1);

	//Configure direction and pwm peripherals
	GPIOC->MODER &= ~(    (3 << (2 * dir1)) | (3 << (2 * dir2))); // Clean registers to be used
	GPIOC->MODER |=  (    (1 << (2 * dir1)) | (1 << (2 * dir2))); // Set direction peripherals as outputs and PWM as alternate function

	GPIOC->ODR &= ~(1 << (dir1) | 1 << (dir2)); // Turn off direction for motor controllers

	elevator_SetSpeed(motor, 0.0);
}

//Selects inverted state and changes it globally
void elevator_Invert(ELEVATOR* motor, bool inverted) {
    motor->inverted = inverted;
}

// Set full forward speed
void elevator_Forward(ELEVATOR* motor) {
	elevator_SetSpeed(motor, 1.0);
}

// Set full reverse speed
void Elevator_Reverse(ELEVATOR* motor) {
	elevator_SetSpeed(motor, -1.0);
}

// Status functions
bool elevator_IsInverted(const ELEVATOR* motor) {
    return motor->inverted;
}

