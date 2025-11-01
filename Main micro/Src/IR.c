#include "IR.h"
#include "chassis.h"
#include "Global_Stop_Causes.h"
#include "Bluetooth_USART2.h"

static float pid_last_error = 0;
static float pid_integral = 0;

void IR_Init(IRSensor* IRSensor){
	if (IRSensor->trig_port == GPIOB)
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if (IRSensor->trig_port == GPIOC)
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if (IRSensor->trig_port == GPIOA)
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	IRSensor->trig_port->MODER &= ~(3<<2*IRSensor->signal_pin);
}

bool IR_GetState(IRSensor* IRSensor){
	int state_boton = IRSensor->trig_port->IDR & (1 << IRSensor->signal_pin);

	return state_boton;
}

void LineFollower_Init(LineFollower* LineFollower){
	IR_Init(&LineFollower->MostOutRight);
	IR_Init(&LineFollower->OutRight);
	IR_Init(&LineFollower->CenterRight);
	IR_Init(&LineFollower->Center);
	IR_Init(&LineFollower->CenterLeft);
	IR_Init(&LineFollower->OutLeft);
	IR_Init(&LineFollower->MostOutLeft);
}

void LineFollower_GetStates(LineFollower* LineFollower, volatile bool* LineFollowerStatesArray){
	LineFollowerStatesArray[0] = IR_GetState(&LineFollower->MostOutRight);
	LineFollowerStatesArray[1] = IR_GetState(&LineFollower->OutRight);
	LineFollowerStatesArray[2] = IR_GetState(&LineFollower->CenterRight);
	LineFollowerStatesArray[3]= IR_GetState(&LineFollower->Center);
	LineFollowerStatesArray[4] = IR_GetState(&LineFollower->CenterLeft);
	LineFollowerStatesArray[5]= IR_GetState(&LineFollower->OutLeft);
	LineFollowerStatesArray[6] = IR_GetState(&LineFollower->MostOutLeft);
}

void computeErrors(volatile bool sensorStates[7], float weights[7], float* error, int* total) {
    *error = 0.0f;
    *total = 0;

    for (int i = 0; i < 7; i++) {
        if (sensorStates[i]) {      // Sensor sees line
            *error += weights[i];
            (*total)++;
        }
    }

    if (*total > 0) {
        *error /= *total;  // average position
    } else {
        *error = 0.0f;     // Line lost
    }
}

void LineFollower_FollowLine(LineFollower* LineFollower, CHASSIS* chassis, float forward_velocity) {
    bool sensorStates[7];
    float weights[7] = {45, 30, 15, 0, -15, -30, -45};
    float error;
    int total;

    // 1️⃣ Read sensors and compute weighted error
    LineFollower_GetStates(LineFollower, sensorStates);
    computeErrors(sensorStates, weights, &error, &total);

    float leftSpeed  = 0.0f;
    float rightSpeed = 0.0f;
    float turnFactor = 0.0f;

    if (total == 0) {
        // 🚫 Line lost → stop motors
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelLeft, 0);
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelRight, 0);
    } else {
    	error = -error;
    	//Side correction
        // 2️⃣ PID computation
        pid_integral += error;
        float derivative = error - pid_last_error;
        pid_last_error = error;

        float Kp = temp_P;
        float Ki = temp_I;
        float Kd = temp_D;

        turnFactor = Kp*error + Ki*pid_integral + Kd*derivative;

        // 3️⃣ Clamp output
        if (turnFactor > 1.0f) turnFactor = 1.0f;
        if (turnFactor < -1.0f) turnFactor = -1.0f;

        // 4️⃣ Base forward velocity
        leftSpeed  = chassis->advanceInverted ? -forward_velocity : forward_velocity;
        rightSpeed = chassis->advanceInverted ? -forward_velocity : forward_velocity;

        // 5️⃣ Apply correction (differential steering)
        if (turnFactor > 0) {
            // Line is to the right → turn right (reduce left)
            leftSpeed  *= (1.0f - turnFactor);
        } else if (turnFactor < 0) {
            // Line is to the left → turn left (reduce right)
            rightSpeed *= (1.0f + turnFactor); // turnFactor is negative
        }

        // 6️⃣ Apply motor speeds
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelLeft, leftSpeed);
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelRight, rightSpeed);
    }


    // 7️⃣ Send debug info
    USART2_SendSensorData(sensorStates, 7, error, total);
}


void LineFollower_FollowLine_PID(LineFollower* LineFollower, CHASSIS* chassis, float forward_velocity,
                                 float Kp, float Ki, float Kd) {
    bool sensorStates[7];
    float weights[7] = {45, 30, 15, 0, -15, -30, -45};
    float error;
    int total;

    // 1️⃣ Read sensor states
    LineFollower_GetStates(LineFollower, sensorStates);
    computeErrors(sensorStates, weights, &error, &total);

    float leftSpeed  = 0;
    float rightSpeed = 0;

    if (total == 0) {
        // Line lost → stop motors
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelLeft, 0);
        Motor_SetSpeed_noBreak_if_0(&chassis->wheelRight, 0);
    } else {
    	pid_integral += error;
		float derivative = error - pid_last_error;
		pid_last_error = error;

		float turnFactor = temp_D*error + temp_I*pid_integral + temp_D*derivative;

		// Limit turnFactor to [-1, 1]
		if (turnFactor > 1.0f) turnFactor = 1.0f;
		if (turnFactor < -1.0f) turnFactor = -1.0f;

		// 3️⃣ Base speed
		float leftSpeed  = &chassis->advanceInverted ? forward_velocity : -forward_velocity;
		float rightSpeed = chassis->advanceInverted ? forward_velocity : -forward_velocity;

		if (turnFactor < 0) {
			leftSpeed = (chassis->advanceInverted ? forward_velocity : -forward_velocity) * (1.0f - turnFactor);   // turn right → reduce left
		} else if (turnFactor > 0) {
			rightSpeed = (chassis->advanceInverted ? forward_velocity : -forward_velocity) * (1.0f + turnFactor);  // turn left → reduce right
		}

		// 4️⃣ Apply speeds to motors
		Motor_SetSpeed_noBreak_if_0(&chassis->wheelLeft, leftSpeed);
		Motor_SetSpeed_noBreak_if_0(&chassis->wheelRight, rightSpeed);
    }

    // 5️⃣ Send debug info
    USART2_SendSensorData(sensorStates, 7, error, total);
    USART2_SendFloat(leftSpeed, 2);
    USART2_SendFloat(rightSpeed, 2);
}

void resetPID() {
	pid_last_error = 0;
	pid_integral = 0;
}

