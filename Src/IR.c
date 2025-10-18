#include "IR.h"
#include "chassis.h"
#include "Bluetooth_USART2.h"

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

	LineFollower->previous_error = 0;
	LineFollower->integral = 0;
	LineFollower->Kp = 0.1;// Tune this
	LineFollower->Ki = 0.0;
	LineFollower->Kd = 0.1; // Tune this
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
    int sum = 0;
    int count = 0;
    for (int i = 0; i < 7; i++) {
        if (sensorStates[i]) {
            sum += weights[i];
            count++;
        }
    }
    *error = (count > 0) ? ( (float)sum/(float)count ) : 0;
    *total = count;
}

void LineFollower_FollowLine(LineFollower* LineFollower, CHASSIS* chassis, float forward_velocity) {
    volatile bool sensorStates[7];
    float weights[7] = {1, 0.5, 0.2, 0, -0.2, -0.5, -1};

    // Obtener estados de sensores (línea negra detectada o no)
    LineFollower_GetStates(LineFollower, sensorStates);

    // Calcular error
    float error = 0.0f;
    int total = 0;


    computeErrors(sensorStates, weights, &error, &total);
    USART2_SendSensorData(sensorStates, 7, error, total);

    // --- Control PID ---
    float derivative = error - LineFollower->previous_error;
    LineFollower->integral += error;
    LineFollower->previous_error = error;

    float angular_velocity = LineFollower->Kp * error
                           + LineFollower->Ki * LineFollower->integral
                           + LineFollower->Kd * derivative;

    // Limitar velocidad angular
    if (angular_velocity > 1.0f) angular_velocity = 1.0f;
    else if (angular_velocity < -1.0f) angular_velocity = -1.0f;

    if (total == 0) {
    	// Aplicar velocidades nulas al  chasis
		set_AdvanceSpeed(chassis, 0);      // Velocidad lineal
		set_TurnSpeed(chassis, 0);        // Corrección de giro
		apply_CurrentSpeedsToMotors_noBrake_if_0(chassis);             // Aplicar al hardware
    } else {
    	// Aplicar velocidades al chasis
		set_AdvanceSpeed(chassis, forward_velocity);      // Velocidad lineal
		set_TurnSpeed(chassis, -angular_velocity);        // Corrección de giro
		apply_CurrentSpeedsToMotors_noBrake_if_0(chassis);             // Aplicar al hardware
    }

}


void reset_LineFollowerPID(LineFollower* LineFollower) {
	LineFollower->previous_error = 0;
	LineFollower->integral = 0;
}

