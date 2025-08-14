#ifndef SERVOS_H
#define SERVOS_H

#include <stdint.h>

void Servo_Init(void);
void Servo_SetAngleRange(uint8_t min_deg, uint8_t max_deg);
void Servo_SetStep(uint8_t step_deg);
void Servo_SetTimeStep(uint16_t ms);
void Servo_StartSweep(void);

#endif // SERVO_CONTROL_H
