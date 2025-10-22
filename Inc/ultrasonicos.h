#ifndef ULTRASONICOS_H
#define ULTRASONICOS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f051x8.h"

/* ---------- Configuration ---------- */
#define OBSTACLE_THRESHOLD_CM 50.0f  // Detection distance (cm)

/* ---------- Pin Definitions ---------- */
#define TRIG1_PIN 5  // PB5
#define ECHO1_PIN 6  // PB6

#define TRIG2_PIN 1  // PB1
#define ECHO2_PIN 2  // PB2

#define TRIG3_PIN 3  // PB3
#define ECHO3_PIN 4  // PB4

#define TRIG4_PIN 7  // PB7
#define ECHO4_PIN 8  // PB8

#define LED_PORT GPIOC
#define LED_PIN 9    // PC9

/* ---------- Global Variables ---------- */
extern volatile float distance1_cm;
extern volatile float distance2_cm;
extern volatile float distance3_cm;
extern volatile float distance4_cm;

/* ---------- Function Prototypes ---------- */
void Ultrasonicos_Init(void);
void Ultrasonicos_MeasureAll(void);
bool Ultrasonicos_Distance_Check(void);
void DelayMs(uint32_t ms);

float Ultrasonicos_GetDistance1(void);

void Ultrasonicos_EXTI_Init(void);
void Ultrasonicos_Trigger1(void);
void EXTI4_15_IRQHandler(void);

#endif /* ULTRASONICOS_H */
