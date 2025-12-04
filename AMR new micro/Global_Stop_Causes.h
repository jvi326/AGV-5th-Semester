/*
 * Global_Stop_Causes.h
 * Definiciones para causas de paro (Stop Flags)
 * Pines PC12 Ultrasonicos con detección de flancos.
 */

#ifndef GLOBAL_STOP_CAUSES_H_
#define GLOBAL_STOP_CAUSES_H_

#include "stm32f051x8.h"

// === Pines y líneas EXTI ===
#define DIS1_SENSOR_PIN       12

// Líneas EXTI correspondientes
#define DIS1_SENSOR_EXTI_LINE  (1 << DIS1_SENSOR_PIN)

// === Estructuras ===
typedef struct {
    uint8_t distance1_flag;
} StopFlags_t;

// === Variables externas ===
extern volatile StopFlags_t stop_flags;

// === Funciones públicas ===
void StopCauses_Init(void);

#endif /* GLOBAL_STOP_CAUSES_H_ */
