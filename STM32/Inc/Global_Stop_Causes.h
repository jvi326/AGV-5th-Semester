/*
 * Global_Stop_Causes.h
 *
 *  Created on: Oct 28, 2025
 *      Author: javie
 */

#ifndef GLOBAL_STOP_CAUSES_H_
#define GLOBAL_STOP_CAUSES_H_

#include "stm32f051x8.h"


// --- Bluetooth Disconnect ---
#define BT_DISCONN_PORT           GPIOA
#define BT_DISCONN_PIN            10
#define BT_DISCONN_EXTI_LINE      (1 << BT_DISCONN_PIN)
#define BT_DISCONN_EXTICR_INDEX   2   // EXTI10–13 are in EXTICR[2]
#define BT_DISCONN_EXTICR_POS     8   // PA10 position in EXTICR[2]

// --- Distance 1 Sensor ---
#define DIS1_SENSOR_PORT          GPIOA
#define DIS1_SENSOR_PIN           11
#define DIS1_SENSOR_EXTI_LINE     (1 << DIS1_SENSOR_PIN)
#define DIS1_SENSOR_EXTICR_INDEX  2
#define DIS1_SENSOR_EXTICR_POS    12

// --- Distance 2 Sensor ---
#define DIS2_SENSOR_PORT          GPIOA
#define DIS2_SENSOR_PIN           7
#define DIS2_SENSOR_EXTI_LINE     (1 << DIS2_SENSOR_PIN)
#define DIS2_SENSOR_EXTICR_INDEX  2
#define DIS2_SENSOR_EXTICR_POS    8

// === Flag structure ===
typedef struct {
    uint8_t bluetooth_flag;   // PA10
    uint8_t distance1_flag;   // PA11
    uint8_t distance2_flag;   // PB10
} StopFlags_t;

extern volatile int error_0_counter;
extern volatile StopFlags_t stop_flags;

// === Public functions ===
void StopCauses_Init(void);
void StopCauses_ClearAll(void);

#endif /* GLOBAL_STOP_CAUSES_H_ */
