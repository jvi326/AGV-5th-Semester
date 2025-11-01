/*
 * Global_Stop_Causes.h
 * Definiciones para causas de paro (Stop Flags)
 * Pines PC11–PC15 con detección de flancos.
 */

#ifndef GLOBAL_STOP_CAUSES_H_
#define GLOBAL_STOP_CAUSES_H_

#include "stm32f051x8.h"

// === Pines y líneas EXTI ===
#define BT_DISCONN_PIN        11
#define DIS1_SENSOR_PIN       12
#define DIS2_SENSOR_PIN       1
#define TEMP_SENSOR_PIN       14
#define COLOR_SENSOR_PIN      15

#define BT_DISCONN_EXTI_LINE  (1 << BT_DISCONN_PIN)
#define DIS1_SENSOR_EXTI_LINE (1 << DIS1_SENSOR_PIN)
#define DIS2_SENSOR_EXTI_LINE (1 << DIS2_SENSOR_PIN)
#define TEMP_SENSOR_EXTI_LINE (1 << TEMP_SENSOR_PIN)
#define COLOR_SENSOR_EXTI_LINE (1 << COLOR_SENSOR_PIN)

// === Estructuras ===
typedef struct {
    uint8_t bluetooth_flag;
    uint8_t distance1_flag;
    uint8_t distance2_flag;
    uint8_t temperature_flag;
    uint8_t color_flag;
} StopFlags_t;

typedef struct {
    uint8_t bluetooth_Fail_cmd;
    uint8_t distance1_Fail_cmd;
    uint8_t distance2_Fail_cmd;
    uint8_t temperature_Fail_cmd;
    uint8_t color_Fail_cmd;
} Treat_Failure_Flags_t;

// === Variables externas ===
extern volatile StopFlags_t stop_flags;
extern volatile Treat_Failure_Flags_t Treat_Failure_Flags;
extern volatile int error_0_counter;

// === Funciones públicas ===
void StopCauses_Init(void);

#endif /* GLOBAL_STOP_CAUSES_H_ */
