/*
 * Global_Stop_Causes.c
 * Configuración de interrupciones externas (EXTI)
 * para PC11–PC15 con detección de flancos y pull-down interno.
 */

#include "Global_Stop_Causes.h"

volatile StopFlags_t stop_flags = {0};
volatile Treat_Failure_Flags_t Treat_Failure_Flags = {0};

void StopCauses_Init(void) {
    // --- Habilitar relojes ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // --- Configurar PC11–PC15 como entrada con Pull-Down ---
    GPIOC->MODER &= ~(
        (3 << (BT_DISCONN_PIN * 2)) |
        (3 << (DIS1_SENSOR_PIN * 2)) |
        (3 << (DIS2_SENSOR_PIN * 2)) |
        (3 << (TEMP_SENSOR_PIN * 2)) |
        (3 << (COLOR_SENSOR_PIN * 2))
    );

    GPIOC->PUPDR &= ~(
        (3 << (BT_DISCONN_PIN * 2)) |
        (3 << (DIS1_SENSOR_PIN * 2)) |
        (3 << (DIS2_SENSOR_PIN * 2)) |
        (3 << (TEMP_SENSOR_PIN * 2)) |
        (3 << (COLOR_SENSOR_PIN * 2))
    );
    GPIOC->PUPDR |= (
        (2 << (BT_DISCONN_PIN * 2)) |
        (2 << (DIS1_SENSOR_PIN * 2)) |
        (2 << (DIS2_SENSOR_PIN * 2)) |
        (2 << (TEMP_SENSOR_PIN * 2)) |
        (2 << (COLOR_SENSOR_PIN * 2))
    ); // Pull-Down interno

    // --- Asignar EXTI PC11–PC15 ---
    SYSCFG->EXTICR[2] &= ~(0xF << 12); // limpiar EXTI11
    SYSCFG->EXTICR[2] |=  (0x2 << 12); // PC11

    SYSCFG->EXTICR[3] &= ~(0xFFFF); // limpiar EXTI12–15
    SYSCFG->EXTICR[3] |=  (0x2222); // PC12–PC15

    // --- Activar detección por flancos de subida y bajada ---
    uint32_t lines = (BT_DISCONN_EXTI_LINE | DIS1_SENSOR_EXTI_LINE |
                      DIS2_SENSOR_EXTI_LINE | TEMP_SENSOR_EXTI_LINE |
                      COLOR_SENSOR_EXTI_LINE);

    EXTI->IMR  |= lines;   // habilita interrupciones
    EXTI->RTSR |= lines;   // flanco de subida
    EXTI->FTSR |= lines;   // flanco de bajada

    // --- Habilitar interrupción EXTI4_15 ---
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
}

void EXTI4_15_IRQHandler(void) {
    uint32_t pending = EXTI->PR;

    // --- Bluetooth Disconnect (PC11) ---
    if (pending & BT_DISCONN_EXTI_LINE) {
        EXTI->PR |= BT_DISCONN_EXTI_LINE;
        uint8_t curr = (GPIOC->IDR & (1 << BT_DISCONN_PIN)) ? 1 : 0;
        stop_flags.bluetooth_flag = curr;
    }

    // --- Distance 1 Sensor (PC12) ---
    if (pending & DIS1_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS1_SENSOR_EXTI_LINE;
        uint8_t curr = (GPIOC->IDR & (1 << DIS1_SENSOR_PIN)) ? 1 : 0;
        stop_flags.distance1_flag = curr;
    }

    // --- Distance 2 Sensor (PC13) ---
    if (pending & DIS2_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS2_SENSOR_EXTI_LINE;
        uint8_t curr = (GPIOC->IDR & (1 << DIS2_SENSOR_PIN)) ? 1 : 0;
        stop_flags.distance2_flag = curr;
    }

    // --- Extreme Temperature (PC14) ---
    if (pending & TEMP_SENSOR_EXTI_LINE) {
        EXTI->PR |= TEMP_SENSOR_EXTI_LINE;
        uint8_t curr = (GPIOC->IDR & (1 << TEMP_SENSOR_PIN)) ? 1 : 0;
        stop_flags.temperature_flag = curr;
    }

    // --- Color Detection (PC15) ---
    if (pending & COLOR_SENSOR_EXTI_LINE) {
        EXTI->PR |= COLOR_SENSOR_EXTI_LINE;
        uint8_t curr = (GPIOC->IDR & (1 << COLOR_SENSOR_PIN)) ? 1 : 0;
        stop_flags.color_flag = curr;
    }
}
