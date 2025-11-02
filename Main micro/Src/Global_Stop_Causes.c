/*
 * Global_Stop_Causes.c
 * Configuración de interrupciones externas (EXTI)
 * para PC11–PC13 y PC15.
 * Pines PC14, PA5, y PA7 solo se usan como lectura (sin interrupciones).
 */

#include "Global_Stop_Causes.h"

volatile StopFlags_t stop_flags = {0};
volatile Treat_Failure_Flags_t Treat_Failure_Flags = {0};

void StopCauses_Init(void) {
    // --- Habilitar relojes ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // --- Configurar PC11–PC15 como entrada con Pull-Down ---
    GPIOC->MODER &= ~(
        (3 << (BT_DISCONN_PIN * 2)) |
        (3 << (DIS1_SENSOR_PIN * 2)) |
        (3 << (DIS2_SENSOR_PIN * 2)) |
        (3 << (COLOR_SENSOR_PIN * 2)) |
        (3 << (RED_PIN * 2))
    );

    GPIOC->PUPDR &= ~(
        (3 << (BT_DISCONN_PIN * 2)) |
        (3 << (DIS1_SENSOR_PIN * 2)) |
        (3 << (DIS2_SENSOR_PIN * 2)) |
        (3 << (COLOR_SENSOR_PIN * 2)) |
        (3 << (RED_PIN * 2))
    );
    GPIOC->PUPDR |= (
        (2 << (BT_DISCONN_PIN * 2)) |
        (2 << (DIS1_SENSOR_PIN * 2)) |
        (2 << (DIS2_SENSOR_PIN * 2)) |
        (2 << (COLOR_SENSOR_PIN * 2)) |
        (2 << (RED_PIN * 2))
    ); // Pull-down interno

    // --- Configurar PA5 y PA7 como entrada con Pull-Down ---
    GPIOA->MODER &= ~(
        (3 << (GREEN_PIN * 2)) |
        (3 << (BLUE_PIN * 2))
    );

    GPIOA->PUPDR &= ~(
        (3 << (GREEN_PIN * 2)) |
        (3 << (BLUE_PIN * 2))
    );
    GPIOA->PUPDR |= (
        (2 << (GREEN_PIN * 2)) |
        (2 << (BLUE_PIN * 2))
    ); // Pull-down interno

    // --- Asignar EXTI solo a PC11–PC13 y PC15 ---
    // PC11 → EXTICR3 (bits 12–15)
    SYSCFG->EXTICR[2] &= ~(0xF << 12);
    SYSCFG->EXTICR[2] |=  (0x2 << 12); // PC11

    // PC12–PC15 → EXTICR4 (bits 0–15)
    SYSCFG->EXTICR[3] &= ~(0xFFFF);
    SYSCFG->EXTICR[3] |=  (0x2222);    // PC12–PC15

    // --- Activar detección por flancos de subida y bajada ---
    uint32_t lines = (BT_DISCONN_EXTI_LINE | DIS1_SENSOR_EXTI_LINE |
                      DIS2_SENSOR_EXTI_LINE | COLOR_SENSOR_EXTI_LINE);

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
        stop_flags.bluetooth_flag = (GPIOC->IDR >> BT_DISCONN_PIN) & 1;
    }

    // --- Distance 1 Sensor (PC12) ---
    if (pending & DIS1_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS1_SENSOR_EXTI_LINE;
        stop_flags.distance1_flag = (GPIOC->IDR >> DIS1_SENSOR_PIN) & 1;
    }

    // --- Distance 2 Sensor (PC13) ---
    if (pending & DIS2_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS2_SENSOR_EXTI_LINE;
        stop_flags.distance2_flag = (GPIOC->IDR >> DIS2_SENSOR_PIN) & 1;
    }

    // --- Color Detection (PC15) ---
    if (pending & COLOR_SENSOR_EXTI_LINE) {
        EXTI->PR |= COLOR_SENSOR_EXTI_LINE;
        stop_flags.color_flag = (GPIOC->IDR >> COLOR_SENSOR_PIN) & 1;
    }
}

void Update_ColorFlags(void) {
    stop_flags.red_flag   = (GPIOC->IDR >> RED_PIN) & 1;
    stop_flags.green_flag = (GPIOA->IDR >> GREEN_PIN) & 1;
    stop_flags.blue_flag  = (GPIOA->IDR >> BLUE_PIN) & 1;
}
