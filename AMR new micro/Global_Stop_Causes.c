/*
 * Global_Stop_Causes.c
 * Configuración de interrupción externa (EXTI)
 * para el sensor ultrasónico en PC12.
 */

#include "Global_Stop_Causes.h"

volatile StopFlags_t stop_flags = {0};

void StopCauses_Init(void) {
    // --- Habilitar relojes ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // --- Configurar PC12 como entrada con Pull-Down ---
    GPIOC->MODER &= ~(3 << (DIS1_SENSOR_PIN * 2));     // Entrada
    GPIOC->PUPDR &= ~(3 << (DIS1_SENSOR_PIN * 2));     // Limpia Pull
    GPIOC->PUPDR |=  (2 << (DIS1_SENSOR_PIN * 2));     // Pull-down interno

    // --- Asignar EXTI12 a PC12 ---
    // EXTICR4 controla EXTI12–EXTI15
    SYSCFG->EXTICR[3] &= ~(0xF << 0);
    SYSCFG->EXTICR[3] |=  (0x2 << 0);   // PC = 0b0010 → puerto C

    // --- Activar interrupción en flanco de subida y bajada ---
    EXTI->IMR  |= DIS1_SENSOR_EXTI_LINE;  // Habilita línea
    EXTI->RTSR |= DIS1_SENSOR_EXTI_LINE;  // Rising edge
    EXTI->FTSR |= DIS1_SENSOR_EXTI_LINE;  // Falling edge

    // --- Habilitar IRQ EXTI4_15 ---
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(void) {
    if (EXTI->PR & DIS1_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS1_SENSOR_EXTI_LINE;  // Limpia bandera

        // Leer estado actual (0 o 1) del pin PC12
        stop_flags.distance1_flag = (GPIOC->IDR >> DIS1_SENSOR_PIN) & 1;
    }
}
