#include "Global_Stop_Causes.h"

volatile StopFlags_t stop_flags = {0};

void StopCauses_Init(void) {
    // Enable GPIOA, GPIOB, and SYSCFG clocks
    RCC->AHBENR  |= (1 << 17) | (1 << 18);
    RCC->APB2ENR |= (1 << 0);

    // --- Bluetooth Disconnect (PA10) ---
    BT_DISCONN_PORT->MODER &= ~(3 << (2 * BT_DISCONN_PIN));
    BT_DISCONN_PORT->PUPDR &= ~(3 << (2 * BT_DISCONN_PIN));
    BT_DISCONN_PORT->PUPDR |=  (1 << (2 * BT_DISCONN_PIN));   // Pull-up
    SYSCFG->EXTICR[BT_DISCONN_EXTICR_INDEX] &= ~(0xF << BT_DISCONN_EXTICR_POS);
    SYSCFG->EXTICR[BT_DISCONN_EXTICR_INDEX] |=  (0x0 << BT_DISCONN_EXTICR_POS); // PA
    EXTI->RTSR |= BT_DISCONN_EXTI_LINE;
    EXTI->FTSR |= BT_DISCONN_EXTI_LINE;
    EXTI->IMR  |= BT_DISCONN_EXTI_LINE;

    // --- Distance 1 Sensor (PA11) ---
    DIS1_SENSOR_PORT->MODER &= ~(3 << (2 * DIS1_SENSOR_PIN));
    DIS1_SENSOR_PORT->PUPDR &= ~(3 << (2 * DIS1_SENSOR_PIN));
    DIS1_SENSOR_PORT->PUPDR |=  (1 << (2 * DIS1_SENSOR_PIN)); // Pull-up
    SYSCFG->EXTICR[DIS1_SENSOR_EXTICR_INDEX] &= ~(0xF << DIS1_SENSOR_EXTICR_POS);
    SYSCFG->EXTICR[DIS1_SENSOR_EXTICR_INDEX] |=  (0x0 << DIS1_SENSOR_EXTICR_POS); // PA
    EXTI->RTSR |= DIS1_SENSOR_EXTI_LINE;
    EXTI->FTSR |= DIS1_SENSOR_EXTI_LINE;
    EXTI->IMR  |= DIS1_SENSOR_EXTI_LINE;

    // --- Distance 2 Sensor (PB10) ---
    DIS2_SENSOR_PORT->MODER &= ~(3 << (2 * DIS2_SENSOR_PIN));
    DIS2_SENSOR_PORT->PUPDR &= ~(3 << (2 * DIS2_SENSOR_PIN));
    DIS2_SENSOR_PORT->PUPDR |=  (1 << (2 * DIS2_SENSOR_PIN)); // Pull-up
    SYSCFG->EXTICR[DIS2_SENSOR_EXTICR_INDEX] &= ~(0xF << DIS2_SENSOR_EXTICR_POS);
    SYSCFG->EXTICR[DIS2_SENSOR_EXTICR_INDEX] |=  (0x1 << DIS2_SENSOR_EXTICR_POS); // PB
    EXTI->RTSR |= DIS2_SENSOR_EXTI_LINE;
    EXTI->FTSR |= DIS2_SENSOR_EXTI_LINE;
    EXTI->IMR  |= DIS2_SENSOR_EXTI_LINE;

    // NVIC shared interrupt for EXTI lines 4–15
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
}

void StopCauses_ClearAll(void) {
    stop_flags.bluetooth_flag  = 0;
    stop_flags.distance1_flag  = 0;
    stop_flags.distance2_flag  = 0;
}

void EXTI4_15_IRQHandler(void) {
    // --- Bluetooth Disconnect ---
    if (EXTI->PR & BT_DISCONN_EXTI_LINE) {
        EXTI->PR |= BT_DISCONN_EXTI_LINE;
        stop_flags.bluetooth_flag =
            (BT_DISCONN_PORT->IDR & (1 << BT_DISCONN_PIN)) ? 1 : 0;
    }

    // --- Distance 1 Sensor ---
    if (EXTI->PR & DIS1_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS1_SENSOR_EXTI_LINE;
        stop_flags.distance1_flag =
            (DIS1_SENSOR_PORT->IDR & (1 << DIS1_SENSOR_PIN)) ? 1 : 0;
    }

    // --- Distance 2 Sensor ---
    if (EXTI->PR & DIS2_SENSOR_EXTI_LINE) {
        EXTI->PR |= DIS2_SENSOR_EXTI_LINE;
        stop_flags.distance2_flag =
            (DIS2_SENSOR_PORT->IDR & (1 << DIS2_SENSOR_PIN)) ? 1 : 0;
    }
}
