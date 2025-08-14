#include "stm32f051x8.h"
#include "servos.h"

#define SERVO_MIN_PULSE 500    // 0.5 ms in us
#define SERVO_MAX_PULSE 2500   // 2.5 ms in us
#define TIMER_FREQ 48000000    // 48 MHz system clock
#define PWM_FREQ 50            // 50 Hz servo signal
#define PWM_PERIOD (TIMER_FREQ / PWM_FREQ)  // 20 ms

static uint8_t angle_min = 0;
static uint8_t angle_max = 180;
static uint8_t step_deg = 1;
static uint8_t current_angle = 0;
static uint8_t direction = 1;

static uint16_t AngleToCCR(uint8_t angle) {
    uint16_t pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180;
    return (pulse * PWM_PERIOD) / 20000; // Convert us to CCR value
}

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;

        TIM15->CCR1 = AngleToCCR(current_angle);
        TIM15->CCR2 = AngleToCCR(current_angle);

        if (direction)
            current_angle += step_deg;
        else
            current_angle -= step_deg;

        if (current_angle >= angle_max) {
            current_angle = angle_max;
            direction = 0;
        } else if (current_angle <= angle_min) {
            current_angle = angle_min;
            direction = 1;
        }
    }
}

void Servo_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN | RCC_APB1ENR_TIM14EN;

    // PB14 = TIM15_CH1, PB15 = TIM15_CH2
    GPIOB->MODER &= ~((3 << (14 * 2)) | (3 << (15 * 2)));
    GPIOB->MODER |= (2 << (14 * 2)) | (2 << (15 * 2));
    GPIOB->AFR[1] |= (0 << ((14 - 8) * 4)) | (0 << ((15 - 8) * 4)); // AF0

    // Timer 15 PWM setup
    TIM15->PSC = 0;
    TIM15->ARR = PWM_PERIOD - 1;
    TIM15->CCR1 = AngleToCCR(0);
    TIM15->CCR2 = AngleToCCR(0);
    TIM15->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM15->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM15->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // Timer 14 interrupt timer (used to update angle)
    NVIC_EnableIRQ(TIM14_IRQn);
}

void Servo_SetAngleRange(uint8_t min_deg, uint8_t max_deg) {
    angle_min = min_deg;
    angle_max = max_deg;
    current_angle = min_deg;
}

void Servo_SetStep(uint8_t step) {
    step_deg = step;
}

void Servo_SetTimeStep(uint16_t ms) {
    TIM14->PSC = 48000 - 1; // 1 ms tick
    TIM14->ARR = ms - 1;
}

void Servo_StartSweep(void) {
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_CEN;
}
