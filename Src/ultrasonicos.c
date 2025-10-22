#include "ultrasonicos.h"
#include "Bluetooth_USART2.h"
#include <stdbool.h>

static volatile uint32_t echo_start = 0, echo_end = 0;
volatile float distance1_cm = 0.0f;
volatile bool distance1_ready = false;

static void Init_GPIO(void);
static void Init_TIM6(void);
static void Init_EXTI_ECHO1(void);
static void Trigger_HCSR04(uint8_t trig_pin);

/* ---------- Initialization ---------- */
void Ultrasonicos_Init(void)
{
    Init_GPIO();
    Init_TIM6();
    Init_EXTI_ECHO1();
    SysTick_Config(48000); // 1 ms tick
}

void Ultrasonicos_MeasureAll(void) {
	Ultrasonicos_Measure1();
}

/* ---------- Trigger and wait for ready flag ---------- */
void Ultrasonicos_Measure1(void)
{
    distance1_ready = false;
    Trigger_HCSR04(TRIG1_PIN);
}

/* ---------- Returns distance when ready ---------- */
float Ultrasonicos_GetDistance1(void)
{
    if (!distance1_ready) return -1.0f; // Not ready yet
    return distance1_cm;
}

/* ---------- GPIO Setup ---------- */
static void Init_GPIO(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // TRIG1 (PB5) output
    GPIOB->MODER |= (1U << (TRIG1_PIN * 2));

    // ECHO1 (PB6) input with pull-down
    GPIOB->MODER &= ~(3U << (ECHO1_PIN * 2));
    GPIOB->PUPDR |=  (2U << (ECHO1_PIN * 2)); // pull-down
}

/* ---------- TIM6 as microsecond timer ---------- */
static void Init_TIM6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 8 - 1; // 48 MHz / 8 = 6 MHz → 0.166 µs per tick
    TIM6->ARR = 0xFFFF;
    TIM6->CR1 |= TIM_CR1_CEN;
}

/* ---------- Configure EXTI on PB6 ---------- */
static void Init_EXTI_ECHO1(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI6;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;

    EXTI->IMR  |= (1U << ECHO1_PIN);       // Unmask line
    EXTI->RTSR |= (1U << ECHO1_PIN);       // Rising edge
    EXTI->FTSR |= (1U << ECHO1_PIN);       // Falling edge

    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* ---------- 10 µs trigger pulse ---------- */
static void Trigger_HCSR04(uint8_t trig_pin)
{
    GPIOB->BRR  = (1U << trig_pin);
    TIM6->CNT = 0;
    GPIOB->BSRR = (1U << trig_pin);
    while (TIM6->CNT < 60); // ~10 µs
    GPIOB->BRR = (1U << trig_pin);
}

/* ---------- EXTI interrupt handler ---------- */
void EXTI4_15_IRQHandler(void)
{
    /* ---- Handle Ultrasonic Echo on PB6 ---- */
    if (EXTI->PR & (1U << ECHO1_PIN)) // ECHO1_PIN = 6
    {
        EXTI->PR = (1U << ECHO1_PIN); // clear flag

        if (GPIOB->IDR & (1U << ECHO1_PIN))
        {
            // Rising edge → start timing
            echo_start = TIM6->CNT;
        }
        else
        {
            // Falling edge → stop timing
            echo_end = TIM6->CNT;

            uint32_t dt = (echo_end >= echo_start)
                ? (echo_end - echo_start)
                : (0x10000 - echo_start + echo_end);

            distance1_cm = (dt * 0.0343f) / 2.0f;  // Convert to cm
            distance1_ready = true;
        }
    }

    /* ---- Handle Bluetooth State on PA7 ---- */
    if (EXTI->PR & (1U << 7)) // PA7 interrupt
    {
        EXTI->PR = (1U << 7); // clear flag

        if (GPIOA->IDR & (1U << 7))
        {
            // Rising edge → Bluetooth reconnected
            flag_Bluetooth_state = 1;
        }
        else
        {
            // Falling edge → Bluetooth disconnected
            flag_Bluetooth_state = 0;
        }
    }
}

