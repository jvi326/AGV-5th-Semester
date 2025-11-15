#include <stdint.h>
#include <stm32f051x8.h>

/* ===================== DEFINICIONES ===================== */
#define PIN_MODO_PA10     10u     /* PA10: modo manual seguidor */
#define PIN_ROJO_PB14     14u     /* PB14: lectura rojo */
#define PIN_VERDE_PB13    13u     /* PB13: lectura verde */
#define PIN_AZUL_PB12     12u     /* PB12: lectura azul */

/* ===================== INICIALIZACIÓN ===================== */
static void INPUTS_Init(void)
{
    /* Habilitar reloj para GPIOA y GPIOB */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* -------- PA10 (modo) como entrada -------- */
    GPIOA->MODER &= ~(3u << (PIN_MODO_PA10 * 2));   /* 00 = input */
    GPIOA->PUPDR &= ~(3u << (PIN_MODO_PA10 * 2));   /* sin pull */
    GPIOA->PUPDR |=  (1u << (PIN_MODO_PA10 * 2));   /* pull-up */

    /* -------- PB14, PB13, PB12 como entradas -------- */
    GPIOB->MODER &= ~((3u << (PIN_ROJO_PB14  * 2)) |
                      (3u << (PIN_VERDE_PB13 * 2)) |
                      (3u << (PIN_AZUL_PB12  * 2)));

    /* pull-up para evitar flotación */
    GPIOB->PUPDR &= ~((3u << (PIN_ROJO_PB14  * 2)) |
                      (3u << (PIN_VERDE_PB13 * 2)) |
                      (3u << (PIN_AZUL_PB12  * 2)));

    GPIOB->PUPDR |=  ((1u << (PIN_ROJO_PB14  * 2)) |
                      (1u << (PIN_VERDE_PB13 * 2)) |
                      (1u << (PIN_AZUL_PB12  * 2)));
}

/* ===================== FUNCIONES ===================== */

/* Devuelve 0 o 1 del pin PA10 */
static uint8_t GetModoManual(void)
{
    return ( (GPIOA->IDR & (1u << PIN_MODO_PA10)) ? 0u : 1u );
}

/* Llena un arreglo con {rojo, verde, azul} */
static void GetColorInputs(volatile uint8_t out_states[3])
{
    out_states[0] = (GPIOB->IDR & (1u << PIN_ROJO_PB14))  ? 0u : 1u;
    out_states[1] = (GPIOB->IDR & (1u << PIN_VERDE_PB13)) ? 0u : 1u;
    out_states[2] = (GPIOB->IDR & (1u << PIN_AZUL_PB12))  ? 0u : 1u;
}
