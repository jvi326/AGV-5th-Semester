#include <stdint.h>
#include <stm32f051x8.h>

/* ===================== DEFINICIONES ===================== */
#define PIN_MODO_PA6     6u     /* PA6: modo manual seguidor */
#define PIN_ROJO_PB1     1u     /* PB1: lectura rojo */
#define PIN_VERDE_PB2    2u     /* PB2: lectura verde */
#define PIN_AZUL_PB3    3u     /* PB3: lectura azul */

/* ===================== INICIALIZACIÓN ===================== */
static void INPUTS_Init(void)
{
	/* Habilitar reloj para GPIOA y GPIOB */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	/* -------- PA10 como entrada con pull-down -------- */
	GPIOA->MODER &= ~(3u << (PIN_MODO_PA6 * 2));      // input
	GPIOA->PUPDR &= ~(3u << (PIN_MODO_PA6 * 2));      // limpiar
	GPIOA->PUPDR |=  (2u << (PIN_MODO_PA6 * 2));      // pull-down (10)

	/* -------- PB14, PB13, PB12 como entradas con pull-down -------- */
	GPIOB->MODER &= ~((3u << (PIN_ROJO_PB1  * 2)) |
	                  (3u << (PIN_VERDE_PB2 * 2)) |
	                  (3u << (PIN_AZUL_PB3  * 2)));

	GPIOB->PUPDR &= ~((3u << (PIN_ROJO_PB1  * 2)) |
	                  (3u << (PIN_VERDE_PB2 * 2)) |
	                  (3u << (PIN_AZUL_PB3 * 2)));

	GPIOB->PUPDR |=  ((2u << (PIN_ROJO_PB1  * 2)) |   // pull-down (10)
	                  (2u << (PIN_VERDE_PB2 * 2)) |
	                  (2u << (PIN_AZUL_PB3  * 2)));
}

/* ===================== FUNCIONES ===================== */

/* Devuelve 0 o 1 del pin PA10 */
static uint8_t GetModoManual(void)
{
    return ( (GPIOA->IDR & (1u << PIN_MODO_PA6)) ? 1u : 0u );
}

/* Llena un arreglo con {rojo, verde, azul} */
static void GetColorInputs(volatile uint8_t out_states[3])
{
    out_states[0] = (GPIOB->IDR & (1u << PIN_ROJO_PB1))  ? 1u : 0u;
    out_states[1] = (GPIOB->IDR & (1u << PIN_VERDE_PB2)) ? 1u : 0u;
    out_states[2] = (GPIOB->IDR & (1u << PIN_AZUL_PB3))  ? 1u : 0u;
}
