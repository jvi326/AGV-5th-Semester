#include <stdint.h>
#include "stm32f051x8.h"
#include "control.h"

/* ===== Config botón: PC13 como entrada (pull-up), activo-bajo ===== */
static void Button_PC13_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    // PC13 input
    GPIOC->MODER  &= ~(3u << (13*2));   // 00: input
    GPIOC->PUPDR  &= ~(3u << (13*2));
    GPIOC->PUPDR  |=  (1u << (13*2));   // 01: pull-up
}

static uint8_t button_read_raw(void)
{
    // Activo-bajo: 0 = PRESIONADO
    return ( (GPIOC->IDR & (1u<<13)) == 0 ) ? 1 : 0;
}

/* Antirrebote + flanco */
static uint8_t button_was_down = 0;
static uint32_t button_last_ms = 0;

static uint8_t button_pressed_edge(void)
{
    uint32_t now = millis();
    uint8_t raw = button_read_raw();

    if (raw != button_was_down) {
        // cambio; espera 20 ms antes de aceptar
        if ((now - button_last_ms) >= 20) {
            button_last_ms = now;
            button_was_down = raw;
            if (raw) {
                // flanco de bajada (se presionó)
                return 1;
            }
        }
    }
    return 0;
}

int main(void)
{
    control_init();        // Tu init (encoders, PWM, SysTick)
    Button_PC13_Init();    // Botón

    uint8_t  run_enabled = 0;
    uint32_t run_deadline = 0;

    while (1) {

    	// 1) ¿Se presionó el botón? → arrancar 10 s
    	if (button_pressed_edge()) {
    	    control_reset_all();               // <—— LIMPIA TODO ANTES DE EMPEZAR
    	    run_enabled = 1;
    	    run_deadline = millis() + 1000u;  // 10 s
    	}

    	// 2) Si estamos corriendo, actualizar control...
    	if (run_enabled) {
    	    control_update();

    	    // 3) ¿se acabó la ventana de 10 s?
    	    if ((int32_t)(millis() - run_deadline) >= 0) {
    	        run_enabled = 0;
    	        control_reset_all();           // <—— SE DETIENE Y QUEDA LIMPIO
    	    }
    	} else {
    	    control_coast(); // opcional mantener, ya que reset_all() dejó todo en 0
    	}
    }
}

