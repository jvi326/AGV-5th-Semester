#include "stops.h"
#include "stm32f051x8.h"

static volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void) {
    ms_ticks++;
}

uint32_t GetTick(void) {
    return ms_ticks;
}

void SysTick_Init(void) {
    // Configura SysTick para generar interrupción cada 1 ms
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1 << 0) | (1 << 1) | (1 << 2); // ENABLE | TICKINT | CLKSOURCE
}

Stop Paradas[NUM_PARADAS] = {
    { COLOR_RED,   10, 0, 0 },
    { COLOR_GREEN, 10, 0, 0 },
	{ COLOR_BLUE,  10, 0, 0 }
};

uint8_t currentStopIndex = 0;

void Stop_Start(Stop* parada) {
    parada->start_tick = GetTick();
    parada->activo = 1;
}

uint8_t Stop_Expired(Stop* parada) {
    if (!parada->activo) return 0;
    if ((GetTick() - parada->start_tick) >= (parada->tiempo * 1000)) {
        parada->activo = 0;
        return 1;
    }
    return 0;
}
