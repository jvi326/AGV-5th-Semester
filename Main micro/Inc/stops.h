#ifndef STOPS_H_
#define STOPS_H_

#include "uart_commands.h"
#include <stdint.h>

typedef struct {
    ColorType color;
    uint16_t tiempo;    // segundos
    uint32_t start_tick;
    uint8_t activo;
} Stop;

#define NUM_PARADAS 3

extern Stop Paradas[NUM_PARADAS];
extern uint8_t currentStopIndex;

void Stop_Start(Stop* parada);
uint8_t Stop_Expired(Stop* parada);
void SysTick_Init(void);
uint32_t GetTick(void);

#endif /* STOPS_H_ */
