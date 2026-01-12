#ifndef STOPS_H_
#define STOPS_H_

#include "uart_commands.h"
#include <stdint.h>

typedef struct {
    ColorType color;
    uint16_t tiempo;    // segundos
    uint8_t waitFlag;
    uint32_t start_tick;
    uint8_t activo;
} Stop;

#define NUM_PARADAS 3

extern Stop Paradas[NUM_PARADAS];

#endif /* STOPS_H_ */
