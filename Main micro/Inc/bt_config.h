#ifndef BT_CONFIG_H
#define BT_CONFIG_H
#include <stdint.h>   // <--- NECESARIO para uint8_t, uint16_t

int BT_ApplyCsvConfig_fromRxBuf(const volatile uint8_t* rx, uint16_t nbytes);
#endif
