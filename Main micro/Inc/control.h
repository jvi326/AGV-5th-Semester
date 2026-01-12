#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "stm32f051x8.h"

/* ===== API ===== */
void control_init(void);          // Inicializa encoders, PWM/DIR y SysTick (como en tu código)
void control_update(void);        // Ejecuta UNA iteración de tu control cada 100 ms
void control_coast(void);         // Apaga PWM y pone INA/INB en 0
void control_reset_start_gate(void); // Reinicia el “arranque coordinado”

void control_reset_all(void);   // Apaga, limpia integradores/filtros/gate y zerea CNT

/* (Opcional) Setpoints accesibles desde main si quieres cambiarlos */
extern float rpm_des1;
extern float rpm_des2;

#endif /* CONTROL_H */
