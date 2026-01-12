#ifndef TEMP_UART2_H
#define TEMP_UART2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ===== Config por defecto (puedes redefinir antes de incluir el .h) ===== */
#ifndef SYSCLK_HZ
#define SYSCLK_HZ         8000000u      /* 8 MHz por defecto; pon 48000000u si vas a 48 MHz */
#endif

#ifndef TEMP_TASK_PERIOD_MS
#define TEMP_TASK_PERIOD_MS  200u       /* Periodicidad de envío por UART2 */
#endif

#ifndef TEMP_THRESHOLD_C
#define TEMP_THRESHOLD_C   25.0f        /* Umbral para PC15 HIGH */
#endif

#ifndef TEMP_SAMPLES
#define TEMP_SAMPLES       16u          /* Promediado de lecturas ADC */
#endif

/* ==== API pública ==== */
/* Llama una vez tras inicializar tu USART2 */
void Temp_Init(void);

/* Llama periódicamente (en el while(1)) — internamente respeta TEMP_TASK_PERIOD_MS */
void Temp_Task_200ms(void);

/* Opcionales de configuración en runtime */
void Temp_SetPeriodMs(uint32_t period_ms);
void Temp_SetThreshold(float thC);
void Temp_SetSamples(uint8_t samples);
void Temp_Tick1ms(void);   // hook de 1 ms


/* Variable de lectura actual (útil para Live Expressions) */
extern volatile float tempC;

#ifdef __cplusplus
}
#endif

#endif /* TEMP_UART2_H */
