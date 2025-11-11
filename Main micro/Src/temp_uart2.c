#include "temp_uart2.h"
#include "stm32f051x8.h"

/* ====== Constantes/defines internos ====== */
#define VREF_VOLTS      3.3f
/* OKY3066-2 -> 0.100 V/°C ; LM35 “pelón” -> 0.010 V/°C */
#define LM35_V_PER_C    0.100f

/* ADC en PB0 = ADC1_IN8 */
#define LM35_GPIO   GPIOB
#define LM35_PIN    0u

/* ---- Prototipos internos ---- */
static inline float adc_to_volts(uint16_t raw) { return (raw * VREF_VOLTS) / 4095.0f; }

/* SysTick */
static void     SysTick_Init_1ms(void);
static uint32_t millis(void);

/* ADC */
static void     ADC_init_PB0(void);
static uint16_t ADC_read_once(void);
static float    readTempC(uint8_t samples);



/* ====== Variables ====== */
static volatile uint32_t ms_ticks = 0;
volatile float tempC = 0.0f;

static uint32_t g_period_ms = TEMP_TASK_PERIOD_MS;
static float    g_thresholdC = TEMP_THRESHOLD_C;
static uint8_t  g_samples = TEMP_SAMPLES;

/* ====== Declaraciones externas de tus funciones USART2 ====== */
extern void USART2_SendString(const char *str);
extern void USART2_SendFloat(float value, uint8_t decimalPlaces);

static void SysTick_Init_1ms(void)
{
    SysTick->LOAD = (SYSCLK_HZ/1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t millis(void){ return ms_ticks; }

/* ---- ADC PB0 ---- */
static void ADC_init_PB0(void)
{
    /* PB0 en analógico */
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    LM35_GPIO->MODER  |=  (3u << (LM35_PIN * 2));
    LM35_GPIO->PUPDR  &= ~(3u << (LM35_PIN * 2));

    /* ADC clock HSI14 */
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) { }

    /* Calibración */
    if (ADC1->CR & ADC_CR_ADEN){ ADC1->CR |= ADC_CR_ADDIS; while (ADC1->CR & ADC_CR_ADEN) { } }
    ADC1->CR |= ADC_CR_ADCAL; while (ADC1->CR & ADC_CR_ADCAL) { }

    /* Muestreo largo para estabilidad */
    ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0; /* 239.5 ciclos */

    /* Canal IN8 (PB0) */
    ADC1->CHSELR = ADC_CHSELR_CHSEL8;

    /* Habilitar ADC */
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) { }

    /* Lectura dummy */
    (void)ADC_read_once();
}

static uint16_t ADC_read_once(void)
{
    if (ADC1->ISR & ADC_ISR_EOC) (void)ADC1->DR;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) { }
    return (uint16_t)ADC1->DR;
}

static float readTempC(uint8_t samples)
{
    if (samples == 0) samples = 1;
    uint32_t acc = 0;
    for(uint8_t i=0;i<samples;i++){
        acc += ADC_read_once();
    }
    float meanCnt = (float)acc / (float)samples;
    float vV = adc_to_volts((uint16_t)meanCnt);
    return vV / LM35_V_PER_C; /* °C */
}



/* ====== API ====== */
void Temp_Init(void)
{
    SysTick_Init_1ms();
    ADC_init_PB0();
}

void Temp_Task_200ms(void)
{
    static uint32_t t0 = 0;
    uint32_t now = millis();
    if ((now - t0) < g_period_ms){
		t0 = now;

		tempC = readTempC(g_samples) + 5;

        // entero en °C con redondeo
  //      int32_t tInt = (int32_t)(tempC + (tempC >= 0 ? 0.5f : -0.5f));

  //      char buffer[12];
  //      sprintf(buffer, "%ld\n", (long)tInt); // solo entero + LF
  //      USART2_SendString(buffer);

		int entero = (int)tempC;
		int decimales = (int)((tempC - entero) * 100);
		char buffer[16];

		sprintf(buffer, "%d.%02d\n", entero, decimales);
		USART2_SendString(buffer);
    }
}

/* ====== Setters opcionales ====== */
void Temp_SetPeriodMs(uint32_t period_ms)
{
    if (period_ms == 0) period_ms = 1;
    g_period_ms = period_ms;
}
void Temp_SetThreshold(float thC) { g_thresholdC = thC; }
void Temp_SetSamples(uint8_t samples)
{
    if (samples == 0) samples = 1;
    g_samples = samples;
}
void Temp_Tick1ms(void) { ms_ticks++; }

