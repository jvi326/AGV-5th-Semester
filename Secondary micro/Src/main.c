#include <stdint.h>
#include <stm32f051x8.h>
#include "uart_commands.h"


uint32_t SystemCoreClock = 8000000;  // define la frecuencia

volatile uint8_t response[4];

/* ================= Config general ================= */
#define VREF_VOLTS      3.3f
/* OKY3066-2 -> 0.100 V/°C ; LM35 “pelón” -> 0.010 V/°C */
#define LM35_V_PER_C    0.100f

/* I2C1 pins */
#define I2C_SCL_PIN     8u  /* PB8 */
#define I2C_SDA_PIN     9u  /* PB9 */

/* ===== SALIDAS LATCH EN PORTC (PC15..PC12) ===== */
#define TEMP_GPIO   GPIOC
#define COLOR_GPIO  GPIOC
#define TEMP_PIN    15u   /* PC15 -> HIGH si tempC > 22 */
#define COLOR_PIN   12u   /* PC12 -> HIGH si colorID > 0 */

/* TCS34725 */
#define TCS34725_ADDRESS        (0x29u << 1)
#define TCS34725_COMMAND_BIT    (0x80u)
#define TCS34725_ENABLE         (0x00u)
#define TCS34725_ENABLE_AEN     (0x02u)
#define TCS34725_ENABLE_PON     (0x01u)
#define TCS34725_ATIME          (0x01u)
#define TCS34725_CONTROL        (0x0Fu)
#define TCS34725_ID             (0x12u)
#define TCS34725_CDATAL         (0x14u)
#define TCS34725_RDATAL         (0x16u)
#define TCS34725_GDATAL         (0x18u)
#define TCS34725_BDATAL         (0x1Au)
#define TCS34725_INTEGRATIONTIME_50MS   (0xEBu)
#define TCS34725_GAIN_4X                (0x01u)

/* =============== Variables (Live Expressions) =============== */
static volatile uint32_t ms_ticks = 0;
static uint8_t  tcs_inited = 0;

volatile float tempC = 0.0f;
volatile int   red = 0, green = 0, blue = 0;
volatile int   colorID = 0;   /* 0=none,1=R,2=G,3=B */

/* ================= Prototipos ================= */
/* SysTick/Delay */
static void SysTick_Init_1ms(void);
void        SysTick_Handler(void);
static void DelayMs(uint32_t ms);

/* I2C1 */
static void I2C1_Init_GPIO_PB8_PB9_AF1(void);
static void I2C1_Init_100kHz(void);
static void I2C1_ClearAllFlags(void);
static void I2C1_WaitFlagSet(uint32_t mask);
static void I2C1_WaitFlagClr(uint32_t mask);
static int  I2C1_WriteBytes(uint8_t addr_w, const uint8_t *data, uint8_t len);
static int  I2C1_WriteThenRead(uint8_t addr_w, const uint8_t *wbuf, uint8_t wlen,
                               uint8_t addr_r, uint8_t *rbuf, uint8_t rlen);

/* TCS34725 */
static void tcs_write8(uint8_t reg, uint32_t value);
static uint8_t  tcs_read8(uint8_t reg);
static uint16_t tcs_read16(uint8_t reg);
static void     TCS34725_Enable(void);
static void     TCS34725_Begin(uint8_t atime, uint8_t gain);
static void     TCS34725_GetRaw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
static void     TCS34725_GetRGB(int *R, int *G, int *B);

/* ADC (PB0/IN8) */
static void     ADC_init_PB0(void);
static uint16_t ADC_read_once(void);
static float    readTemp(uint8_t samples);

volatile uint8_t bytes_in_terminal;

/* Utils */
static inline float adc_to_volts(uint16_t raw) { return (raw * VREF_VOLTS) / 4095.0f; }
static void delay_cycles(volatile uint32_t n){ while(n--) __asm__("nop"); }

/* ================= Implementación ================= */
void SysTick_Handler(void){ ms_ticks++; }
static void SysTick_Init_1ms(void)
{
    SysTick->LOAD = 48000000u/1000u - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
static void DelayMs(uint32_t ms){ uint32_t s=ms_ticks; while((ms_ticks-s)<ms){ __NOP(); } }
/* ---- I2C1 ---- */
static void I2C1_ClearAllFlags(void)
{
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
                I2C_ICR_ARLOCF | I2C_ICR_OVRCF  | I2C_ICR_TIMOUTCF | I2C_ICR_PECCF;
}
static void I2C1_WaitFlagSet(uint32_t mask){ while ((I2C1->ISR & mask) == 0) { __NOP(); } }
static void I2C1_WaitFlagClr(uint32_t mask){ while ((I2C1->ISR & mask) != 0) { __NOP(); } }
static int I2C1_WriteBytes(uint8_t addr_w, const uint8_t *data, uint8_t len)
{
    I2C1_ClearAllFlags();
    I2C1_WaitFlagClr(I2C_ISR_BUSY);
    I2C1->CR2 = ((uint32_t)addr_w & I2C_CR2_SADD) |
                (0u << I2C_CR2_RD_WRN_Pos) |
                ((uint32_t)len << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;
    for(uint8_t i=0;i<len;i++){ I2C1_WaitFlagSet(I2C_ISR_TXIS); I2C1->TXDR = data[i]; }
    I2C1_WaitFlagSet(I2C_ISR_STOPF);
    I2C1_ClearAllFlags();
    return 0;
}
static int I2C1_WriteThenRead(uint8_t addr_w, const uint8_t *wbuf, uint8_t wlen,
                              uint8_t addr_r, uint8_t *rbuf, uint8_t rlen)
{
    I2C1_ClearAllFlags();
    I2C1_WaitFlagClr(I2C_ISR_BUSY);
    I2C1->CR2 = ((uint32_t)addr_w & I2C_CR2_SADD) |
                (0u << I2C_CR2_RD_WRN_Pos) |
                ((uint32_t)wlen << I2C_CR2_NBYTES_Pos) |
                0 |
                I2C_CR2_START;
    for(uint8_t i=0;i<wlen;i++){ I2C1_WaitFlagSet(I2C_ISR_TXIS); I2C1->TXDR = wbuf[i]; }
    I2C1_WaitFlagSet(I2C_ISR_TC);
    I2C1->CR2 = ((uint32_t)addr_r & I2C_CR2_SADD) |
                I2C_CR2_RD_WRN |
                ((uint32_t)rlen << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;
    for(uint8_t i=0;i<rlen;i++){ I2C1_WaitFlagSet(I2C_ISR_RXNE); rbuf[i] = (uint8_t)I2C1->RXDR; }
    I2C1_WaitFlagSet(I2C_ISR_STOPF);
    I2C1_ClearAllFlags();
    return 0;
}
static void I2C1_Init_GPIO_PB8_PB9_AF1(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~((3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2)));
    GPIOB->MODER |=  ((2u<<(I2C_SCL_PIN*2)) | (2u<<(I2C_SDA_PIN*2)));
    GPIOB->OTYPER |= (1u<<I2C_SCL_PIN) | (1u<<I2C_SDA_PIN);
    GPIOB->OSPEEDR |= (3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2));
    GPIOB->PUPDR &= ~((3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2)));
    GPIOB->PUPDR |=  ((1u<<(I2C_SCL_PIN*2)) | (1u<<(I2C_SDA_PIN*2)));
    GPIOB->AFR[1] &= ~((0xFu<<((I2C_SCL_PIN-8u)*4)) | (0xFu<<((I2C_SDA_PIN-8u)*4)));
    GPIOB->AFR[1] |=  ((0x1u<<((I2C_SCL_PIN-8u)*4)) | (0x1u<<((I2C_SDA_PIN-8u)*4)));
}
static void I2C1_Init_100kHz(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST; RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 &= ~I2C_CR1_ANFOFF;
    I2C1->CR1 &= ~I2C_CR1_DNF;
    I2C1->TIMINGR = 0x2000090E; /* ~100 kHz @48 MHz */
    I2C1->CR1 |= I2C_CR1_PE;
}

/* ---- TCS34725 ---- */
static void tcs_write8(uint8_t reg, uint32_t value)
{
    uint8_t tx[2] = { (uint8_t)(TCS34725_COMMAND_BIT | reg), (uint8_t)(value & 0xFFu) };
    (void)I2C1_WriteBytes(TCS34725_ADDRESS, tx, 2);
}
static uint8_t tcs_read8(uint8_t reg)
{
    uint8_t cmd = (uint8_t)(TCS34725_COMMAND_BIT | reg), v=0;
    (void)I2C1_WriteThenRead(TCS34725_ADDRESS, &cmd, 1, TCS34725_ADDRESS, &v, 1);
    return v;
}
static uint16_t tcs_read16(uint8_t reg)
{
    uint8_t cmd = (uint8_t)(TCS34725_COMMAND_BIT | reg), rx[2]={0,0};
    (void)I2C1_WriteThenRead(TCS34725_ADDRESS, &cmd, 1, TCS34725_ADDRESS, rx, 2);
    return (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
}
static void TCS34725_Enable(void)
{
    tcs_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON); DelayMs(3);
    tcs_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN); DelayMs(50);
}
static void TCS34725_Begin(uint8_t atime, uint8_t gain)
{
    uint8_t id = tcs_read8(TCS34725_ID);
    if ((id != 0x44u) && (id != 0x4Du) && (id != 0x10u)) { tcs_inited = 0; return; }
    tcs_inited = 1;
    tcs_write8(TCS34725_ATIME, atime);
    tcs_write8(TCS34725_CONTROL, gain);
    TCS34725_Enable();
}
static void TCS34725_GetRaw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    if (!tcs_inited) { *r=*g=*b=*c=0; return; }
    *c = tcs_read16(TCS34725_CDATAL);
    *r = tcs_read16(TCS34725_RDATAL);
    *g = tcs_read16(TCS34725_GDATAL);
    *b = tcs_read16(TCS34725_BDATAL);
    DelayMs(50);
}
static void TCS34725_GetRGB(int *R, int *G, int *B)
{
    uint16_t r,g,b,c; TCS34725_GetRaw(&r,&g,&b,&c);
    if (c == 0) { *R=*G=*B=0; return; }
    int Ri = (int)((uint32_t)r*255u/c);
    int Gi = (int)((uint32_t)g*255u/c);
    int Bi = (int)((uint32_t)b*255u/c);
    if (Ri>255) Ri=255;
    if (Gi>255) Gi=255;
    if (Bi>255) Bi=255;
    *R = Ri; *G = Gi; *B = Bi;
}

/* ---- ADC PB0 ---- */
static void ADC_init_PB0(void)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    GPIOB->MODER  |=  (3u << (0 * 2));
    GPIOB->PUPDR  &= ~(3u << (0 * 2));
    RCC->CR2 |= RCC_CR2_HSI14ON; while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) { }
    if (ADC1->CR & ADC_CR_ADEN){ ADC1->CR |= ADC_CR_ADDIS; while (ADC1->CR & ADC_CR_ADEN) { } }
    ADC1->CR |= ADC_CR_ADCAL; while (ADC1->CR & ADC_CR_ADCAL) { }
    ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0;
    ADC1->CHSELR = ADC_CHSELR_CHSEL8;
    ADC1->CR |= ADC_CR_ADEN; while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) { }
    (void)ADC_read_once();
}
static uint16_t ADC_read_once(void)
{
    if (ADC1->ISR & ADC_ISR_EOC) (void)ADC1->DR;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) { }
    return (uint16_t)ADC1->DR;
}
static float readTemp(uint8_t samples)
{
    if (samples == 0) samples = 1;
    uint32_t acc=0; for(uint8_t i=0;i<samples;i++){ acc += ADC_read_once(); delay_cycles(1500); }
    float meanCnt = (float)acc / (float)samples;
    float vV = adc_to_volts((uint16_t)meanCnt);
    return vV / LM35_V_PER_C; /* °C */
}

/* ==== SALIDAS LATCH EN PORTC (TEMP=PC15, COLOR=PC12) ==== */
static void OUTS_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    /* TEMP_OUT (PC15) */
    TEMP_GPIO->MODER  &= ~(3u<<(TEMP_PIN*2));
    TEMP_GPIO->MODER  |=  (1u<<(TEMP_PIN*2));   /* 01: output */
    TEMP_GPIO->OTYPER &= ~(1u<<TEMP_PIN);
    TEMP_GPIO->OSPEEDR |= (3u<<(TEMP_PIN*2));
    TEMP_GPIO->PUPDR  &= ~(3u<<(TEMP_PIN*2));
    TEMP_GPIO->BRR = (1u<<TEMP_PIN);           /* inicia LOW */

    /* COLOR_OUT (PC12) */
    COLOR_GPIO->MODER  &= ~(3u<<(COLOR_PIN*2));
    COLOR_GPIO->MODER  |=  (1u<<(COLOR_PIN*2)); /* 01: output */
    COLOR_GPIO->OTYPER &= ~(1u<<COLOR_PIN);
    COLOR_GPIO->OSPEEDR |= (3u<<(COLOR_PIN*2));
    COLOR_GPIO->PUPDR  &= ~(3u<<(COLOR_PIN*2));
    COLOR_GPIO->BRR = (1u<<COLOR_PIN);          /* inicia LOW */
}

/* Helpers de escritura rápida */
static inline void TEMP_OUT_HIGH(void){ TEMP_GPIO->BSRR = (1u<<TEMP_PIN); }
static inline void TEMP_OUT_LOW (void){ TEMP_GPIO->BRR  = (1u<<TEMP_PIN); }
static inline void COLOR_OUT_HIGH(void){ COLOR_GPIO->BSRR = (1u<<COLOR_PIN); }
static inline void COLOR_OUT_LOW (void){ COLOR_GPIO->BRR  = (1u<<COLOR_PIN); }

/* ===================== MAIN ===================== */
int main(void)
{
     SysTick_Init_1ms();

    /* I2C + TCS34725 */
    I2C1_Init_GPIO_PB8_PB9_AF1();
    I2C1_Init_100kHz();
    TCS34725_Begin(0xEBu, 0x01u); /* ~50 ms, 4x */

    /* ADC PB0 (LM35/OKY3066-2) */
    ADC_init_PB0();

    /*SALIDAS LATCH */
    OUTS_Init();

    /*USART1 INIT */
    USART1_Init();

    while (1)
    {
        /* Lecturas */
        tempC = readTemp(16);
        TCS34725_GetRGB((int*)&red, (int*)&green, (int*)&blue);

        /* Clasificador simple */
        if (red   > 110 && green < 100 && blue  < 100) colorID = 1;
        else if (green > 97 && blue  < 100 && red   < 100) colorID = 2;
        else if (blue  >  80 && red   < 100 && green < 100) colorID = 3;
        else colorID = 0;

        /* ======= SALIDAS LATCH ======= */
        /* TEMP_OUT (PC15): HIGH si tempC > 22.0 */
        if (tempC > 25.0f) {
            TEMP_OUT_HIGH();
            //USART1_WriteFloatCelsius(tempC);
        } else {
            TEMP_OUT_LOW();
        }

        /* COLOR_OUT (PC12): HIGH si hay color detectado */
        if (colorID > 0) {
            COLOR_OUT_HIGH();
            //USART1_WriteColorID(colorID);
        } else {
            COLOR_OUT_LOW();
        }
        /* ============================= */

        //DelayMs(5);

		bytes_in_terminal = USART1_AvailableBytes();

        if (bytes_in_terminal >= 2) {
            uint8_t cmd     = USART1_ReadByte();
            uint8_t sub_cmd = USART1_ReadByte();


            response[0] = ACKNOWLEDGE;
            response[1] = cmd;
            response[2] = sub_cmd;

            // --- Responder según el comando recibido ---
            if (cmd == C_TEMPERATURE && sub_cmd == RETRIEVE_TEMP) {
            	uint8_t tempInt = (uint8_t)tempC; // → 23
            	response[3] = tempInt;

            	USART1_SendByte(response[0]);
            	USART1_SendByte(response[1]);
            	USART1_SendByte(response[2]);
            	USART1_SendByte(response[3]);
            }
            else if (cmd == C_COLOR && sub_cmd == RETRIEVE_COLOR) {
            	switch (colorID) {
					case 1:
						response[3] = (uint8_t)COLOR_RED;
						break;
					case 2:
						response[3] = (uint8_t)COLOR_GREEN;
						break;
					case 3:
						response[3] = (uint8_t)COLOR_BLUE;
						break;
					default:
						response[3] = (uint8_t)COLOR_INVALID;
						break;
				}
            	USART1_SendByte(response[0]);
            	USART1_SendByte(response[1]);
            	USART1_SendByte(response[2]);
            	USART1_SendByte(response[3]);
            }
        }
    }
}
