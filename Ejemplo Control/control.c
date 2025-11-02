#include <stdint.h>
#include "stm32f051x8.h"
#include "control.h"

/* ================== Prototipos (tuyos) ================== */
void TIM1_Encoder_Init(void);   // Motor 1: PA8(A), PA9(B)
void TIM2_Encoder_Init(void);   // Motor 2: PA0(A), PA1(B)
void PWM_init_both(void);       // PC8=TIM3_CH3 (M2), PC9=TIM3_CH4 (M1) + DIR pins

void SysTick_Handler(void);
uint32_t millis(void);
void delay(uint32_t ms);

void setMotor1PWM1(int16_t pwm_value); // M1 Atrás
void setMotor1PWM2(int16_t pwm_value); // M1 Adelante
void setMotor2PWM1(int16_t pwm_value); // M2 Adelante
void setMotor2PWM2(int16_t pwm_value); // M2 Atrás

/* ================== SysTick utils (UNA sola vez) ================== */
volatile uint32_t tick_count = 0;
uint32_t millis(void){ return tick_count; }
void SysTick_Handler(void){ tick_count++; }
void delay(uint32_t ms){ uint32_t t = millis(); while ((millis()-t) < ms) { __NOP(); } }

/* ================== Globals control ================== */
uint32_t last_time = 0;
const  uint32_t intervalo = 100; // ms

/* ===== Parámetros comunes ===== */
const float PPR   = 1548.0f;     // pulsos por revolución
const uint16_t DUTY_MIN  = 40;   // ~4% de 1023 (piso normal del control)

/* ===== Arranque coordinado ===== */
#define START_KICK_PWM   200   // patada fuerte común (180–260 según fricción)
#define HOLD_PWM          80   // sostén para el que ya giró (60–120)
#define START_THR_RPM      5   // umbral para considerar “ya gira” (3–8)
#define START_KICKS        3   // ciclos de patada simultánea (3*100ms = 300ms)
#define START_TIMEOUT     40   // 40*100ms = 4s (salida de seguridad del gate)

uint8_t  start_gate      = 1;     // 1=estamos en arranque coordinado
uint8_t  start_kicks_ctr = START_KICKS;
uint16_t start_timeout   = START_TIMEOUT;

/* ===== Setpoint de prueba ===== */
float rpm_des1 = 120.0f;  // + adelante, - atrás
float rpm_des2 = 120.0f;

/* ===== Motor 1 (TIM1 + PC10/PC11/PC9) ===== */
int16_t counter1, deltaNp1;
float   rpm1 = 0.0f, rpm_f1 = 0.0f;
float   e1 = 0.0f, inte1 = 0.0f, u1 = 0.0f;
const float kp1 = 10.0f, beta1 = 2.0f, b_est1 = 33.300268f;

/* ===== Motor 2 (TIM2 + PC6/PC7/PC8) ===== */
int16_t counter2, deltaNp2;
float   rpm2 = 0.0f, rpm_f2 = 0.0f;
float   e2 = 0.0f, inte2 = 0.0f, u2 = 0.0f;
const float kp2 = 8.0f,  beta2 = 2.0f, b_est2 = 26.300268f;

/* ====== Parche A: Anti-glitch encoders + filtro suave ====== */
float last_rpm1 = 0.0f, last_rpm2 = 0.0f;
const float DROP_FRAC = 0.5f;        // ignora caídas >50% si venías >10 rpm
const float MIN_RPM_VALID = 10.0f;
const float ALPHA = 0.4f;            // filtro exponencial (más suave que 0.7)

/* ====== Parche B: PI de rectitud (trim en duty) ====== */
float syncKp = 2.0f;                 // duty por rpm de diferencia
float syncKi = 0.2f;                 // duty por rpm*s
float syncInt = 0.0f;
const float syncDt = 0.1f;           // 100 ms
const float syncIlim = 400.0f;

/* ====== Parche C: Slew-rate (rampa de duty) ====== */
const int16_t SLEW_STEP = 25;        // max cambio de duty por ciclo
int16_t last_out1 = 0, last_out2 = 0;
static inline int16_t slew(int16_t prev, int16_t tgt){
    int16_t d = tgt - prev;
    if (d >  SLEW_STEP) d =  SLEW_STEP;
    if (d < -SLEW_STEP) d = -SLEW_STEP;
    return prev + d;
}

/* ================== Prototipos “privados” de bajo nivel (tuyos) ================== */
void TIM1_Encoder_Init(void);
void TIM2_Encoder_Init(void);
void PWM_init_both(void);

/* ================== Implementación API ================== */
void control_init(void)
{
    TIM1_Encoder_Init();    // M1 encoder
    TIM2_Encoder_Init();    // M2 encoder
    PWM_init_both();        // TIM3 CH3/CH4 + pines dirección

    // SysTick 1 ms @ 8 MHz (igual que tu main)
    SysTick->LOAD = 8000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;

    last_time = millis();
}

void control_reset_start_gate(void)
{
    start_gate      = 1;
    start_kicks_ctr = START_KICKS;
    start_timeout   = START_TIMEOUT;
    // También reiniciamos integradores “suaves”
    syncInt = 0.0f;
    inte1   = 0.0f;
    inte2   = 0.0f;
    last_out1 = 0;
    last_out2 = 0;
}

void control_coast(void)
{
    // PWM=0 y dirección a 0 en ambos
    TIM3->CCR3 = 0; // M2
    TIM3->CCR4 = 0; // M1
    GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16)); // M1 INA/INB = 0
    GPIOC->BSRR = (1u << (6+16))  | (1u << (7+16));  // M2 INA/INB = 0
}

/* ===== UNA iteración de tu control, sin while(1) ===== */
void control_update(void)
{
    uint32_t now = millis();
    if ((now - last_time) < intervalo) {
        return; // esperamos a que se cumplan 100ms
    }
    last_time = now;

    /* ====== LECTURA ENCODERS ====== */
    // Motor 1 (si “adelante” te da rpm negativa, invierte el signo)
    counter1 = (int16_t)TIM1->CNT;  TIM1->CNT = 0;
    deltaNp1 = counter1;            // ó = -counter1 si lo necesitas

    // Motor 2 (ajusta el signo según tu cableado)
    counter2 = (int16_t)TIM2->CNT;  TIM2->CNT = 0;
    deltaNp2 = -counter2;           // ó =  counter2 si lo necesitas

    /* ====== RPM + FILTRO (con anti-glitch) ====== */
    float rpm1_raw = (deltaNp1 * 600.0f) / PPR;
    float rpm2_raw = (deltaNp2 * 600.0f) / PPR;

    // Ignora caídas bruscas espurias
    if ( (last_rpm1 > MIN_RPM_VALID) && (rpm1_raw < last_rpm1*(1.0f - DROP_FRAC)) )
        rpm1_raw = last_rpm1;
    if ( (last_rpm2 > MIN_RPM_VALID) && (rpm2_raw < last_rpm2*(1.0f - DROP_FRAC)) )
        rpm2_raw = last_rpm2;

    // Filtro exponencial (suaviza ruido y picos)
    rpm_f1 = ALPHA * rpm1_raw + (1.0f - ALPHA) * rpm_f1;
    rpm_f2 = ALPHA * rpm2_raw + (1.0f - ALPHA) * rpm_f2;
    rpm1 = rpm_f1;  rpm2 = rpm_f2;

    last_rpm1 = rpm_f1;
    last_rpm2 = rpm_f2;

    /* ====== CONTROL M1 ====== */
    e1    = rpm_des1 - rpm_f1;
    inte1 += e1 * (intervalo / 1000.0f);
    if (inte1 >  200.0f) inte1 =  200.0f;
    if (inte1 < -200.0f) inte1 = -200.0f;

    u1 = (1.0f / b_est1) * (kp1 * e1 - beta1 * rpm_f1 + kp1 * beta1 * inte1);
    if (u1 >  1023.0f) u1 =  1023.0f;
    if (u1 < -1023.0f) u1 = -1023.0f;

    /* ====== CONTROL M2 ====== */
    e2    = rpm_des2 - rpm_f2;
    inte2 += e2 * (intervalo / 1000.0f);
    if (inte2 >  200.0f) inte2 =  200.0f;
    if (inte2 < -200.0f) inte2 = -200.0f;

    u2 = (1.0f / b_est2) * (kp2 * e2 - beta2 * rpm_f2 + kp2 * beta2 * inte2);
    if (u2 >  1023.0f) u2 =  1023.0f;
    if (u2 < -1023.0f) u2 = -1023.0f;

    /* ====== DUTY base de cada lazo ====== */
    int16_t du1 = (int16_t)((u1 >= 0.0f) ? u1 : -u1);
    int16_t du2 = (int16_t)((u2 >= 0.0f) ? u2 : -u2);
    if (du1 > 1023) du1 = 1023; if (du1 < 0) du1 = 0;
    if (du2 > 1023) du2 = 1023; if (du2 < 0) du2 = 0;

    // Piso mínimo sólo si hay consigna (evita vibración)
    if (rpm_des1 != 0.0f && du1 > 0 && du1 < DUTY_MIN) du1 = DUTY_MIN;
    if (rpm_des2 != 0.0f && du2 > 0 && du2 < DUTY_MIN) du2 = DUTY_MIN;

    /* ====== TRIM de rectitud en DUTY (PI) ====== */
    float e_sync = (rpm_f1 - rpm_f2);      // >0: M1 más rápida
    syncInt += e_sync * syncDt;
    if (syncInt >  syncIlim) syncInt =  syncIlim;
    if (syncInt < -syncIlim) syncInt = -syncIlim;

    float trim = syncKp*e_sync + syncKi*syncInt;   // duty a restar/sumar

    int16_t du1_trim = du1 - (int16_t)trim;        // baja a la rápida
    int16_t du2_trim = du2 + (int16_t)trim;        // sube a la lenta

    // Re-aplica pisos/techos con sentido común
    if (rpm_des1 != 0.0f && du1_trim > 0 && du1_trim < DUTY_MIN) du1_trim = DUTY_MIN;
    if (rpm_des2 != 0.0f && du2_trim > 0 && du2_trim < DUTY_MIN) du2_trim = DUTY_MIN;
    if (du1_trim < 0) du1_trim = 0; if (du1_trim > 1023) du1_trim = 1023;
    if (du2_trim < 0) du2_trim = 0; if (du2_trim > 1023) du2_trim = 1023;

    /* ====== CONTROL Y APLICACIÓN DE PWM (arranque coordinado + normal) ====== */
    int16_t out1 = du1_trim, out2 = du2_trim;  // por defecto, salidas del control “normal”

    // Estado de movimiento
    uint8_t m1_moving = (rpm_f1 > START_THR_RPM || rpm_f1 < -START_THR_RPM);
    uint8_t m2_moving = (rpm_f2 > START_THR_RPM || rpm_f2 < -START_THR_RPM);

    // -------- Gate de arranque coordinado --------
    if (start_gate) {
        if (start_kicks_ctr > 0) {
            // Patada simultánea a ambos los primeros N ciclos
            out1 = START_KICK_PWM;
            out2 = START_KICK_PWM;
            start_kicks_ctr--;
        } else {
            if (m1_moving && m2_moving) {
                // Ambos ya superaron el umbral → liberamos el control normal
                start_gate = 0;
            } else if (!m1_moving && m2_moving) {
                // M1 parado, M2 ya gira: sigue pateando M1; sostén bajo M2
                out1 = START_KICK_PWM;
                out2 = HOLD_PWM;
            } else if (m1_moving && !m2_moving) {
                // M1 ya gira, M2 parado
                out1 = HOLD_PWM;
                out2 = START_KICK_PWM;
            } else {
                // Ambos parados aún → patada a ambos
                out1 = START_KICK_PWM;
                out2 = START_KICK_PWM;
            }
        }

        // Salida por timeout de seguridad (evita quedarse eternamente en gate)
        if (start_timeout > 0) {
            start_timeout--;
            if (start_timeout == 0) start_gate = 0;
        }
    }

    /* ====== Slew-rate para evitar tirones ====== */
    out1 = slew(last_out1, out1);
    out2 = slew(last_out2, out2);
    last_out1 = out1;
    last_out2 = out2;

    /* ====== Dirección y escritura de PWM seguras ====== */
    if (rpm_des1 < 0.0f) setMotor1PWM2(out1);      // atrás
    else if (rpm_des1 > 0.0f) setMotor1PWM1(out1); // adelante
    else setMotor1PWM1(0);

    if (rpm_des2 < 0.0f) setMotor2PWM2(out2);
    else if (rpm_des2 > 0.0f) setMotor2PWM1(out2);
    else setMotor2PWM1(0);
}

/* ==================== IMPLEMENTACIÓN DE TUS DRIVERS ==================== */
/* ============== TIM1 ENCODER (PA8/PA9, AF2) ============== */
void TIM1_Encoder_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA->MODER &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOA->MODER |=  ((2u << (8*2)) | (2u << (9*2)));     // AF
    GPIOA->AFR[1] &= ~((0xFu << 0) | (0xFu << 4));       // p8,p9
    GPIOA->AFR[1] |=  ((2u   << 0) | (2u   << 4));       // AF2

    GPIOA->PUPDR &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOA->PUPDR |=  ((1u << (8*2)) | (1u << (9*2)));    // pull-up

    TIM1->CR1 = 0; TIM1->SMCR = 0; TIM1->CCMR1 = 0; TIM1->CCER = 0;
    TIM1->CCMR1 |= (1u << 0) | (1u << 8);  // CC1S=01, CC2S=01
    TIM1->CCER  &= ~((1u << 1) | (1u << 5));
    TIM1->SMCR  |= 0b011;                  // encoder mode 3
    TIM1->ARR    = 0xFFFF;
    TIM1->CNT    = 0;
    TIM1->CR1   |= 1u;                     // CEN
}

/* ============== TIM2 ENCODER (PA0/PA1, AF2) ============== */
void TIM2_Encoder_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER &= ~((3u << (0*2)) | (3u << (1*2)));
    GPIOA->MODER |=  ((2u << (0*2)) | (2u << (1*2))); // AF
    GPIOA->AFR[0] &= ~((0xFu << (4*0)) | (0xFu << (4*1)));
    GPIOA->AFR[0] |=  ((2u   << (4*0)) | (2u   << (4*1))); // AF2

    GPIOA->PUPDR &= ~((3u << (0*2)) | (3u << (1*2)));
    GPIOA->PUPDR |=  ((1u << (0*2)) | (1u << (1*2))); // pull-up

    TIM2->SMCR  = 0b011;
    TIM2->CCMR1 = (1u << 0) | (1u << 8);
    TIM2->CCER &= ~((1u << 1) | (1u << 5));
    TIM2->ARR   = 0xFFFF;
    TIM2->CNT   = 0;
    TIM2->CR1  |= 1u;
}

/* ====== PWM/DIR ambos motores (TIM3_CH3 y TIM3_CH4) ====== */
void PWM_init_both(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Dirección:
     * M1: PC10/PC11
     * M2: PC6/PC7
     */
    // PC10/PC11
    GPIOC->MODER   &= ~((3u << (10*2)) | (3u << (11*2)));
    GPIOC->MODER   |=  ((1u << (10*2)) | (1u << (11*2)));
    GPIOC->OTYPER  &= ~((1u << 10) | (1u << 11));
    GPIOC->OSPEEDR |=  ((3u << (10*2)) | (3u << (11*2)));
    GPIOC->PUPDR   &= ~((3u << (10*2)) | (3u << (11*2)));

    // PC6/PC7
    GPIOC->MODER   &= ~((3u << (6*2)) | (3u << (7*2)));
    GPIOC->MODER   |=  ((1u << (6*2)) | (1u << (7*2)));
    GPIOC->OTYPER  &= ~((1u << 6) | (1u << 7));
    GPIOC->OSPEEDR |=  ((3u << (6*2)) | (3u << (7*2)));
    GPIOC->PUPDR   &= ~((3u << (6*2)) | (3u << (7*2)));

    // Estado seguro (coast)
    GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16)); // M1
    GPIOC->BSRR = (1u << (6+16))  | (1u << (7+16));  // M2

    /* PWM pins:
     * PC9  -> TIM3_CH4 (M1 PWM)
     * PC8  -> TIM3_CH3 (M2 PWM)
     */
    // PC9
    GPIOC->MODER  &= ~(3u << (9*2));
    GPIOC->MODER  |=  (2u << (9*2));          // AF
    GPIOC->AFR[1] &= ~(0xFu << ((9-8)*4));    // AF0

    // PC8
    GPIOC->MODER  &= ~(3u << (8*2));
    GPIOC->MODER  |=  (2u << (8*2));          // AF
    GPIOC->AFR[1] &= ~(0xFu << ((8-8)*4));    // AF0

    // TIM3 ~1 kHz, 10 bits
    TIM3->PSC  = 7;       // 1 MHz
    TIM3->ARR  = 1023;    // ≈977 Hz
    TIM3->CCR3 = 0;       // M2 duty
    TIM3->CCR4 = 0;       // M1 duty

    // CH3 PWM1 + preload
    TIM3->CCMR2 &= ~(7u << 4);
    TIM3->CCMR2 |=  (6u << 4);
    TIM3->CCMR2 |=  (1u << 3);

    // CH4 PWM1 + preload
    TIM3->CCMR2 &= ~(7u << 12);
    TIM3->CCMR2 |=  (6u << 12);
    TIM3->CCMR2 |=  (1u << 11);

    // Activo-alto y habilitar
    TIM3->CCER &= ~((1u << 9) | (1u << 13));
    TIM3->CCER |=  (1u << 8) | (1u << 12);

    TIM3->CR1 |= (1u << 7);  // ARPE
    TIM3->EGR |= 1u;         // UG
    TIM3->CR1 |= 1u;         // CEN
}

/* ====== Motor 1: PC10/PC11 + PC9 (TIM3 CCR4) ====== */
void setMotor1PWM2(int16_t pwm_value) { // adelante
    if (pwm_value <= 0) {
        TIM3->CCR4 = 0;
        GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << 10);          // INA1=1
    GPIOC->BSRR = (1u << (11+16));     // INB1=0
    TIM3->CCR4 = (uint16_t)pwm_value;
}
void setMotor1PWM1(int16_t pwm_value) { // atrás
    if (pwm_value <= 0) {
        TIM3->CCR4 = 0;
        GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << (10+16));     // INA1=0
    GPIOC->BSRR = (1u << 11);          // INB1=1
    TIM3->CCR4 = (uint16_t)pwm_value;
}

/* ====== Motor 2: PC6/PC7 + PC8 (TIM3 CCR3) ====== */
void setMotor2PWM1(int16_t pwm_value) { // adelante
    if (pwm_value <= 0) {
        TIM3->CCR3 = 0;
        GPIOC->BSRR = (1u << (6+16)) | (1u << (7+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << 6);           // INA2=1
    GPIOC->BSRR = (1u << (7+16));      // INB2=0
    TIM3->CCR3 = (uint16_t)pwm_value;
}
void setMotor2PWM2(int16_t pwm_value) { // atrás
    if (pwm_value <= 0) {
        TIM3->CCR3 = 0;
        GPIOC->BSRR = (1u << (6+16)) | (1u << (7+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << (6+16));      // INA2=0
    GPIOC->BSRR = (1u << 7);           // INB2=1
    TIM3->CCR3 = (uint16_t)pwm_value;
}
void control_reset_all(void)
{
    /* 1) Salidas a estado seguro (coast) */
    control_coast();

    /* 2) Limpiar estado del control */
    rpm1 = rpm2 = 0.0f;
    rpm_f1 = rpm_f2 = 0.0f;
    last_rpm1 = last_rpm2 = 0.0f;

    e1 = e2 = 0.0f;
    inte1 = inte2 = 0.0f;
    u1 = u2 = 0.0f;

    syncInt = 0.0f;
    last_out1 = 0;
    last_out2 = 0;

    /* 3) Reiniciar “arranque coordinado” */
    start_gate      = 1;
    start_kicks_ctr = START_KICKS;
    start_timeout   = START_TIMEOUT;

    /* 4) Zerar contadores de encoders y base de muestreo */
    TIM1->CNT = 0;
    TIM2->CNT = 0;
    last_time = millis();
}
