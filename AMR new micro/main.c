#include <stdint.h>
#include <math.h>
#include "stm32f051x8.h"
#include "Global_Stop_Causes.h"

/* ==================== CONFIG GENERAL ==================== */
#define SYSCLK_HZ 8000000u

/* ==================== PINES DE ENTRADA ==================== */
#define PIN_MODO_PA6     6u     /* PA6: modo manual seguidor (play/pause) */
#define PIN_ROJO_PB1     1u     /* PB1: selección CÍRCULO */
#define PIN_VERDE_PB2    2u     /* PB2: selección INFINITO */
#define PIN_AZUL_PB3     3u     /* PB3: selección RESORTE */

/* ==================== PROTOTIPOS ==================== */
void TIM1_Encoder_Init(void);   // Motor 1: PA8(A), PA9(B)
void TIM2_Encoder_Init(void);   // Motor 2: PA0(A), PA1(B)
void PWM_init_both(void);       // PC8=TIM3_CH3 (M2), PC9=TIM3_CH4 (M1) + DIR pins
void TIM6_Config(void);
void TIM6_IRQHandler(void);

void SysTick_Handler(void);
uint32_t millis(void);
void delay(uint32_t ms);

static float wrapToPi(float a);
void navigation_step(float Ts);
static void path_init(void);

/* Trayectorias */
static void calcularPuntoCircular(float t, float *x, float *y);
static void calcularPuntoInfinito(float t, float *x, float *y);
static void calcularPuntoResorte(float t, float *x, float *y);
static void mover_desde_VW(float vel_lineal, float vel_angular);

/* Motores */
void setMotor1PWM1(int16_t pwm_value);
void setMotor1PWM2(int16_t pwm_value);
void setMotor2PWM1(int16_t pwm_value);
void setMotor2PWM2(int16_t pwm_value);

/* Entradas digitales (modo y colores) */
static void INPUTS_Init(void);
static uint8_t GetModoManual(void);
static void GetColorInputs(volatile uint8_t out_states[3]);

/* Selección de trayectoria y control de estado */
static void UpdateTrajectorySelection(void);
static void ResetTrayectoria(void);
static void StopRobot(void);
static void ResetOdometry(void);


/* ==================== TIMEBASE ==================== */
volatile uint32_t tick_count = 0;
static volatile uint32_t g_ms = 0;

uint8_t colors[3];


uint32_t millis(void){
    return tick_count;
}

void SysTick_Handler(void){
    g_ms++;
    tick_count++;
}

void delay(uint32_t ms){
    uint32_t t = millis();
    while ((millis()-t) < ms) {
        __NOP();
    }
}

/* Tipo de trayectoria */
typedef enum {
    TRJ_NONE = 0,
    TRJ_CIRCLE,
    TRJ_INFINITY,
    TRJ_SPRING
} traj_t;

volatile traj_t current_traj = TRJ_NONE;
float T_max_actual      = 0.0f;
float tolerancia_actual = 0.05f;

/* === Parámetros físicos / control === */
const  uint32_t intervalo = 100; // ms (0.1 s)

const float PPR   = 2000.0f;   // pulsos por revolución encoder 500 x 4
const float alpha = 0.7f;      // filtro exponencial de rpm

// Parametros del robot
const float R_WHEEL = 0.105f;   // radio de rueda [m]
const float L_AXLE  = 0.37f;    // distancia entre ruedas [m]

// Estado del robot
volatile float x_pos = 0.0f;     // [m]
volatile float y_pos = 0.0f;     // [m]
volatile float theta = 0.0f;     // [rad]
volatile float theta_deg = 0.0f; // [°]

volatile float v_x   = 0.0f;     // [m/s]
volatile float v_y   = 0.0f;     // [m/s]
volatile float omega = 0.0f;     // [rad/s]

// RPM deseadas
float rpm_des1 = 0.0f; // Motor 1 (derecha)
float rpm_des2 = 0.0f; // Motor 2 (izquierda)

/* ===== Motor 1 (TIM1 + PC10/PC11/PC9) ===== */
int16_t counter1, deltaNp1;
float   rpm1 = 0.0f, rpm_f1 = 0.0f;
float   e1 = 0.0f, inte1 = 0.0f, u1 = 0.0f;
const float kp1 = 6.0f, ki1 = 3.0f, kd1 = 0.0f;

/* ===== Motor 2 (TIM2 + PC6/PC7/PC8) ===== */
int16_t counter2, deltaNp2;
float   rpm2 = 0.0f, rpm_f2 = 0.0f;
float   e2 = 0.0f, inte2 = 0.0f, u2 = 0.0f;
const float kp2 = 7.0f, ki2 = 3.0f, kd2 = 0.0f;

/* Derivadas filtradas PID */
static float prev_e1 = 0.0f, prev_e2 = 0.0f;
static float derf1 = 0.0f, derf2 = 0.0f;
const  float der_alpha = 0.7f;

/* ======== TRAYECTORIA (PARÁMETROS GLOBALES) ======== */
static float tiempo_trayectoria = 0.0f;
static float x_obj = 0.0f;
static float y_obj = 0.0f;

volatile float dist       = 0.0f;
volatile float theta_goal = 0.0f;

static float x_ref_prev = 0.0f;
static float y_ref_prev = 0.0f;

#define PI             3.14159265359f

#define N_REP_INF    1          // cuántas veces repite la trayectoria completa



static uint8_t trayectoria_terminada = 0;
static uint8_t contador_inf          = 0;

/* ====================== MAIN ====================== */
int main(void) {
    /* INIT HARDWARE ROBOT */
    TIM1_Encoder_Init();
    TIM2_Encoder_Init();
    PWM_init_both();
    INPUTS_Init();
    StopCauses_Init();

    // SysTick 1 ms @ 8 MHz
    SysTick->LOAD = (SYSCLK_HZ/1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    // Inicializar estado de trayectoria (sin trayectoria aún)
    path_init();

    // Timer 6 a 100 ms
    TIM6_Config();

    while (1) {
    }
}

/* ============== TIM6: genera interrupción cada 100 ms ============== */
void TIM6_Config(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // 8 MHz / 8000 = 1 kHz (1 ms)
    TIM6->PSC = 8000u - 1u;
    // 100 cuentas de 1 ms = 100 ms
    TIM6->ARR = 100u - 1u;

    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1  |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_IRQn);
}

void TIM6_IRQHandler(void){
    if (!(TIM6->SR & TIM_SR_UIF)) return;
    TIM6->SR &= ~TIM_SR_UIF;

    float Ts = intervalo / 1000.0f;  // 0.1 s

    /* ====== LECTURA ENCODERS ====== */
    counter1 = (int16_t)TIM1->CNT;
    TIM1->CNT = 0;
    deltaNp1 = counter1;

    counter2 = (int16_t)TIM2->CNT;
    TIM2->CNT = 0;
    deltaNp2 = -counter2;   // invertido mecánico

    /* ====== RPM + FILTRO (EMA) ====== */
    rpm1   = (deltaNp1 * 600.0f) / PPR;
    rpm_f1 = alpha * rpm1 + (1.0f - alpha) * rpm_f1;

    rpm2   = (deltaNp2 * 600.0f) / PPR;
    rpm_f2 = alpha * rpm2 + (1.0f - alpha) * rpm_f2;

    /* ====== VELOCIDADES ====== */
    float Wr = rpm_f1 * 2.0f * PI / 60.0f;
    float Wl = rpm_f2 * 2.0f * PI / 60.0f;

    float v = R_WHEEL * 0.5f * (Wr + Wl);
    omega  = R_WHEEL * (Wr - Wl) / L_AXLE;

    /* ====== ODOMETRÍA ====== */
    theta = wrapToPi(theta + omega * Ts);
    theta_deg = theta * 180.0f / PI;

    v_x = v * cosf(theta);
    v_y = v * sinf(theta);
    x_pos += v_x * Ts;
    y_pos += v_y * Ts;

    /* ====== OBSTÁCULO: PC12 HIGH ====== */
    if (stop_flags.distance1_flag) {
        StopRobot();
        return;
    }

    /* ====== MODO MANUAL (PA6): PAUSA/PLAY ====== */
    if (!GetModoManual()) {
        // Modo OFF -> sólo parar, PERO NO resetear trayectoria
        StopRobot();
        return;
    }

    /* ====== SELECCIÓN DE TRAYECTORIA (PB1/PB2/PB3) ====== */
    UpdateTrajectorySelection();

    // Si no hay trayectoria o ya se terminó, paramos
    if (current_traj == TRJ_NONE || trayectoria_terminada) {
        StopRobot();
        return;
    }

    /* ====== NAVEGACIÓN: actualiza rpm_des1, rpm_des2 ====== */
    navigation_step(Ts);

    /* ====== CONTROL PID M1 (derecha) ====== */
    e1 = rpm_des1 - rpm_f1;

    float raw_der1 = (e1 - prev_e1) / Ts;
    derf1 = der_alpha * derf1 + (1.0f - der_alpha) * raw_der1;
    prev_e1 = e1;

    float u_raw1 = kp1 * e1 + ki1 * inte1 + kd1 * derf1;

    if (!((u_raw1 > 1023.0f && e1 > 0.0f) || (u_raw1 < -1023.0f && e1 < 0.0f))) {
        inte1 += e1 * Ts;
        if (inte1 > 200.0f) inte1 = 200.0f;
        if (inte1 < -200.0f) inte1 = -200.0f;
    }

    u1 = kp1 * e1 + ki1 * inte1 + kd1 * derf1;
    if (u1 > 1023.0f) u1 = 1023.0f;
    if (u1 < -1023.0f) u1 = -1023.0f;

    if (fabsf(rpm_f1) > 50) {
        u1    = 0.0f;
        inte1 = 0.0f;
    }

    int16_t du1 = (int16_t)((u1 >= 0.0f) ? u1 : -u1);
    if (rpm_des1 < 0.0f)      setMotor1PWM1(du1);
    else if (rpm_des1 > 0.0f) setMotor1PWM2(du1);
    else                      setMotor1PWM1(0);

    /* ====== CONTROL PID M2 (izquierda) ====== */
    e2 = rpm_des2 - rpm_f2;

    float raw_der2 = (e2 - prev_e2) / Ts;
    derf2 = der_alpha * derf2 + (1.0f - der_alpha) * raw_der2;
    prev_e2 = e2;

    float u_raw2 = kp2 * e2 + ki2 * inte2 + kd2 * derf2;

    if (!((u_raw2 > 1023.0f && e2 > 0.0f) || (u_raw2 < -1023.0f && e2 < 0.0f))) {
        inte2 += e2 * Ts;
        if (inte2 > 200.0f) inte2 = 200.0f;
        if (inte2 < -200.0f) inte2 = -200.0f;
    }

    u2 = kp2 * e2 + ki2 * inte2 + kd2 * derf2;
    if (u2 > 1023.0f) u2 = 1023.0f;
    if (u2 < -1023.0f) u2 = -1023.0f;

    if (fabsf(rpm_f2) > 50) {
        u2    = 0.0f;
        inte2 = 0.0f;
    }

    int16_t du2 = (int16_t)((u2 >= 0.0f) ? u2 : -u2);
    if (rpm_des2 < 0.0f)      setMotor2PWM2(du2);
    else if (rpm_des2 > 0.0f) setMotor2PWM1(du2);
    else                      setMotor2PWM1(0);
}

/* ============== Navegación estilo código 1 ============== */
void navigation_step(float Ts) {
    (void)Ts;

    // Si no hay trayectoria o ya terminó, no mandamos movimiento
    if (current_traj == TRJ_NONE || trayectoria_terminada) {
        rpm_des1 = 0.0f;
        rpm_des2 = 0.0f;
        return;
    }

    float dx = x_obj - x_pos;
    float dy = y_obj - y_pos;
    float distancia = sqrtf(dx*dx + dy*dy);
    dist = distancia;

    // Cuando llegamos al waypoint actual
    if (distancia <= tolerancia_actual) {

        if (tiempo_trayectoria < T_max_actual) {
            // Seguimos avanzando en esta pasada
            tiempo_trayectoria += 1.0f;

            // Recalcular punto según la trayectoria actual
            switch (current_traj) {
                case TRJ_CIRCLE:
                    calcularPuntoCircular(tiempo_trayectoria, &x_obj, &y_obj);
                    break;
                case TRJ_INFINITY:
                    calcularPuntoInfinito(tiempo_trayectoria, &x_obj, &y_obj);
                    break;
                case TRJ_SPRING:
                    calcularPuntoResorte(tiempo_trayectoria, &x_obj, &y_obj);
                    break;
                default:
                    break;
            }

        } else {
            // Terminamos un recorrido completo (0..T_MAX_INF)
            if (contador_inf < (N_REP_INF - 1)) {
                // Queremos otra vuelta
                contador_inf++;
                tiempo_trayectoria = 0.0f;

                switch (current_traj) {
                    case TRJ_CIRCLE:
                        calcularPuntoCircular(tiempo_trayectoria, &x_obj, &y_obj);
                        break;
                    case TRJ_INFINITY:
                        calcularPuntoInfinito(tiempo_trayectoria, &x_obj, &y_obj);
                        break;
                    case TRJ_SPRING:
                        calcularPuntoResorte(tiempo_trayectoria, &x_obj, &y_obj);
                        break;
                    default:
                        break;
                }
            } else {
                // Ya hicimos N_REP_INF recorridos -> detener
                trayectoria_terminada = 1;
                rpm_des1 = 0.0f;
                rpm_des2 = 0.0f;
                return;
            }
        }

        // Recalcula error con nuevo objetivo
        dx = x_obj - x_pos;
        dy = y_obj - y_pos;
        distancia = sqrtf(dx*dx + dy*dy);
        dist = distancia;
    }

    float angulo_objetivo = atan2f(dy, dx);
    float error_angular   = wrapToPi(angulo_objetivo - theta);
    theta_goal = angulo_objetivo;

    const float velocidad_lineal_max  = 0.7f;
    const float velocidad_angular_max = 0.9f;

    float V_cmd = 0.0f;
    float W_cmd = 0.0f;

    if (fabsf(error_angular) > 0.5f) {
        V_cmd = 0.0f;
        W_cmd = (error_angular > 0.0f) ? velocidad_angular_max : -velocidad_angular_max;
    } else {
        float factor_distancia = (distancia < 0.3f) ?
                                 (distancia / 0.3f) : 1.0f;

        float factor_angular = 1.0f - fabsf(error_angular) / PI;
        if (factor_angular < 0.0f) factor_angular = 0.0f;

        V_cmd = velocidad_lineal_max * factor_distancia * factor_angular;
        W_cmd = 3.0f * error_angular;

        if (W_cmd >  velocidad_angular_max) W_cmd =  velocidad_angular_max;
        if (W_cmd < -velocidad_angular_max) W_cmd = -velocidad_angular_max;
    }

    mover_desde_VW(V_cmd, W_cmd);
}

/* ========= FUNCIONES DE TRAYECTORIA ========= */

// CÍRCULO: radio ≈ 1 m
void calcularPuntoCircular(float t, float *x, float *y) {
    float angulo = 2.0f * PI * t / (T_max_actual);
    *x = sinf(angulo);
    *y = cosf(angulo);
}

// INFINITO (lemniscata simple)
void calcularPuntoInfinito(float t, float *x, float *y) {
    float angulo_x = 2.0f * PI * t / 100.0f;
    float angulo_y = 4.0f * PI * t / 100.0f;
    *x = 1.0f * sinf(angulo_x);
    *y = 0.5f * sinf(angulo_y);
}

// RESORTE (zig-zag o espira mientras avanza en Y)
void calcularPuntoResorte(float t, float *x, float *y) {
    float R = .6f;
    float theta = 1.5f * (2.0f * M_PI * t / 50.0f);
    float forward_factor = 0.2f;

    *x = forward_factor * theta + R * sinf(theta);
    *y = R * cosf(theta) - R;
}

/* ====== De V, W a rpm_des1, rpm_des2 ====== */
static void mover_desde_VW(float vel_lineal, float vel_angular) {
    float Wr_des = (vel_lineal / R_WHEEL +
                   (L_AXLE * vel_angular) / (2.0f * R_WHEEL));
    float Wl_des = (vel_lineal / R_WHEEL -
                   (L_AXLE * vel_angular) / (2.0f * R_WHEEL));

    float rpm_r_des = Wr_des * 60.0f / (2.0f * PI);
    float rpm_l_des = Wl_des * 60.0f / (2.0f * PI);

    const float RPM_MAX = 30.0f;
    if (rpm_r_des >  RPM_MAX) rpm_r_des =  RPM_MAX;
    if (rpm_r_des < -RPM_MAX) rpm_r_des = -RPM_MAX;
    if (rpm_l_des >  RPM_MAX) rpm_l_des =  RPM_MAX;
    if (rpm_l_des < -RPM_MAX) rpm_l_des = -RPM_MAX;

    rpm_des1 = rpm_r_des;
    rpm_des2 = rpm_l_des;
}

/* ================= ENTRADAS (PA6, PB1, PB2, PB3) ================= */
static void INPUTS_Init(void)
{
    /* Habilitar reloj para GPIOA y GPIOB */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* PA6 como entrada con pull-down */
    GPIOA->MODER &= ~(3u << (PIN_MODO_PA6 * 2));      // input
    GPIOA->PUPDR &= ~(3u << (PIN_MODO_PA6 * 2));      // limpiar
    GPIOA->PUPDR |=  (2u << (PIN_MODO_PA6 * 2));      // pull-down (10)

    /* PB1, PB2, PB3 como entradas con pull-down */
    GPIOB->MODER &= ~((3u << (PIN_ROJO_PB1  * 2)) |
                      (3u << (PIN_VERDE_PB2 * 2)) |
                      (3u << (PIN_AZUL_PB3  * 2)));

    GPIOB->PUPDR &= ~((3u << (PIN_ROJO_PB1  * 2)) |
                      (3u << (PIN_VERDE_PB2 * 2)) |
                      (3u << (PIN_AZUL_PB3  * 2)));

    GPIOB->PUPDR |=  ((2u << (PIN_ROJO_PB1  * 2)) |   // pull-down (10)
                      (2u << (PIN_VERDE_PB2 * 2)) |
                      (2u << (PIN_AZUL_PB3  * 2)));
}

/* Devuelve 0 o 1 del pin PA6 (modo manual) */
static uint8_t GetModoManual(void)
{
    return ( (GPIOA->IDR & (1u << PIN_MODO_PA6)) ? 1u : 0u );
}

/* Llena un arreglo con {rojo, verde, azul} */
static void GetColorInputs(volatile uint8_t out_states[3])
{
    out_states[0] = (GPIOB->IDR & (1u << PIN_ROJO_PB1))  ? 1u : 0u;
    out_states[1] = (GPIOB->IDR & (1u << PIN_VERDE_PB2)) ? 1u : 0u;
    out_states[2] = (GPIOB->IDR & (1u << PIN_AZUL_PB3))  ? 1u : 0u;
}

/* Selección de trayectoria según PB1/PB2/PB3 */
static void UpdateTrajectorySelection(void)
{
    GetColorInputs(colors);

    uint8_t num_on = colors[0] + colors[1] + colors[2];
    traj_t new_traj = TRJ_NONE;

    if (num_on == 1) {
        if (colors[0])      new_traj = TRJ_CIRCLE;   // rojo
        else if (colors[1]) new_traj = TRJ_INFINITY; // verde
        else                new_traj = TRJ_SPRING;   // azul
    } else {
        new_traj = TRJ_NONE;   // 0 ó más de 1 → no hacer nada
    }

    if (new_traj != current_traj) {
        current_traj = new_traj;

        if (current_traj != TRJ_NONE) {
            // Cambiamos de figura -> reinicia tiempo y contador de recorridos
            ResetTrayectoria();   // aquí dentro pones tiempo_trayectoria = 0; contador_inf = 0;
            ResetOdometry();      // x_pos = y_pos = theta = 0;

            // Configuramos pasos y tolerancia según la figura elegida
            switch (current_traj) {
            case TRJ_CIRCLE:
            	T_max_actual     = 100.0f;   // menos pasos → círculo más compacto
                tolerancia_actual = 0.05f;  // precisión más fina
                calcularPuntoCircular(tiempo_trayectoria, &x_obj, &y_obj);
                break;

            case TRJ_INFINITY:
            	T_max_actual     = 100.0f;  // más pasos para el 8
                tolerancia_actual = 0.05f;
                calcularPuntoInfinito(tiempo_trayectoria, &x_obj, &y_obj);
                break;

            case TRJ_SPRING:
            	T_max_actual     = 100.0f;  // más largo en Y
                tolerancia_actual = 0.05f;
                calcularPuntoResorte(tiempo_trayectoria, &x_obj, &y_obj);
                break;

            default:
                break;
            }
        } else {
            // Pasamos a "ninguna figura" -> parar, pero NO resetear t
            StopRobot();
        }
    }
}


/* ====================== path_init y utilidades ====================== */
static void path_init(void) {
    tiempo_trayectoria  = 0.0f;
    trayectoria_terminada = 0;
    contador_inf        = 0;
    x_obj = 0.0f;
    y_obj = 0.0f;
    x_ref_prev = x_obj;
    y_ref_prev = y_obj;
    current_traj = TRJ_NONE;    // al inicio no hay trayectoria
}

/* Solo reinicia variables de la trayectoria, NO la odometría */
static void ResetTrayectoria(void)
{
    tiempo_trayectoria   = 0.0f;
    trayectoria_terminada= 0;
    contador_inf         = 0;
}

/* Parar motores y poner RPM deseada en cero */
static void StopRobot(void)
{
    rpm_des1 = 0.0f;
    rpm_des2 = 0.0f;
    inte1 = 0.0f;
    inte2 = 0.0f;

    setMotor1PWM1(0);
    setMotor1PWM2(0);
    setMotor2PWM1(0);
    setMotor2PWM2(0);
}

static void ResetOdometry(void)
{
    x_pos = 0.0f;
    y_pos = 0.0f;
    theta = 0.0f;
    theta_deg = 0.0f;
    v_x = 0.0f;
    v_y = 0.0f;
    omega = 0.0f;
}


/* ============== wrapToPi ============== */
static float wrapToPi(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

/* ============== TIM1 ENCODER (PA8/PA9, AF2) ============== */
void TIM1_Encoder_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA->MODER &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOA->MODER |=  ((2u << (8*2)) | (2u << (9*2)));
    GPIOA->AFR[1] &= ~((0xFu << 0) | (0xFu << 4));
    GPIOA->AFR[1] |=  ((2u   << 0) | (2u   << 4));

    GPIOA->PUPDR &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOA->PUPDR |=  ((1u << (8*2)) | (1u << (9*2)));

    TIM1->CR1 = 0; TIM1->SMCR = 0; TIM1->CCMR1 = 0; TIM1->CCER = 0;
    TIM1->CCMR1 |= (1u << 0) | (1u << 8);
    TIM1->CCER  &= ~((1u << 1) | (1u << 5));
    TIM1->SMCR  |= 0b011;
    TIM1->ARR    = 0xFFFF;
    TIM1->CNT    = 0;
    TIM1->CR1   |= 1u;
}

/* ============== TIM2 ENCODER (PA0/PA1, AF2) ============== */
void TIM2_Encoder_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER &= ~((3u << (0*2)) | (3u << (1*2)));
    GPIOA->MODER |=  ((2u << (0*2)) | (2u << (1*2)));
    GPIOA->AFR[0] &= ~((0xFu << (4*0)) | (0xFu << (4*1)));
    GPIOA->AFR[0] |=  ((2u   << (4*0)) | (2u   << (4*1)));

    GPIOA->PUPDR &= ~((3u << (0*2)) | (3u << (1*2)));
    GPIOA->PUPDR |=  ((1u << (0*2)) | (1u << (1*2)));

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

    // Dirección: M1: PC10/PC11, M2: PC6/PC7
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

    // Estado seguro
    GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16));
    GPIOC->BSRR = (1u << (6+16))  | (1u << (7+16));

    // PWM pins PC9->CH4 (M1), PC8->CH3 (M2)
    GPIOC->MODER  &= ~(3u << (9*2));
    GPIOC->MODER  |=  (2u << (9*2));
    GPIOC->AFR[1] &= ~(0xFu << ((9-8)*4));

    GPIOC->MODER  &= ~(3u << (8*2));
    GPIOC->MODER  |=  (2u << (8*2));
    GPIOC->AFR[1] &= ~(0xFu << ((8-8)*4));

    TIM3->PSC  = 7;
    TIM3->ARR  = 1023;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    TIM3->CCMR2 &= ~(7u << 4);
    TIM3->CCMR2 |=  (6u << 4);
    TIM3->CCMR2 |=  (1u << 3);

    TIM3->CCMR2 &= ~(7u << 12);
    TIM3->CCMR2 |=  (6u << 12);
    TIM3->CCMR2 |=  (1u << 11);

    TIM3->CCER &= ~((1u << 9) | (1u << 13));
    TIM3->CCER |=  (1u << 8) | (1u << 12);

    TIM3->CR1 |= (1u << 7);
    TIM3->EGR |= 1u;
    TIM3->CR1 |= 1u;
}

/* ====== Motor 1: PC10/PC11 + PC9 (TIM3 CCR4) ====== */
void setMotor1PWM1(int16_t pwm_value) { // adelante
    if (pwm_value <= 0) {
        TIM3->CCR4 = 0;
        GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << 10);
    GPIOC->BSRR = (1u << (11+16));
    TIM3->CCR4 = (uint16_t)pwm_value;
}
void setMotor1PWM2(int16_t pwm_value) { // atrás
    if (pwm_value <= 0) {
        TIM3->CCR4 = 0;
        GPIOC->BSRR = (1u << (10+16)) | (1u << (11+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << (10+16));
    GPIOC->BSRR = (1u << 11);
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
    GPIOC->BSRR = (1u << 6);
    GPIOC->BSRR = (1u << (7+16));
    TIM3->CCR3 = (uint16_t)pwm_value;
}
void setMotor2PWM2(int16_t pwm_value) { // atrás
    if (pwm_value <= 0) {
        TIM3->CCR3 = 0;
        GPIOC->BSRR = (1u << (6+16)) | (1u << (7+16));
        return;
    }
    if (pwm_value > (int16_t)TIM3->ARR) pwm_value = (int16_t)TIM3->ARR;
    GPIOC->BSRR = (1u << (6+16));
    GPIOC->BSRR = (1u << 7);
    TIM3->CCR3 = (uint16_t)pwm_value;
}
