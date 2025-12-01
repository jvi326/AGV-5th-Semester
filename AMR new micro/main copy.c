#include <stdint.h>
#include <math.h>
#include "stm32f051x8.h"

/* ==================== CONFIG GENERAL ==================== */
#define SYSCLK_HZ 8000000u

/* I2C1: PB6=SCL, PB7=SDA (AF1) PARA MPU6050 */
#define I2C_SCL_PIN   6u
#define I2C_SDA_PIN   7u

/* MPU6050 regs */
#define MPU_ADDR         0x68u
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B

/* Escalas físicas (±2 g, ±250 dps) */
#define ACC_LSB_PER_G    16384.0f
#define GYR_LSB_PER_DPS  131.0f
#define G0               9.80665f

/* Ajuste de signos por orientación del módulo */
#define GYRO_SIGN_X   (+1.0f)
#define GYRO_SIGN_Y   (+1.0f)
#define GYRO_SIGN_Z   (+1.0f)

/* ==================== PROTOTIPOS EXISTENTES (ROBOT) ==================== */

//Encoders
void TIM1_Encoder_Init(void);   // Motor 1: PA8(A), PA9(B)
void TIM2_Encoder_Init(void);   // Motor 2: PA0(A), PA1(B)

// PWM de ambos motores
void PWM_init_both(void);       // PC8=TIM3_CH3 (M2), PC9=TIM3_CH4 (M1) + DIR pins

//SystickTick para las interrupciones
void SysTick_Handler(void);
uint32_t millis(void);
void delay(uint32_t ms);

static float wrapToPi(float a); // Hacer que en ángulo vaya de [-pi , pi ]
void navigation_step(float Ts);

// Trayectoria por puntos
void function(void);            // avanza al siguiente “punto virtual”
static void path_init(void);    // inicializa la trayectoria

void setMotor1PWM1(int16_t pwm_value); // M1 Adelante
void setMotor1PWM2(int16_t pwm_value); // M1 Atrás
void setMotor2PWM1(int16_t pwm_value); // M2 Adelante
void setMotor2PWM2(int16_t pwm_value); // M2 Atrás

/* ==================== PROTOTIPOS MPU6050 / I2C ==================== */
static void i2c1_gpio_init(void);
static void i2c1_init_100k(void);
static void i2c1_write_reg(uint8_t dev7, uint8_t reg, uint8_t data);
static void i2c1_read_bytes(uint8_t dev7, uint8_t reg, uint8_t *buf, uint8_t len);

static void mpu_init(void);
static void accel_calibrate_zero(uint16_t N);
static void gyro_calibrate(uint16_t N);
static void mpu_read_all(void);
static void integrate_angles(float dt);

/* ==================== TIMEBASE ==================== */
volatile uint32_t tick_count = 0;
static volatile uint32_t g_ms = 0;

uint32_t millis(void){
    return tick_count;
}

void SysTick_Handler(void){
    g_ms++;
    tick_count++;
}

static void delay_ms(uint32_t ms){
    uint32_t t = g_ms;
    while((g_ms - t) < ms){
        __NOP();
    }
}

void delay(uint32_t ms){
    uint32_t t = millis();
    while ((millis()-t) < ms) {
        __NOP();
    }
}

/* ==================== DATOS MPU6050 ==================== */

/* Datos crudos y escalados */
static volatile float ax_g=0, ay_g=0, az_g=0;
static volatile float gx_dps=0, gy_dps=0, gz_dps=0;
static volatile float ax_mps2=0, ay_mps2=0, az_mps2=0;
static float gx_bias=0, gy_bias=0, gz_bias=0;
static int32_t ax_off=0, ay_off=0, az_off=0;

/* Ángulo Z (heading) */
volatile float angZ_deg = 0.0f;   // integral de ωz (heading en grados)

/* ==================== ESTADO DE NAVEGACIÓN (ROBOT) ==================== */
typedef enum {
    NAV_ALIGN = 0,   // primero solo girar al ángulo deseado
    NAV_GO    = 1    // luego avanzar hacia el punto
} nav_state_t;

volatile nav_state_t nav_state = NAV_ALIGN;

// Control PI
uint32_t last_time = 0;
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

// RPM deseadas para llegar al objetivo
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

// Posición deseada (waypoint actual)
float x_ref = 0.0f;   // [m] objetivo en X
float y_ref = 0.0f;   // [m] objetivo en Y

/* Ganancia base (recta) */
float kv = 0.7f;             // ganancia de velocidad lineal
const float kw = 1.0f;       // ganancia de velocidad angular

// Para que el giro (NAV_ALIGN) sea más suave
const float kw_align        = 0.7f;  // ganancia angular sólo para alinearse
const float omega_align_max = 0.6f;  // [rad/s] límite para el giro suave

const float v_max     = 0.5f;   // [m/s] máx velocidad lineal
const float v_min     = 0.20f;  // [m/s] mín velocidad lineal útil
const float omega_max = 1.5f;   // [rad/s] máx velocidad angular

const float dist_tol  = 0.01f;  // [m] tolerancia de distancia al objetivo

/* ================= WAYPOINTS: SIN ARREGLO (function() matemática) ================= */

#define N_WP_LINE   21
#define N_WP_CIRC   120
#define N_WP        (N_WP_LINE + N_WP_CIRC)

static uint16_t idx_wp = 0;       // índice de waypoint actual

volatile float dist       = 0.0f;       // [m]
volatile float theta_goal = 0.0f;       // [rad]

static float x_ref_prev = 0.0f;
static float y_ref_prev = 0.0f;

/* ===========================================================
 * Inicializa la trayectoria:
 *  - idx_wp = 0
 *  - function() generará los puntos siguientes (recta + círculo)
 * =========================================================== */
static void path_init(void) {
    idx_wp = 0;

    // Primer punto (misma posición inicial, luego function() saltará al siguiente)
    float t = 0.0f;
    x_ref = 1.0f * t;
    y_ref = 0.0f;

    x_ref_prev = x_ref;
    y_ref_prev = y_ref;

    nav_state = NAV_ALIGN;
}

/* ====================== MAIN ====================== */
int main(void) {
    /* ======= INIT HARDWARE ROBOT ======= */
    TIM1_Encoder_Init();    // M1 encoder (derecha)
    TIM2_Encoder_Init();    // M2 encoder (izquierda)
    PWM_init_both();        // TIM3 CH3/CH4 + pines dirección

    // SysTick 1 ms @ 8 MHz
    SysTick->LOAD = (SYSCLK_HZ/1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    nav_state = NAV_ALIGN;   // al iniciar: primero alinear ángulo

    /* ======= INIT MPU6050 (I2C) ======= */
    i2c1_gpio_init();
    i2c1_init_100k();
    mpu_init();

    /* Calibraciones (sensor quieto y horizontal) */
    // accel_calibrate_zero(300);
    gyro_calibrate(400);

    /* Reset de ángulo Z */
    angZ_deg = 0.0f;

    // Derivada filtrada
    static float prev_e1 = 0.0f, prev_e2 = 0.0f;
    static float derf1 = 0.0f, derf2 = 0.0f;
    const float der_alpha = 0.7f;

    // ====== INICIALIZAR LA TRAYECTORIA (RECTA + CÍRCULO R=1.0) ======
    path_init();

    while (1) {
        uint32_t now = millis();
        if ((now - last_time) >= intervalo) {
            last_time = now;

            float Ts = (intervalo / 1000.0f);  // 0.1 s

            /* ====== LECTURA ENCODERS ====== */
            // Motor 1 (derecha)
            counter1 = (int16_t)TIM1->CNT;
            TIM1->CNT = 0;
            deltaNp1 = counter1;

            // Motor 2 (izquierda)
            counter2 = (int16_t)TIM2->CNT;
            TIM2->CNT = 0;
            deltaNp2 = -counter2;   // ojo: invertido mecánico

            /* ====== RPM + FILTRO (EMA) ====== */
            rpm1   = (deltaNp1 * 600.0f) / PPR;
            rpm_f1 = alpha * rpm1 + (1.0f - alpha) * rpm_f1;

            rpm2   = (deltaNp2 * 600.0f) / PPR;
            rpm_f2 = alpha * rpm2 + (1.0f - alpha) * rpm_f2;

            /* ====== VELOCIDADES A PARTIR DE ENCODERS ====== */
            float Wr = rpm_f1 * 2.0f * 3.14159265f / 60.0f;  // [rad/s]
            float Wl = rpm_f2 * 2.0f * 3.14159265f / 60.0f;  // [rad/s]

            float v = R_WHEEL * 0.5f * (Wr + Wl);            // [m/s]
            omega  = R_WHEEL * (Wr - Wl) / L_AXLE;           // [rad/s]

            /* ====== ACTUALIZAR ÁNGULO THETA CON MPU6050 (EJE Z) ====== */
            mpu_read_all();
            integrate_angles(Ts);

            const float DEG2RAD = 3.14159265f / 180.0f;
            theta = wrapToPi(angZ_deg * DEG2RAD);
            theta_deg = theta * 180.0f / 3.14159265f;

            /* ====== ODOMETRÍA (posición x, y) USANDO THETA DEL MPU ====== */
            v_x = v * cosf(theta);
            v_y = v * sinf(theta);

            x_pos += v_x * Ts;
            y_pos += v_y * Ts;

            /* ====== NAVEGACIÓN (actualiza rpm_des1, rpm_des2) ====== */
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
    }
}

/* ============== Función de navegación ============== */
void navigation_step(float Ts) {
    (void)Ts;

    // Si ya terminamos todos los waypoints -> parar
    if (idx_wp >= N_WP) {
        rpm_des1 = 0.0f;
        rpm_des2 = 0.0f;
        return;
    }

    // --- Error de posición desde robot al waypoint actual ---
    float ex = x_ref - x_pos;
    float ey = y_ref - y_pos;
    dist = sqrtf(ex*ex + ey*ey);

    // --- Vector tangente al camino (de punto anterior al actual) ---
    float tx = x_ref - x_ref_prev;
    float ty = y_ref - y_ref_prev;
    float t2 = tx*tx + ty*ty;

    if (t2 < 1e-6f) {
        tx = cosf(theta);
        ty = sinf(theta);
        t2 = tx*tx + ty*ty;
    }

    theta_goal = atan2f(ty, tx);
    theta_goal = wrapToPi(theta_goal);

    // Proyección del robot sobre el segmento [prev -> actual]
    float vx = x_pos - x_ref_prev;
    float vy = y_pos - y_ref_prev;
    float lambda = (vx*tx + vy*ty) / t2;

    // Condición para pasar al siguiente punto
    if ((dist < dist_tol) || (lambda > 1.0f)) {
        function();  // avanza al siguiente punto (math)
        return;
    }

    float v_cmd     = 0.0f;
    float omega_cmd = 0.0f;
    float err_theta = 0.0f;

    const float ang_tol_align = 0.15f;  // ~8.5°

    /* Ganancias distintas para recta vs círculo */
    float kv_eff    = kv;
    float kw_eff    = kw;
    float v_max_eff = v_max;

    if (idx_wp >= N_WP_LINE) {      // estamos en la parte circular
        kv_eff    = 0.35f;          // menos agresivo en velocidad lineal
        kw_eff    = 1.5f;           // más fuerte en giro
        v_max_eff = 0.25f;          // más despacio en el círculo
    }

    // ====== NAV_ALIGN: solo giro hasta alinear con la tangente ======
    if (nav_state == NAV_ALIGN) {
        err_theta = wrapToPi(theta_goal - theta);

        v_cmd     = 0.0f;
        omega_cmd = kw_align * err_theta;

        if (omega_cmd >  omega_align_max) omega_cmd =  omega_align_max;
        if (omega_cmd < -omega_align_max) omega_cmd = -omega_align_max;

        if (fabsf(err_theta) < ang_tol_align) {
            nav_state = NAV_GO;
        }
    }

    // ====== NAV_GO: avanzar siguiendo la trayectoria ======
    if (nav_state == NAV_GO) {
        err_theta = wrapToPi(theta_goal - theta);

        v_cmd     = kv_eff * dist;
        omega_cmd = kw_eff * err_theta;

        if (v_cmd < 0.0f) v_cmd = 0.0f;
        if (v_cmd > v_max_eff) v_cmd = v_max_eff;
        if (dist > dist_tol && v_cmd < v_min) {
            v_cmd = v_min;
        }

        if (omega_cmd >  omega_max) omega_cmd =  omega_max;
        if (omega_cmd < -omega_max) omega_cmd = -omega_max;
    }

    // ===== (v_cmd, omega_cmd) -> velocidades de ruedas =====
    float Wr_des = (2.0f * v_cmd + omega_cmd * L_AXLE) / (2.0f * R_WHEEL);
    float Wl_des = (2.0f * v_cmd - omega_cmd * L_AXLE) / (2.0f * R_WHEEL);

    float rpm_r_des = Wr_des * 60.0f / (2.0f * 3.14159265f);
    float rpm_l_des = Wl_des * 60.0f / (2.0f * 3.14159265f);

    // En NAV_GO no dejamos ir hacia atrás
    if (nav_state == NAV_GO) {
        if (rpm_r_des < 0.0f) rpm_r_des = 0.0f;
        if (rpm_l_des < 0.0f) rpm_l_des = 0.0f;
    }

    const float RPM_MAX = 30.0f;
    if (rpm_r_des >  RPM_MAX) rpm_r_des =  RPM_MAX;
    if (rpm_r_des < -RPM_MAX) rpm_r_des = -RPM_MAX;
    if (rpm_l_des >  RPM_MAX) rpm_l_des =  RPM_MAX;
    if (rpm_l_des < -RPM_MAX) rpm_l_des = -RPM_MAX;

    rpm_des1 = rpm_r_des;   // derecha
    rpm_des2 = rpm_l_des;   // izquierda
}

/* ===== FUNCIÓN: genera el siguiente waypoint matemático ===== */
void function(void) {
    // Si ya estamos en el último waypoint, detenerse
    if (idx_wp >= (N_WP - 1)) {
        rpm_des1 = 0.0f;
        rpm_des2 = 0.0f;
        idx_wp   = N_WP;
        return;
    }

    idx_wp++;   // pasar al siguiente índice

    x_ref_prev = x_ref;
    y_ref_prev = y_ref;

    // ----- Tramo 1: recta de (0,0) a (1,0) -----
    if (idx_wp < N_WP_LINE) {
        float t = (float)idx_wp / (float)(N_WP_LINE - 1);  // 0..1
        x_ref = 1.0f * t;
        y_ref = 0.0f;
    }
    // ----- Tramo 2: círculo de radio 1 centrado en (0,0) -----
    else {
        uint16_t k_circ = idx_wp - N_WP_LINE;   // 0..N_WP_CIRC-1
        float    dtheta = 2.0f * 3.14159265f / (float)N_WP_CIRC;
        float    ang    = dtheta * (float)(k_circ + 1);    // de dθ a 2π

        float R = 1.0f;   // radio del círculo

        x_ref = R * cosf(ang);
        y_ref = R * sinf(ang);
    }

    // Cada vez que cambiamos de punto, volvemos a alinear primero
    nav_state = NAV_ALIGN;
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

/* ============== wrapToPi ============== */
static float wrapToPi(float a) {
    while (a >  3.14159265f) a -= 2.0f * 3.14159265f;
    while (a < -3.14159265f) a += 2.0f * 3.14159265f;
    return a;
}

/* ==================== I2C1 (MPU6050) ==================== */
static void i2c1_gpio_init(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER   &= ~((3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2)));
    GPIOB->MODER   |=  ((2u<<(I2C_SCL_PIN*2)) | (2u<<(I2C_SDA_PIN*2)));
    GPIOB->OTYPER  |=  (1u<<I2C_SCL_PIN) | (1u<<I2C_SDA_PIN);
    GPIOB->PUPDR   &= ~((3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2)));
    GPIOB->PUPDR   |=  ((1u<<(I2C_SCL_PIN*2)) | (1u<<(I2C_SDA_PIN*2)));
    GPIOB->AFR[0]  &= ~((0xFu<<(I2C_SCL_PIN*4)) | (0xFu<<(I2C_SDA_PIN*4)));
    GPIOB->AFR[0]  |=  ((0x1u<<(I2C_SCL_PIN*4)) | (0x1u<<(I2C_SDA_PIN*4)));
    GPIOB->OSPEEDR |=  ((3u<<(I2C_SCL_PIN*2)) | (3u<<(I2C_SDA_PIN*2)));
}
static void i2c1_init_100k(void){
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x00201D2B; // 100 kHz @ 8 MHz
    I2C1->CR1 |= I2C_CR1_PE;
}
static void i2c1_write_reg(uint8_t dev7, uint8_t reg, uint8_t data){
    I2C1->CR2 = ((uint32_t)dev7<<1) | (2u<<16) | I2C_CR2_AUTOEND;
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    I2C1->TXDR = reg;

    while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    I2C1->TXDR = data;

    while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
    I2C1->ICR = I2C_ICR_STOPCF;
}
static void i2c1_read_bytes(uint8_t dev7, uint8_t reg, uint8_t *buf, uint8_t len){
    if(!len){ return; }

    I2C1->CR2 = ((uint32_t)dev7<<1) | (1u<<16);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    I2C1->TXDR = reg;

    while(!(I2C1->ISR & I2C_ISR_TC)){;}

    I2C1->CR2 = ((uint32_t)dev7<<1) | (len<<16) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND;
    I2C1->CR2 |= I2C_CR2_START;

    for(uint8_t i=0;i<len;i++){
        while(!(I2C1->ISR & I2C_ISR_RXNE)){;}
        buf[i]= (uint8_t)I2C1->RXDR;
    }

    while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
    I2C1->ICR = I2C_ICR_STOPCF;
}

/* ==================== Init & Calibraciones MPU ==================== */
static void mpu_init(void){
    i2c1_write_reg(MPU_ADDR, REG_PWR_MGMT_1, 0x01);
    delay_ms(20);
    i2c1_write_reg(MPU_ADDR, REG_CONFIG,       0x03);
    i2c1_write_reg(MPU_ADDR, REG_SMPLRT_DIV,   0x07);
    i2c1_write_reg(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);
    i2c1_write_reg(MPU_ADDR, REG_GYRO_CONFIG,  0x00);
    i2c1_write_reg(MPU_ADDR, REG_INT_ENABLE,   0x00);
}

/* Acel en reposo horizontal: X≈0, Y≈0, Z≈+1 g */
static void accel_calibrate_zero(uint16_t N){
    int64_t sx=0, sy=0, sz=0;
    for(uint16_t i=0;i<N;i++){
        uint8_t r[14]; i2c1_read_bytes(MPU_ADDR, REG_ACCEL_XOUT_H, r, 14);
        int16_t ax=(int16_t)((r[0]<<8)|r[1]);
        int16_t ay=(int16_t)((r[2]<<8)|r[3]);
        int16_t az=(int16_t)((r[4]<<8)|r[5]);
        sx+=ax; sy+=ay; sz+=az;
        delay_ms(5);
    }
    ax_off = (int32_t)(sx/N);
    ay_off = (int32_t)(sy/N);
    az_off = (int32_t)(sz/N) - (int32_t)ACC_LSB_PER_G;
}

/* Giroscopio en reposo */
static void gyro_calibrate(uint16_t N){
    float sx=0, sy=0, sz=0;
    for(uint16_t i=0;i<N;i++){
        uint8_t r[14]; i2c1_read_bytes(MPU_ADDR, REG_ACCEL_XOUT_H, r, 14);
        int16_t gx=(int16_t)((r[8]<<8)|r[9]);
        int16_t gy=(int16_t)((r[10]<<8)|r[11]);
        int16_t gz=(int16_t)((r[12]<<8)|r[13]);
        sx += (float)gx / GYR_LSB_PER_DPS;
        sy += (float)gy / GYR_LSB_PER_DPS;
        sz += (float)gz / GYR_LSB_PER_DPS;
        delay_ms(5);
    }
    gx_bias=sx/N; gy_bias=sy/N; gz_bias=sz/N;
}

/* ==================== Lectura + conversión ==================== */
static void mpu_read_all(void){
    uint8_t r[14]; i2c1_read_bytes(MPU_ADDR, REG_ACCEL_XOUT_H, r, 14);

    int16_t axr=(int16_t)((r[0]<<8)|r[1]);
    int16_t ayr=(int16_t)((r[2]<<8)|r[3]);
    int16_t azr=(int16_t)((r[4]<<8)|r[5]);

    int16_t gxr=(int16_t)((r[8]<<8)|r[9]);
    int16_t gyr=(int16_t)((r[10]<<8)|r[11]);
    int16_t gzr=(int16_t)((r[12]<<8)|r[13]);

    int32_t axc=(int32_t)axr-ax_off;
    int32_t ayc=(int32_t)ayr-ay_off;
    int32_t azc=(int32_t)azr-az_off;

    ax_g = ((float)axc)/ACC_LSB_PER_G;
    ay_g = ((float)ayc)/ACC_LSB_PER_G;
    az_g = ((float)azc)/ACC_LSB_PER_G;

    ax_mps2 = ax_g*G0;
    ay_mps2 = ay_g*G0;
    az_mps2 = az_g*G0;

    gx_dps = ((float)gxr / GYR_LSB_PER_DPS - gx_bias)*GYRO_SIGN_X;
    gy_dps = ((float)gyr / GYR_LSB_PER_DPS - gy_bias)*GYRO_SIGN_Y;
    gz_dps = ((float)gzr / GYR_LSB_PER_DPS - gz_bias)*GYRO_SIGN_Z;
}

/* ==================== Integración de ángulo Z ==================== */
static void integrate_angles(float dt){
    // Integrar solo la velocidad angular Z para obtener heading en grados
    angZ_deg += gz_dps * dt;

    // Evitar que el valor crezca demasiado
    if (angZ_deg >  720.0f || angZ_deg < -720.0f) {
        angZ_deg = fmodf(angZ_deg, 360.0f);
    }
}
