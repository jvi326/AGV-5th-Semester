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
#define ACC_LSB_PER_G    16384.0f   // ±2 g
#define GYR_LSB_PER_DPS  131.0f     // ±250 dps
#define G0               9.80665f

/* Ajuste de signos por orientación del módulo (ajusta si tu módulo está montado distinto) */
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

//Para la función de trayectoria
void function(void);   // prototipo

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

/* Helpers MPU */
static inline float wrap_deg_0_360(float a);
static inline float clampf(float x, float lo, float hi);

/* ==================== TIMEBASE ==================== */
volatile uint32_t tick_count = 0;
static volatile uint32_t g_ms = 0;

uint32_t millis(void){
    return tick_count;
}

void SysTick_Handler(void){
    /* Se usan ambos contadores:
       - g_ms para delay_ms (MPU)
       - tick_count para millis() (navegación) */
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

/* Ángulos */
volatile float yaw_deg=0.0f, pitch_deg=0.0f, roll_deg=0.0f;
static volatile float yaw_mod_deg=0.0f, yaw_cum_deg=0.0f;
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

// Posición deseada
float x_ref = 0.0f;   // [m] objetivo en X
float y_ref = 0.0f;   // [m] objetivo en Y

float kv = 0.7f;             // ganancia de velocidad lineal
const float kw = 1.0f;       // ganancia de velocidad angular

// Para que el giro (NAV_ALIGN) sea más suave
const float kw_align        = 0.7f;  // ganancia angular sólo para alinearse
const float omega_align_max = 0.6f;  // [rad/s] límite para el giro suave

const float v_max     = 0.5f;   // [m/s] máx velocidad lineal
const float v_min     = 0.18f;  // [m/s] mín velocidad lineal útil
const float omega_max = 1.5f;   // [rad/s] máx velocidad angular

const float dist_tol  = 0.02f;  // [m] tolerancia de distancia al objetivo
const float ang_tol   = 0.10f;  // [rad] (~3 grados) tolerancia de ángulo

volatile float dist = 0.0f;         // [m]
volatile float theta_goal = 0.0f;   // [rad] ÁNGULO OBJETIVO DEL TRAMO (CONSTANTE EN NAV_GO)

const float S_STEP = 0.05f;   // 5 cm entre puntos (0.05 m)
static float s_traj = 0.0f;   // acumulado de trayectoria

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
    accel_calibrate_zero(300);   // Acel: X~0, Y~0, Z~+1g
    gyro_calibrate(400);         // Gyro: ω~0 en reposo

    /* Reset de ángulos */
    yaw_deg = pitch_deg = roll_deg = 0.0f;
    yaw_mod_deg = yaw_cum_deg = 0.0f;
    angZ_deg = 0.0f;

    // Derivada filtrada (se quedan estáticas en el lazo)
    static float prev_e1 = 0.0f, prev_e2 = 0.0f;
    static float derf1 = 0.0f, derf2 = 0.0f;
    const float der_alpha = 0.7f; // filtro para la derivada

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

            // Motor 2 (izquierda, con signo invertido si la mecánica lo requiere)
            counter2 = (int16_t)TIM2->CNT;
            TIM2->CNT = 0;
            deltaNp2 = -counter2;   // Si te da al revés, quita este menos

            /* ====== RPM + FILTRO (EMA) ====== */
            rpm1   = (deltaNp1 * 600.0f) / PPR;   // 60s/min * 0.1s
            rpm_f1 = alpha * rpm1 + (1.0f - alpha) * rpm_f1;

            rpm2   = (deltaNp2 * 600.0f) / PPR;
            rpm_f2 = alpha * rpm2 + (1.0f - alpha) * rpm_f2;

            /* ====== VELOCIDADES A PARTIR DE ENCODERS ====== */
            float Wr = rpm_f1 * 2.0f * 3.14159265f / 60.0f;  // [rad/s] rueda derecha
            float Wl = rpm_f2 * 2.0f * 3.14159265f / 60.0f;  // [rad/s] rueda izquierda

            float v = R_WHEEL * 0.5f * (Wr + Wl);            // [m/s] velocidad lineal
            omega  = R_WHEEL * (Wr - Wl) / L_AXLE;           // [rad/s] velocidad angular

            /* ====== ACTUALIZAR ÁNGULO THETA CON MPU6050 (EJE Z) ======
             * Se integra el giroscopio en Z dentro de integrate_angles().
             * Aquí tomamos angZ_deg como heading del robot.
             */
            mpu_read_all();           // lee ACC + GYRO
            integrate_angles(Ts);     // integra giroscopio, actualiza angZ_deg

            const float DEG2RAD = 3.14159265f / 180.0f;
            theta = wrapToPi(angZ_deg * DEG2RAD);  // heading en rad usando eje Z del MPU
            theta_deg = theta * 180.0f / 3.14159265f;

            /* ====== ODOMETRÍA (posición x, y) USANDO THETA DEL MPU ====== */
            v_x = v * cosf(theta);
            v_y = v * sinf(theta);

            x_pos += v_x * Ts;
            y_pos += v_y * Ts;
            // theta ya viene del MPU, no se integra aquí con omega

            /* ====== NAVEGACIÓN (actualiza rpm_des1, rpm_des2) ====== */
            navigation_step(Ts);

            /* ====== CONTROL PID M1 (derecha) ====== */
            e1 = rpm_des1 - rpm_f1;

            // Derivada filtrada (sobre el error)
            float raw_der1 = (e1 - prev_e1) / Ts;
            derf1 = der_alpha * derf1 + (1.0f - der_alpha) * raw_der1;
            prev_e1 = e1;

            // Salida "previa" para checar saturación y hacer anti-windup condicional
            float u_raw1 = kp1 * e1 + ki1 * inte1 + kd1 * derf1;

            // Anti-windup: solo integra si no está saturando en la misma dirección
            if (!((u_raw1 > 1023.0f && e1 > 0.0f) || (u_raw1 < -1023.0f && e1 < 0.0f))) {
                inte1 += e1 * Ts;
                if (inte1 > 200.0f) inte1 = 200.0f;
                if (inte1 < -200.0f) inte1 = -200.0f;
            }

            // Salida final + saturación dura
            u1 = kp1 * e1 + ki1 * inte1 + kd1 * derf1;
            if (u1 > 1023.0f) u1 = 1023.0f;
            if (u1 < -1023.0f) u1 = -1023.0f;

            if (fabsf(rpm_f1) > 50) {
                u1    = 0.0f;
                inte1 = 0.0f;     // resetea integrador para que no se enloquezca
            }

            // Actuación por signo de setpoint (M1)
            int16_t du1 = (int16_t)((u1 >= 0.0f) ? u1 : -u1);
            if (rpm_des1 < 0.0f)      setMotor1PWM1(du1);      // atrás
            else if (rpm_des1 > 0.0f) setMotor1PWM2(du1);      // adelante
            else                      setMotor1PWM1(0);        // paro

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

            // Motor 2:
            if (fabsf(rpm_f2) > 50) {
                u2    = 0.0f;
                inte2 = 0.0f;
            }

            int16_t du2 = (int16_t)((u2 >= 0.0f) ? u2 : -u2);
            if (rpm_des2 < 0.0f)      setMotor2PWM2(du2);      // atrás
            else if (rpm_des2 > 0.0f) setMotor2PWM1(du2);      // adelante
            else                      setMotor2PWM1(0);        // paro

            // En Live Expressions puedes ver:
            // x_pos, y_pos, theta_deg, angZ_deg, x_ref, y_ref, rpm_des1, rpm_des2, rpm_f1, rpm_f2
        }
    }
}

/* ============== Función de navegación ============== */
void navigation_step(float Ts) {
    (void)Ts;

    // --- Estado anterior para detectar cambios de modo ---
    static nav_state_t last_state = NAV_ALIGN;

    // Error de posición
    float dx = x_ref - x_pos;
    float dy = y_ref - y_pos;

    dist = sqrtf(dx*dx + dy*dy);

    // Ángulo hacia el punto actual (respecto al mundo)
    float theta_to_target = atan2f(dy, dx);

    // Error angular respecto al robot (usando theta_goal, ver más abajo)
    float err_theta = 0.0f;

    // 1) ¿ya llegué a este punto?
    if (dist < dist_tol) {
        function();          // siguiente punto de la trayectoria
        nav_state = NAV_ALIGN;   // IMPORTANTE: volver a alinearse para el nuevo tramo
        last_state = nav_state;
        return;
    }

    float v_cmd     = 0.0f;
    float omega_cmd = 0.0f;

    // Histeresis angular
    const float ang_tol_align   = 0.25f;  // ~14°

    // --- Si CAMBIÓ el estado desde la última llamada, resetea el control ---
    if (nav_state != last_state) {
        inte1 = 0.0f;
        inte2 = 0.0f;
        rpm_des1 = 0.0f;
        rpm_des2 = 0.0f;
        // Opcional: apagar PWM directamente
        TIM3->CCR3 = 0;
        TIM3->CCR4 = 0;
    }

    // ===== MODO 1: Alinear primero (solo giro, sin avanzar) =====
    if (nav_state == NAV_ALIGN) {
        // AQUÍ sí actualizamos theta_goal hacia el punto ACTUAL
        theta_goal = theta_to_target;

        err_theta = theta_goal - theta;
        err_theta = wrapToPi(err_theta);

        v_cmd     = 0.0f;
        omega_cmd = kw_align * err_theta;   // giro suave

        // límites de giro en alineación
        if (omega_cmd >  omega_align_max) omega_cmd =  omega_align_max;
        if (omega_cmd < -omega_align_max) omega_cmd = -omega_align_max;

        // Si ya quedó bien alineado -> ahora sí avanzar
        if (fabsf(err_theta) < ang_tol_align) {
            nav_state = NAV_GO;
        }
    }

    // ===== MODO 2: Avanzar hacia el punto =====
    if (nav_state == NAV_GO) {
        // EN NAV_GO YA NO CAMBIAMOS theta_goal,
        // se queda fijo con el valor que tenía al salir de NAV_ALIGN.
        err_theta = theta_goal - theta;
        err_theta = wrapToPi(err_theta);

        v_cmd     = kv * dist;
        omega_cmd = kw * err_theta;

        // nunca ir hacia atrás en NAV_GO
        if (v_cmd < 0.0f) v_cmd = 0.0f;

        // limitar velocidad lineal
        if (v_cmd > v_max) v_cmd = v_max;

        // si está lejos y v_cmd muy pequeña, usar mínima
        if (dist > dist_tol && v_cmd < v_min) {
            v_cmd = v_min;
        }

        // límites de omega cuando está avanzando
        if (omega_cmd >  omega_max) omega_cmd =  omega_max;
        if (omega_cmd < -omega_max) omega_cmd = -omega_max;
    }

    // ===== (v_cmd, omega_cmd) → velocidades angulares de ruedas =====
    float Wr_des = (2.0f * v_cmd + omega_cmd * L_AXLE) / (2.0f * R_WHEEL); // derecha
    float Wl_des = (2.0f * v_cmd - omega_cmd * L_AXLE) / (2.0f * R_WHEEL); // izquierda

    // rad/s → RPM
    float rpm_r_des = Wr_des * 60.0f / (2.0f * 3.14159265f);
    float rpm_l_des = Wl_des * 60.0f / (2.0f * 3.14159265f);

    // En NAV_GO no queremos reversa, solo corrección con omega
    if (nav_state == NAV_GO) {
        if (rpm_r_des < 0.0f) rpm_r_des = 0.0f;
        if (rpm_l_des < 0.0f) rpm_l_des = 0.0f;
    }

    // limitar RPM deseadas
    const float RPM_MAX = 30.0f;
    if (rpm_r_des >  RPM_MAX) rpm_r_des =  RPM_MAX;
    if (rpm_r_des < -RPM_MAX) rpm_r_des = -RPM_MAX;
    if (rpm_l_des >  RPM_MAX) rpm_l_des =  RPM_MAX;
    if (rpm_l_des < -RPM_MAX) rpm_l_des = -RPM_MAX;

    rpm_des1 = rpm_r_des;  // Motor 1 (derecha)
    rpm_des2 = rpm_l_des;  // Motor 2 (izquierda)

    // actualizar estado previo
    last_state = nav_state;
}

/* ========== AQUÍ ESTÁ TU FUNCIÓN DE TRAYECTORIA ========== */
/*
 * AQUÍ defines la trayectoria: a partir de s_traj (acumulado en metros)
 * pones x_ref, y_ref del siguiente punto.
 *
 * EJEMPLO actual: línea recta en Y.
 * Si quieres una ELIPSE pequeña, aquí puedes cambiar por:
 *
 *   const float a = 0.25f; // semieje X [m]
 *   const float b = 0.15f; // semieje Y [m]
 *   float t = s_traj / 0.2f;     // factor para que avance despacio
 *   if (t > 2.0f*3.14159265f) {  // reiniciar una vuelta
 *       s_traj = 0.0f;
 *       t = 0.0f;
 *   }
 *   x_ref = a * cosf(t);
 *   y_ref = b * sinf(t);
 */
void function(void) {
    // avanzamos a lo largo de la trayectoria
    s_traj += S_STEP;

    // ====== EJEMPLO: trayectoria en línea recta en Y ======
    x_ref = 0.0f;
    y_ref = s_traj;

    // --- EJEMPLO ELIPSE (DESCOMENTAR SI LA QUIERES USAR) ---
    /*
    const float a = 0.25f; // semieje X [m]
    const float b = 0.15f; // semieje Y [m]
    float t = s_traj / 0.2f;
    if (t > 2.0f*3.14159265f) {
        s_traj = 0.0f;
        t = 0.0f;
    }
    x_ref = a * cosf(t);
    y_ref = b * sinf(t);
    */
}

/* ============== TIM1 ENCODER (PA8/PA9, AF2) ============== */
void TIM1_Encoder_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA->MODER &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOA->MODER |=  ((2u << (8*2)) | (2u << (9*2)));
    // AF
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

    TIM3->PSC  = 7;          // 1 kHz con ARR=1023 a 8 MHz
    TIM3->ARR  = 1023;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    // CH3 PWM mode 1
    TIM3->CCMR2 &= ~(7u << 4);
    TIM3->CCMR2 |=  (6u << 4);
    TIM3->CCMR2 |=  (1u << 3);

    // CH4 PWM mode 1
    TIM3->CCMR2 &= ~(7u << 12);
    TIM3->CCMR2 |=  (6u << 12);
    TIM3->CCMR2 |=  (1u << 11);

    TIM3->CCER &= ~((1u << 9) | (1u << 13));
    TIM3->CCER |=  (1u << 8) | (1u << 12);

    TIM3->CR1 |= (1u << 7);  // ARPE
    TIM3->EGR |= 1u;         // UG
    TIM3->CR1 |= 1u;         // CEN
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
    /* 100 kHz @ 8 MHz */
    I2C1->TIMINGR = 0x00201D2B;
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

    /* Escribir registro */
    I2C1->CR2 = ((uint32_t)dev7<<1) | (1u<<16);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    I2C1->TXDR = reg;

    while(!(I2C1->ISR & I2C_ISR_TC)){;}   // importante TC aquí

    /* Leer len bytes */
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
    i2c1_write_reg(MPU_ADDR, REG_PWR_MGMT_1, 0x01);  // PLL Xgyro, SLEEP=0
    delay_ms(20);
    i2c1_write_reg(MPU_ADDR, REG_CONFIG,       0x03); // DLPF 44 Hz
    i2c1_write_reg(MPU_ADDR, REG_SMPLRT_DIV,   0x07); // 125 Hz
    i2c1_write_reg(MPU_ADDR, REG_ACCEL_CONFIG, 0x00); // ±2 g
    i2c1_write_reg(MPU_ADDR, REG_GYRO_CONFIG,  0x00); // ±250 dps
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
    /* conservamos +1 g en Z */
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

/* ===== Helpers ===== */
static inline float clampf(float x, float lo, float hi){
    return (x<lo)?lo:((x>hi)?hi:x);
}
static inline float wrap_deg_0_360(float a){
    while(a>=360.0f){a-=360.0f;}
    while(a<0.0f){a+=360.0f;}
    return a;
}

/* ==================== Integración de ángulos (usa gyro Z para angZ_deg) ==================== */
static void integrate_angles(float dt){
    /* Integración por rectángulos: θ(k+1) = θ(k) + ω(k)*dt */
    roll_deg  += gx_dps * dt;   // X
    pitch_deg += gy_dps * dt;   // Y
    yaw_deg   += gz_dps * dt;   // Z

    angZ_deg = yaw_deg;         // este es el ángulo acumulado en Z que usamos como heading

    /* Yaw envuelto y acumulado (por si lo quieres en 0..360 o acumulado) */
    static float prev_mod = 0.0f;
    yaw_mod_deg = wrap_deg_0_360(yaw_deg);
    float d = yaw_mod_deg - prev_mod;
    if(d>180.0f){ d-=360.0f; }
    if(d<-180.0f){ d+=360.0f; }
    yaw_cum_deg += d;
    prev_mod = yaw_mod_deg;

    /* Fusión con ACC para roll/pitch (suaviza deriva) */
    float ax=ax_g, ay=ay_g, az=az_g;
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if(n>1e-6f){ ax/=n; ay/=n; az/=n; }
    float roll_acc  = atan2f(ay, az) * 57.2957795f;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;

    float rate_sum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
    float beta_min = 0.02f;
    float beta_max = 0.25f;
    float rate_thr = 5.0f;
    float w_rate   = 1.0f - clampf(rate_sum / rate_thr, 0.0f, 1.0f);

    float g_err  = fabsf(n - 1.0f);
    float g_thr  = 0.08f;
    float w_grav = 1.0f - clampf(g_err / g_thr, 0.0f, 1.0f);  // <<< LÍNEA ARREGLADA

    float beta = beta_min + (beta_max - beta_min) * (w_rate * w_grav);

    roll_deg  = (1.0f-beta)*roll_deg  + beta*roll_acc;
    pitch_deg = (1.0f-beta)*pitch_deg + beta*pitch_acc;
}