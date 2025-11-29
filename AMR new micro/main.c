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

// Posición deseada actual
float x_ref = 0.0f;   // [m] objetivo en X
float y_ref = 0.0f;   // [m] objetivo en Y

// Punto anterior de la trayectoria (para vector tangente)
static float x_ref_prev = 0.0f;
static float y_ref_prev = 0.0f;

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
volatile float theta_goal = 0.0f;   // [rad] ÁNGULO OBJETIVO DEL TRAMO (tangente)

/* ==================== TRAYECTORIA: 20 PUNTOS LÍNEA + 100 PUNTOS CÍRCULO ==================== */
/*  - Puntos 0..19: línea recta (0,0) -> (1,0)
 *  - Puntos 20..119: círculo r=1 centrado en (0,0), con pequeño offset en ángulo
 */
#define NUM_WP   120
#define N_LINE   20
#define N_CIRCLE 100

static const float x_wp[NUM_WP] = {
    /* ===== 20 pts línea (0,0) -> (1,0) ===== */
     0.000000f,  0.052632f,  0.105263f,  0.157895f,  0.210526f,
     0.263158f,  0.315789f,  0.368421f,  0.421053f,  0.473684f,
     0.526316f,  0.578947f,  0.631579f,  0.684211f,  0.736842f,
     0.789474f,  0.842105f,  0.894737f,  0.947368f,  1.000000f,

    /* ===== 100 pts círculo r=1, centrado en (0,0) (offset de 0.5 step) ===== */
     0.999507f,  0.995562f,  0.987688f,  0.975917f,  0.960294f,
     0.940881f,  0.917755f,  0.891007f,  0.860742f,  0.827081f,
     0.790155f,  0.750111f,  0.707107f,  0.661312f,  0.612907f,
     0.562083f,  0.509041f,  0.453990f,  0.397148f,  0.338738f,
     0.278991f,  0.218143f,  0.156434f,  0.094108f,  0.031411f,
    -0.031411f, -0.094108f, -0.156434f, -0.218143f, -0.278991f,
    -0.338738f, -0.397148f, -0.453990f, -0.509041f, -0.562083f,
    -0.612907f, -0.661312f, -0.707107f, -0.750111f, -0.790155f,
    -0.827081f, -0.860742f, -0.891007f, -0.917755f, -0.940881f,
    -0.960294f, -0.975917f, -0.987688f, -0.995562f, -0.999507f,
    -0.999507f, -0.995562f, -0.987688f, -0.975917f, -0.960294f,
    -0.940881f, -0.917755f, -0.891007f, -0.860742f, -0.827081f,
    -0.790155f, -0.750111f, -0.707107f, -0.661312f, -0.612907f,
    -0.562083f, -0.509041f, -0.453990f, -0.397148f, -0.338738f,
    -0.278991f, -0.218143f, -0.156434f, -0.094108f, -0.031411f,
     0.031411f,  0.094108f,  0.156434f,  0.218143f,  0.278991f,
     0.338738f,  0.397148f,  0.453990f,  0.509041f,  0.562083f,
     0.612907f,  0.661312f,  0.707107f,  0.750111f,  0.790155f,
     0.827081f,  0.860742f,  0.891007f,  0.917755f,  0.940881f,
     0.960294f,  0.975917f,  0.987688f,  0.995562f,  0.999507f
};

static const float y_wp[NUM_WP] = {
    /* ===== 20 pts línea (0,0) -> (1,0) ===== */
     0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
     0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
     0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
     0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,

    /* ===== 100 pts círculo r=1 ===== */
     0.031411f,  0.094108f,  0.156434f,  0.218143f,  0.278991f,
     0.338738f,  0.397148f,  0.453990f,  0.509041f,  0.562083f,
     0.612907f,  0.661312f,  0.707107f,  0.750111f,  0.790155f,
     0.827081f,  0.860742f,  0.891007f,  0.917755f,  0.940881f,
     0.960294f,  0.975917f,  0.987688f,  0.995562f,  0.999507f,
     0.999507f,  0.995562f,  0.987688f,  0.975917f,  0.960294f,
     0.940881f,  0.917755f,  0.891007f,  0.860742f,  0.827081f,
     0.790155f,  0.750111f,  0.707107f,  0.661312f,  0.612907f,
     0.562083f,  0.509041f,  0.453990f,  0.397148f,  0.338738f,
     0.278991f,  0.218143f,  0.156434f,  0.094108f,  0.031411f,
    -0.031411f, -0.094108f, -0.156434f, -0.218143f, -0.278991f,
    -0.338738f, -0.397148f, -0.453990f, -0.509041f, -0.562083f,
    -0.612907f, -0.661312f, -0.707107f, -0.750111f, -0.790155f,
    -0.827081f, -0.860742f, -0.891007f, -0.917755f, -0.940881f,
    -0.960294f, -0.975917f, -0.987688f, -0.995562f, -0.999507f,
    -0.999507f, -0.995562f, -0.987688f, -0.975917f, -0.960294f,
    -0.940881f, -0.917755f, -0.891007f, -0.860742f, -0.827081f,
    -0.790155f, -0.750111f, -0.707107f, -0.661312f, -0.612907f,
    -0.562083f, -0.509041f, -0.453990f, -0.397148f, -0.338738f,
    -0.278991f, -0.218143f, -0.156434f, -0.094108f, -0.031411f
};

static uint16_t wp_idx = 0;   // índice del waypoint actual (0..NUM_WP-1)

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

    /* ======= INICIALIZAR TRAYECTORIA ======= */
    wp_idx      = 0;
    x_ref       = x_wp[0];
    y_ref       = y_wp[0];
    x_ref_prev  = x_ref;
    y_ref_prev  = y_ref;

    x_pos       = 0.0f;
    y_pos       = 0.0f;
    theta       = 0.0f;   // asumimos robot apuntando al +X
    theta_deg   = 0.0f;

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

            /* ====== ACTUALIZAR ÁNGULO THETA CON MPU6050 (EJE Z) ====== */
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
                inte1 = 0.0f;
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

            if (fabsf(rpm_f2) > 50) {
                u2    = 0.0f;
                inte2 = 0.0f;
            }

            int16_t du2 = (int16_t)((u2 >= 0.0f) ? u2 : -u2);
            if (rpm_des2 < 0.0f)      setMotor2PWM2(du2);      // atrás
            else if (rpm_des2 > 0.0f) setMotor2PWM1(du2);      // adelante
            else                      setMotor2PWM1(0);        // paro
        }
    }
}

/* ============== Función de navegación (usa el ARRAY de puntos) ============== */
/* ============== Función de navegación (usa el ARRAY de puntos) ============== */
void navigation_step(float Ts) {
    (void)Ts;

    // Si ya terminamos todos los waypoints -> parar motores
    if (wp_idx >= NUM_WP) {
        rpm_des1 = 0.0f;
        rpm_des2 = 0.0f;
        return;
    }

    /* -------- Distancia al waypoint actual -------- */
    float ex = x_ref - x_pos;
    float ey = y_ref - y_pos;
    dist = sqrtf(ex*ex + ey*ey);   // distancia al waypoint actual

    /* -------- Vector tangente (de waypoint actual al siguiente) -------- */
    float tx, ty;
    if (wp_idx < (NUM_WP - 1)) {
        // usar segmento [waypoint actual -> siguiente] como tangente
        float x_next = x_wp[wp_idx + 1];
        float y_next = y_wp[wp_idx + 1];
        tx = x_next - x_ref;
        ty = y_next - y_ref;
    } else {
        // último punto: usar [prev -> actual]
        tx = x_ref - x_ref_prev;
        ty = y_ref - y_ref_prev;
    }

    float t2 = tx*tx + ty*ty;
    if (t2 < 1e-6f) {
        // si el vector es casi cero, usar orientación actual del robot
        tx = cosf(theta);
        ty = sinf(theta);
        t2 = tx*tx + ty*ty;
    }

    // Ángulo deseado de la trayectoria = ángulo de la tangente
    theta_goal = atan2f(ty, tx);
    theta_goal = wrapToPi(theta_goal);

    /* -------- LÓGICA ROBUSTA PARA CAMBIAR DE WAYPOINT -------- */
    if (wp_idx < (NUM_WP - 1)) {
        float x_next = x_wp[wp_idx + 1];
        float y_next = y_wp[wp_idx + 1];

        float ex_next = x_next - x_pos;
        float ey_next = y_next - y_pos;
        float dist_next = sqrtf(ex_next*ex_next + ey_next*ey_next);

        // margen pequeño para evitar oscilación: 1 cm
        const float ADV_MARGIN = 0.01f;

        // SI:
        //  - ya estoy dentro de la tolerancia del waypoint actual, O
        //  - estoy claramente más cerca del siguiente que del actual,
        // ENTONCES avanzo al siguiente waypoint
        if ((dist < dist_tol) || (dist_next + ADV_MARGIN < dist)) {
            wp_idx++;

            x_ref_prev = x_ref;
            y_ref_prev = y_ref;

            x_ref = x_wp[wp_idx];
            y_ref = y_wp[wp_idx];

            return;  // este ciclo solo actualiza el índice; el control actúa en el siguiente tick
        }
    } else {
        // Último waypoint: si ya estoy cerca, paro
        if (dist < dist_tol) {
            rpm_des1 = 0.0f;
            rpm_des2 = 0.0f;
            return;
        }
    }

    /* -------- CONTROL DE VELOCIDADES v_cmd y omega_cmd -------- */
    float v_cmd     = 0.0f;
    float omega_cmd = 0.0f;
    float err_theta = 0.0f;

    const float ang_tol_align = 0.25f;  // ~14°

    // ====== NAV_ALIGN: solo girar hasta alinearse con la tangente ======
    if (nav_state == NAV_ALIGN) {
        err_theta = wrapToPi(theta_goal - theta);

        v_cmd     = 0.0f;
        omega_cmd = kw_align * err_theta;

        if (omega_cmd >  omega_align_max) omega_cmd =  omega_align_max;
        if (omega_cmd < -omega_align_max) omega_cmd = -omega_align_max;

        // Cuando ya estoy alineada, empiezo a avanzar
        if (fabsf(err_theta) < ang_tol_align) {
            nav_state = NAV_GO;
        }
    }

    // ====== NAV_GO: avanzar siguiendo la curva ======
    if (nav_state == NAV_GO) {
        err_theta = wrapToPi(theta_goal - theta);

        v_cmd     = kv * dist;
        omega_cmd = kw * err_theta;

        if (v_cmd < 0.0f) v_cmd = 0.0f;
        if (v_cmd > v_max) v_cmd = v_max;
        if (dist > dist_tol && v_cmd < v_min) {
            v_cmd = v_min;
        }

        if (omega_cmd >  omega_max) omega_cmd =  omega_max;
        if (omega_cmd < -omega_max) omega_cmd = -omega_max;
    }

    /* -------- (v_cmd, omega_cmd) -> rpm de ruedas -------- */
    float Wr_des = (2.0f * v_cmd + omega_cmd * L_AXLE) / (2.0f * R_WHEEL);
    float Wl_des = (2.0f * v_cmd - omega_cmd * L_AXLE) / (2.0f * R_WHEEL);

    float rpm_r_des = Wr_des * 60.0f / (2.0f * 3.14159265f);
    float rpm_l_des = Wl_des * 60.0f / (2.0f * 3.14159265f);

    // En NAV_GO no dejamos retroceder (solo hacia adelante sobre el camino)
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
    float w_grav = 1.0f - clampf(g_err / g_thr, 0.0f, 1.0f);

    float beta = beta_min + (beta_max - beta_min) * (w_rate * w_grav);

    roll_deg  = (1.0f-beta)*roll_deg  + beta*roll_acc;
    pitch_deg = (1.0f-beta)*pitch_deg + beta*pitch_acc;
}
