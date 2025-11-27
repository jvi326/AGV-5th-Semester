#include <stdint.h>
#include <math.h>
#include "stm32f051x8.h"

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

volatile uint32_t tick_count = 0;

uint32_t millis(void){ return tick_count; }

void SysTick_Handler(void){ tick_count++; }

void delay(uint32_t ms){
    uint32_t t = millis();
    while ((millis()-t) < ms) {
        __NOP();
    }
}

// ====== ESTADO DE NAVEGACIÓN ======
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

float kv = 1.0f;      // ganancia de velocidad lineal
const float kw = 0.6f;      // ganancia de velocidad angular

// Para que el giro (NAV_ALIGN) sea más suave
const float kw_align        = 0.7f;  // ganancia angular sólo para alinearse
const float omega_align_max = 0.6f;  // [rad/s] límite para el giro suave

const float v_max     = 0.5f;   // [m/s] máx velocidad lineal
const float v_min     = 0.2f;   // [m/s] máx velocidad lineal
const float omega_max = 1.5f;   // [rad/s] máx velocidad angular

const float dist_tol  = 0.10f;  // [m] tolerancia de distancia al objetivo
const float ang_tol   = 0.10f;  // [rad] (~3 grados) tolerancia de ángulo

volatile float dist = 0.0f;     // [m]
volatile float theta_goal = 0.0f;     // [rad]


const float S_STEP = 0.1f;   // 50 cm entre puntos
static float s_traj = 0.0f;  // acumulado de trayectoria





/* ====================== MAIN ====================== */
int main(void)
{
    TIM1_Encoder_Init();    // M1 encoder (derecha)
    TIM2_Encoder_Init();    // M2 encoder (izquierda)
    PWM_init_both();        // TIM3 CH3/CH4 + pines dirección

    // SysTick 1 ms @ 8 MHz
    SysTick->LOAD = 8000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;

    nav_state = NAV_ALIGN;   // al iniciar: primero alinear ángulo


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

            /* ====== ODOMETRÍA (posición x, y, theta) ======
             * Asumimos:
             *  - Motor 1 (rpm_f1) = rueda DERECHA
             *  - Motor 2 (rpm_f2) = rueda IZQUIERDA
             */
            float Wr = rpm_f1 * 2.0f * 3.14159265f / 60.0f;  // [rad/s]
            float Wl = rpm_f2 * 2.0f * 3.14159265f / 60.0f;  // [rad/s]

            float v = R_WHEEL * 0.5f * (Wr + Wl);            // [m/s]
            omega  = R_WHEEL * (Wr - Wl) / L_AXLE;           // [rad/s]

            v_x = v * cosf(theta);
            v_y = v * sinf(theta);

            x_pos += v_x * Ts;
            y_pos += v_y * Ts;
            theta += omega * Ts;

            // Normalizar theta a [-pi, pi]
            if (theta > 3.14159265f)        theta -= 2.0f * 3.14159265f;
            else if (theta < -3.14159265f)  theta += 2.0f * 3.14159265f;

            theta_deg = theta * 180.0f / 3.14159265f;

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

            // En Live Expressions:
            // x_pos, y_pos, theta_deg, x_ref, y_ref, rpm_des1, rpm_des2, rpm_f1, rpm_f2
        }
    }
}

/* ============== Función de navegación ============== */
void navigation_step(float Ts)
{
    (void)Ts;

    // --- Estado anterior para detectar cambios de modo ---
    static nav_state_t last_state = NAV_ALIGN;

    // Error de posición
    float dx = x_ref - x_pos;
    float dy = y_ref - y_pos;

    dist = sqrtf(dx*dx + dy*dy);

    // Ángulo deseado hacia el punto (respecto al mundo)
    theta_goal = atan2f(dy, dx);

    // Error angular respecto al robot
    float err_theta = theta_goal - theta;
    err_theta = wrapToPi(err_theta);

    // 1) ¿ya llegué a este punto?  -> SOLO usamos distancia YA QUE SI LE PONES ANGULO CON AND POCAS VECES ES VERDADERO
    if (dist < dist_tol) {
        function();              // siguiente punto de la trayectoria
        last_state = nav_state;
        return;
    }

    float v_cmd     = 0.0f;
    float omega_cmd = 0.0f;

    // Histeresis angular
    const float ang_tol_align   = 0.25f;  // ~14°
    const float ang_tol_realign = 0.80f;  // ~46°

    // --- Si estamos avanzando y el error se hizo MUY grande -> regresar a alinear ---
    if (nav_state == NAV_GO && fabsf(err_theta) > ang_tol_realign) {
        nav_state = NAV_ALIGN;
    }

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


void function(void)
{
    s_traj += S_STEP;


    x_ref = s_traj;
    y_ref = s_traj;
}


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
