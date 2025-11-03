#include <stdint.h>
#include <stdbool.h>
#include "stm32f051x8.h"
#include "motor_controller.h"
#include "chassis.h"
#include "Bluetooth_USART2.h"
#include "IR.h"
#include "Global_Stop_Causes.h"
#include "uart_commands.h"
#include "stops.h"
#include "control.h"
#include "color_recipes.h"
#include "bt_config.h"

uint32_t SystemCoreClock = 8000000;  // define la frecuencia

#define RX_BUF_SIZE 32
#define MAX_MENSAJE 20

IRSensor IR_MostOuterRight = { GPIOB, 9};
IRSensor IR_OuterRight     = { GPIOB, 10};
IRSensor IR_CenterRight    = { GPIOB, 11};
IRSensor IR_Center         = { GPIOB, 12};
IRSensor IR_CenterLeft     = { GPIOB, 13};
IRSensor IR_OuterLeft      = { GPIOB, 14};
IRSensor IR_MostOuterLeft  = { GPIOB, 15};

volatile bool emergencyStop = 0;
volatile uint8_t WaitForAnswer = 0;
volatile int emergencyCounter = 0;
volatile int error_0_counter = 0;

volatile bool sensorStates[7];

bool justEnteredLineMode;
bool dis_detected;
int  lineFollowerMode = 0;

float forward_speed_LF = 0; // Choose based on desired speed (0.0 to 1.0)
float temp_P = 0.1f;
float temp_I = 0.0f;
float temp_D = 0.45f;

volatile uint8_t current_temperature = 0;
volatile uint8_t current_color = 0;
volatile uint8_t compare_color = 0;
Stop* currentStop;

volatile uint8_t bytes_in_terminal;
volatile uint8_t frame[4];
volatile uint8_t ack;
volatile uint8_t cmd;
volatile uint8_t sub_cmd;
volatile uint8_t data;

/* ===== Hooks para color_recipes (definidos FUERA de main) ===== */
bool cr_line_detected(void) {
    // criterio simple: el IR central está sobre la línea
    return sensorStates[3] ? true : false;
}

void cr_get_rgb_flags(uint8_t* r, uint8_t* g, uint8_t* b) {
    if (r) *r = stop_flags.red_flag   ? 1u : 0u;
    if (g) *g = stop_flags.green_flag ? 1u : 0u;
    if (b) *b = stop_flags.blue_flag  ? 1u : 0u;
}

// Lee directamente PB12 (IR central). Ajusta el sentido si tu sensor es activo-bajo.
static inline void RefreshCenterIR(void){
   sensorStates[3] = ( (GPIOB->IDR & (1u << 12)) != 0 );  // 1 = línea detectada
}

int main(void) {
    USART2_Init_Interrupt();
    System_Ready_Indicator();
    StopCauses_Init();

    UART_SetRole(ROLE_MASTER);   // define el rol
    USART1_Init();               // inicializa UART

    /* ===== Init de control y recetas ===== */
    control_init();   // encoders+PWM+SysTick del módulo de control
    cr_init();        // recetas (mask=0, wait=10 s por defecto)

    MotorController motorA;
    MotorController motorB;
    CHASSIS agv;

    // Initialize motor (Dir1, Dir2, PWM, BrakePin)
    Motor_Init(&motorA, 6, 7, 8, 5);
    Motor_Init(&motorB, 10, 11, 9, 4);
    Motor_Invert(&motorB, 1);
    Init_Chassis(&agv, motorA, motorB);
    set_AdvanceInverted(&agv, 1);

    LineFollower Follower = {IR_MostOuterRight, IR_OuterRight, IR_CenterRight, IR_Center, IR_CenterLeft, IR_OuterLeft, IR_MostOuterLeft};
    LineFollower_Init(&Follower);

    Treat_Failure_Flags = (Treat_Failure_Flags_t){1,1,1,1,1,1,1};

    // Test sequence / main loop
    while (1) {

    	if (rx_ready == 1) {
    	    // 1) Intenta aplicar on,mode,speed,mask ANTES del handler (que borra rx_buf)
    	    (void)BT_ApplyCsvConfig_fromRxBuf(rx_buf, rx_pos);

    	    // 2) Tu flujo original se mantiene
    	    USART2_HandleMessage(&agv);
    	}
        Update_ColorFlags();
        RefreshCenterIR();
        // Máquina de recetas de color: puede parar 10 s y reanudar control_update()
        cr_tick();
        if (cr_is_busy()) {
            // Si está actuando (parado/esperando/reanudando), evita que el resto
            // del loop toque los motores en esta iteración.
            continue;
        }

        if ((!emergencyStop) && (!stop_flags.distance1_flag) && (!stop_flags.distance2_flag) && (!stop_flags.color_flag)) {

            Treat_Failure_Flags = (Treat_Failure_Flags_t){1,1,1,1,1,1,1};

            if (lineFollowerMode == 0) {
                apply_CurrentSpeedsToMotors_noBrake_if_0(&agv);
            } else if (lineFollowerMode == 1) {
                if (justEnteredLineMode) {
                    resetPID();  // integral = 0, last_error = 0
                    justEnteredLineMode = 0;
                }
                apply_CurrentSpeedsToMotors_noBrake_if_0(&agv);
                LineFollower_FollowLine(&Follower, &agv, forward_speed_LF);
            } else if (lineFollowerMode == 2) {
                if (justEnteredLineMode) {
                    resetPID();  // integral = 0, last_error = 0
                    justEnteredLineMode = 0;
                }
                LineFollower_FollowLine_PID(&Follower, &agv, forward_speed_LF, temp_P, temp_I, temp_D);
            }

        } else {
            if ( (stop_flags.distance1_flag) || (stop_flags.distance2_flag) ) {
            	pause_Chassis_with_STOP(&agv);
            }

            /*
            if (WaitForAnswer == 0) { // Enviar solo una vez y esperar respuesta
                pause_Chassis(&agv);
                if (stop_flags.temperature_flag && Treat_Failure_Flags.temperature_Fail_cmd)
                {
                    // Enviar comando de temperatura
                    Master_RequestTemperature();
                    WaitForAnswer = 1;
                } else if (stop_flags.color_flag && Treat_Failure_Flags.color_Fail_cmd)
                {
                    // Enviar comando de color
                    Master_RequestColor();
                    WaitForAnswer = 1;
                }
            } */

            /* bytes_in_terminal = USART1_AvailableBytes(); */

            /*
            if ((WaitForAnswer == 1) && (bytes_in_terminal >= 4)) {
                pause_Chassis(&agv);
                for (uint8_t i = 0; i < 4; i++)
                    frame[i] = USART1_ReadByte();   // leer realmente los bytes (no peek)

                uint8_t ack     = frame[0];
                uint8_t cmd     = frame[1];
                uint8_t sub_cmd = frame[2];
                uint8_t data    = frame[3];

                if (ack == ACKNOWLEDGE)
                {
                    if (cmd == C_TEMPERATURE)
                    {
                        if (sub_cmd == RETRIEVE_TEMP){
                            current_temperature = data;
                            Treat_Failure_Flags.temperature_Fail_cmd = 0;
                        }
                    } else if (cmd == C_COLOR)
                    {
                        if (sub_cmd == RETRIEVE_COLOR){
                            current_color = data;
                            Treat_Failure_Flags.color_Fail_cmd = 0;
                        }
                    }

                    WaitForAnswer = 0;

                }

                USART1_ClearBuffer();  // limpia cualquier residuo
            }
            */

            /*
            // BLOQUE VIEJO DE COLOR: protégelo para que no choque con la receta
            if ( !cr_is_busy() && (lineFollowerMode == 1) && stop_flags.color_flag ) {

                Update_ColorFlags();
                int k = -1; // índice de la parada que se revisará

                if (stop_flags.red_flag) {
                    current_color = COLOR_RED; // rojo
                    k = 0; // primera instancia
                } else if (stop_flags.green_flag) {
                    current_color = COLOR_GREEN; // verde
                    k = 1; // segunda instancia
                } else if (stop_flags.blue_flag) {
                    current_color = COLOR_BLUE; // azul
                    k = 2; // tercera instancia
                } else {
                    current_color = COLOR_INVALID; // sin color detectado
                    break;
                }

                // Solo si hay un color válido
                if (k >= 0 && k < NUM_PARADAS) {
                    // Revisar el tercer valor (waitFlag)
                    if (Paradas[k].waitFlag == 1) {
                        // CASO 1: Color si debe esperar
                    	pause_Chassis_with_STOP(&agv);
                    } else {
                        // CASO 2: Color debe seguir
                        set_AdvanceSpeed(&agv, 0.15f);
                        set_TurnSpeed(&agv, 0.0f);
                        apply_CurrentSpeedsToMotors_noBrake_if_0(&agv);
                    }
                } else {
                    // CASO 3: Color no válido
                    set_AdvanceSpeed(&agv, 0.15f);
                    set_TurnSpeed(&agv, 0.0f);
                    apply_CurrentSpeedsToMotors_noBrake_if_0(&agv);
                }
            }
            */
        }
    }
}

