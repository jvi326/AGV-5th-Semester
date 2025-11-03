#include "color_recipes.h"
#include "control.h"
#include "stm32f051x8.h"

extern uint32_t millis(void);
extern bool justEnteredLineMode;   // para que el follower haga reset al regresar


static volatile uint8_t  g_mask = 0;         // bits R(4) G(2) B(1)
static volatile uint32_t g_wait_ms = 10000;  // 10 s
static CR_State          g_state = CR_IDLE;
static uint32_t          g_deadline_ms = 0;

void cr_init(void){ g_mask=0; g_wait_ms=10000; g_state=CR_IDLE; g_deadline_ms=0; }
void cr_set_mask(uint8_t mask){ g_mask = (uint8_t)(mask & 0x07); }
void cr_set_wait_ms(uint32_t ms){ g_wait_ms = ms; }
bool cr_is_busy(void){ return (g_state != CR_IDLE); }

/* Hooks débiles (por si olvidas implementarlos) */
__attribute__((weak)) bool cr_line_detected(void){ return false; }
__attribute__((weak)) void cr_get_rgb_flags(uint8_t* r, uint8_t* g, uint8_t* b){
    if (r) *r=0; if (g) *g=0; if (b) *b=0;
}

void cr_tick(void){
    uint8_t r=0,g=0,b=0; cr_get_rgb_flags(&r,&g,&b);
    uint8_t idx = cr_color_index(r,g,b);

    switch (g_state){
    case CR_IDLE:
        if ((idx & g_mask) != 0){
            control_reset_all();
            control_coast();
            g_deadline_ms = millis() + g_wait_ms;
            g_state = CR_WAITING_AT_COLOR;
        }
        break;

    case CR_WAITING_AT_COLOR:
        control_coast();
        if ((int32_t)(millis() - g_deadline_ms) >= 0){
            control_reset_all();
            g_state = CR_RESUME_UNTIL_LINE;
            // (opcional) guarda de tiempo máx para buscar línea:
            // g_resume_guard_ms = millis() + 5000;
        }
        break;

    /* ======= PÉGALO AQUÍ: sustituye el case por este ======= */
    case CR_RESUME_UNTIL_LINE:
        // Corre tu control (ruedas) hasta que el IR central detecte de nuevo la línea
        control_update();

        if (cr_line_detected()){
            control_reset_all();     // suelta motores y limpia integradores
            justEnteredLineMode = 1; // el seguidor hará resetPID al retomar
            g_state = CR_IDLE;       // entregar control al follower
        }

        // (opcional) si pusiste guarda de tiempo:
        // if ((int32_t)(millis() - g_resume_guard_ms) >= 0) {
        //     control_reset_all();
        //     justEnteredLineMode = 1;
        //     g_state = CR_IDLE;
        // }

        break;

    default:
        g_state = CR_IDLE;
        break;
    }
}
