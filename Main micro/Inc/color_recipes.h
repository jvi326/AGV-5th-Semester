#ifndef COLOR_RECIPES_H
#define COLOR_RECIPES_H

#include <stdint.h>
#include <stdbool.h>

/* R=4, G=2, B=1 -> mask 0..7 */
typedef enum { CR_IDLE=0, CR_WAITING_AT_COLOR, CR_RESUME_UNTIL_LINE } CR_State;

void  cr_init(void);                // mask=0, wait=10000 ms
void  cr_set_mask(uint8_t mask);    // 0..7
void  cr_set_wait_ms(uint32_t ms);  // p.ej., 10000
void  cr_tick(void);                // llámalo cada iteración del while(1)
bool  cr_is_busy(void);             // true cuando el módulo toma control

/* Hooks (los defines tú en main.c) */
bool  cr_line_detected(void);
void  cr_get_rgb_flags(uint8_t* r, uint8_t* g, uint8_t* b);

uint8_t cr_get_mask(void);  // 0..7; >0 = receta activa


/* Utilidad */
static inline uint8_t cr_color_index(uint8_t r, uint8_t g, uint8_t b) {
    return (uint8_t)((r?4:0) | (g?2:0) | (b?1:0));
}
#endif
