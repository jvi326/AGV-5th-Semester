#include "bt_config.h"
#include "Bluetooth_USART2.h"  // usamos tu atof()
#include "color_recipes.h"
#include "control.h"

/* Estas 3 variables existen en main.c */
extern int   lineFollowerMode;
extern float forward_speed_LF;
extern bool  justEnteredLineMode;

static int split4(const volatile uint8_t* rx, uint16_t n, float out[4]){
    int idx=0; int start=0;
    for (uint16_t i=0;i<n && idx<4;i++){
        if (rx[i]==',' || rx[i]=='\n' || rx[i]=='\r'){
            /* antes: out[idx++] = atof(&rx[start], i-start); */
            out[idx++] = atof((volatile uint8_t*)&rx[start], i-start);
            start = i+1;
        }
    }
    if (idx<4 && start<n) {
        /* antes: out[idx++] = atof(&rx[start], n-start); */
        out[idx++] = atof((volatile uint8_t*)&rx[start], n-start);
    }
    return (idx==4)?1:0;
}

/* on,mode,speed,mask : 1,2,0.125,7 */
int BT_ApplyCsvConfig_fromRxBuf(const volatile uint8_t* rx, uint16_t nbytes){
    float v[4];
    if (!split4(rx,nbytes,v)) return 0;

    int   on   = (int)v[0];
    int   mode = (int)v[1];
    float sp   =       v[2];
    int   mask = (int)v[3];

    if (mode<0) mode=0;
    if (mode>2) mode=2;
    if (lineFollowerMode != mode) justEnteredLineMode = 1;
    lineFollowerMode = mode;

    if (sp<0.0f) sp=0.0f;
    if (sp>1.0f) sp=1.0f;
    forward_speed_LF = sp;

    if (mask<0) mask=0;
    if (mask>7) mask=7;
    cr_set_mask((uint8_t)mask);
    cr_set_wait_ms(10000u);          // 10 s

    if (on){ control_reset_all(); }
    else   { control_reset_all(); control_coast(); }

    return 1;
}
