#ifndef WS2812_H
#define WS2812_H

#include <Adafruit_NeoPixel.h>

// ---- CONFIG WS2812 ----
#define WS_PIN       22        // Pin de datos WS2812B (D22)
#define NUM_LEDS     80        // Total de LEDs en la tira

// ---- OBJETO PIXEL ----
static Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(NUM_LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);

// ------------------------------------------------------------
// Inicializa la tira LED
// ------------------------------------------------------------
static inline void WS2812_Init() {
    strip.begin();
    strip.show();             // apaga todo
    strip.setBrightness(50);
}

// ------------------------------------------------------------
// Función fija: 40 verdes + 40 rojos
// ------------------------------------------------------------
static inline void WS2812_GreenRedPattern() {

    // Primeros 40 verdes
    for (int i = 0; i < 40 && i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    }

    // Siguientes 40 rojos
    for (int i = 40; i < 80 && i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
    }

    strip.show();
}

// ------------------------------------------------------------
// Función genérica: N verdes + M rojos
// ------------------------------------------------------------
static inline void WS2812_SetPattern(int greenCount, int redCount) {

    // --- Verdes ---
    int idx = 0;
    for (; idx < greenCount && idx < NUM_LEDS; idx++) {
        strip.setPixelColor(idx, strip.Color(0, 255, 0));
    }

    // --- Rojos ---
    for (int r = 0; r < redCount && idx < NUM_LEDS; r++, idx++) {
        strip.setPixelColor(idx, strip.Color(255, 0, 0));
    }

    strip.show();
}

// ------------------------------------------------------------
// Función general con colores personalizables
// ------------------------------------------------------------
static inline void WS2812_SetTwoColors(int countA, int countB,
                                       uint32_t colorA, uint32_t colorB)
{
    int idx = 0;

    // --- Bloque A ---
    for (; idx < countA && idx < NUM_LEDS; idx++) {
        strip.setPixelColor(idx, colorA);
    }

    // --- Bloque B ---
    for (int r = 0; r < countB && idx < NUM_LEDS; r++, idx++) {
        strip.setPixelColor(idx, colorB);
    }

    strip.show();
}

// ------------------------------------------------------------
// Apaga todos los LEDs
// ------------------------------------------------------------
static inline void WS2812_Clear() {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();
}

#endif
