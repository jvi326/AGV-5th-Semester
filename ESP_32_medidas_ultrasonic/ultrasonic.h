#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t trigPin;
    uint8_t echoPin;
} UltrasonicSensor;

void Ultrasonic_Init(UltrasonicSensor* sensor, uint8_t trigPin, uint8_t echoPin);
float Ultrasonic_ReadDistance(UltrasonicSensor* sensor);

#ifdef __cplusplus
}
#endif

#endif
