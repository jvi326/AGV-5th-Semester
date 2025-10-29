#include "ultrasonic.h"
#include "globals.h"

void Ultrasonic_Init(UltrasonicSensor* sensor, uint8_t trigPin, uint8_t echoPin) {
    sensor->trigPin = trigPin;
    sensor->echoPin = echoPin;

    pinMode(sensor->trigPin, OUTPUT);
    pinMode(sensor->echoPin, INPUT);
}

float Ultrasonic_ReadDistance(UltrasonicSensor* sensor) {
    // Pulso de 10 µs en TRIG
    digitalWrite(sensor->trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor->trigPin, LOW);

    // Medir duración del pulso en ECHO
    long duration = pulseIn(sensor->echoPin, HIGH, 30000UL); // timeout 30 ms

    if (duration == 0) return -1.0; // timeout o error

    // Convertir a cm
    float distance = duration / 58.0f;
    return distance;
}
