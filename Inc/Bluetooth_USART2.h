#ifndef BLUETOOTH_USART2_H_
#define BLUETOOTH_USART2_H_

#include "stm32f051x8.h"
#include "motor_controller.h"
#include "chassis.h"
#include "ultrasonicos.h"
#include <stdint.h>
#include <stdbool.h>

// ===================== CONSTANTS =====================
#define MAX_NUMBERS 10
#define MAX_DIGITS  5
#define MAX_ARRAYS  10
#define RX_BUF_SIZE 32
#define MAX_MENSAJE 20

// ===================== STRUCTURES =====================
typedef struct {
    uint8_t array[MAX_DIGITS + 1]; // +1 for terminator or newline
    uint8_t length;
} NumberArray;

// ===================== EXTERNAL VARIABLES =====================
extern volatile NumberArray parsedArrays[MAX_ARRAYS];
extern volatile uint8_t numParsedArrays;

extern volatile Numeros indicacionesArray[MAX_NUMBERS];
extern volatile uint8_t numindicaciones;

extern volatile float mensaje[MAX_MENSAJE];
extern volatile uint8_t mensaje_size;

extern volatile uint8_t rx_buf[RX_BUF_SIZE];
extern volatile uint16_t rx_pos;
extern volatile uint8_t rx_ready;
extern volatile uint8_t bt_ready;

extern bool flag_Bluetooth_state;

// ===================== FUNCTION PROTOTYPES =====================

// Initialization
void System_Ready_Indicator(void);
void USART2_Init_Interrupt(void);

// Communication
void USART2_IRQHandler(void);
void USART2_HandleMessage(CHASSIS* AGV_Chassis);
void USART2_SendChar(char c);
void USART2_SendString(const char *str);
void USART2_SendFloat(float value, uint8_t decimalPlaces);

// Parsing and conversion
uint8_t parseCSV(const volatile uint8_t* rx_buf, uint16_t buf_size, volatile NumberArray* result);
uint8_t arrayToArrayIntOrFloat(volatile NumberArray* numbersAsArray, uint8_t numberOfArrays, volatile Numeros* result);

// Utility
void delay_ms(uint32_t ms);
int atoi(uint8_t* data, int size);
char* itoa(int num, char* str, int base);
float atof(volatile uint8_t* data, int size);
int strncmp(const char *s1, const char *s2, int n);
void decideNegPos(volatile Numeros* numeros, uint8_t count);

void USART2_SendSensorData(const volatile bool *sensorStates, uint8_t count, float floatValue, int intValue);
void SendObstacleStatusFloat(float detected);
void Ultrasonicos_SendDistance1(void);

#endif /* BLUETOOTH_USART2_H_ */
