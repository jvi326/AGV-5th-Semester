#ifndef UART_COMMANDS_H_
#define UART_COMMANDS_H_

#include "stm32f051x8.h"
#include <stdint.h>
#include <stddef.h> // <-- Necesario para size_t

#define MAX_PAYLOAD  16
#define USART_TX_PIN     9    // PA9
#define USART_RX_PIN     10   // PA10
#define USART_BAUDRATE   9600

typedef struct {
    uint8_t data[MAX_PAYLOAD];
    size_t length;
} Payload;

// Comandos principales
typedef enum {
    C_COLOR        = 0x01,
    C_TEMPERATURE  = 0x02,
    C_ULTRASONIC   = 0x03,
    C_REVOLUTIONS  = 0x04
} CommandType;

// Subcomandos
typedef enum {
    RETRIEVE_TEMP  = 0x05,
    SET_THRESHOLD  = 0x06,
	RETRIEVE_COLOR = 0x07,
	CHECK_COLOR    = 0x08,
    READ_DISTANCE  = 0x09
} SubCommandType;

// Colores
typedef enum {
    COLOR_RED   = 0x10,
    COLOR_GREEN = 0x11,
    COLOR_BLUE  = 0x12,
	COLOR_INVALID  = 0x13
} ColorType;

typedef enum {
    ACKNOWLEDGE   = 0xA0,
    TRUE  = 0xFF,
    FALSE = 0x00
} Answers;

typedef enum {
    ROLE_MASTER = 0,
    ROLE_SLAVE
} UART_Role;

void USART1_Init(void);
void USART1_SendByte(uint8_t data);
uint8_t USART1_ReadByte(void);
uint8_t USART1_DataAvailable(void);

// Rol (maestro o esclavo)
void UART_SetRole(UART_Role role);
UART_Role UART_GetRole(void);

void Master_RequestTemperature(void);
void Master_RequestColor(void);
void Master_CheckColor(uint8_t color);
void Master_SetThreshold(uint8_t value);

uint8_t USART1_PeekByte(uint16_t index);
void USART1_ClearBuffer(void);
uint8_t USART1_AvailableBytes(void);

#endif
