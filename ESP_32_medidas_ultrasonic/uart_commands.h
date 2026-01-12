#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"

// -------------------- UART Configuration --------------------
#define UART_NUM        UART_NUM_2
#define UART_BAUDRATE   115200
#define UART_TX_PIN     17
#define UART_RX_PIN     16
#define MAX_PAYLOAD     32
#define RX_BUFFER_SIZE  256

// -------------------- Command Types --------------------
typedef enum {
    CMD_ULTRASONICOS = 0x01,
    CMD_COLOR        = 0x02,
    CMD_REVOLUCIONES = 0x03,
    CMD_Acknowledge = 0x40 
} CommandType;

// -------------------- Subcommands --------------------
typedef enum {
    SUBCMD_UMBRAL      = 0x04,
    SUBCMD_MEDIDAS     = 0x05
} UltrasonicSubCommand;

typedef enum {
    SUBCMD_COLOR_SET   = 0x06,
    SUBCMD_COLOR_GET   = 0x07
} ColorSubCommand;

// -------------------- Payload Struct --------------------
typedef struct {
    uint8_t data[MAX_PAYLOAD];
    size_t length;
} Payload;

#ifdef __cplusplus
extern "C" {
#endif

void USART_Init(void);
Payload CreatePayload(uint8_t cmd_type, uint8_t sub_cmd, uint8_t* data, size_t data_len);
void SendPayload(Payload* p);
size_t USART_Receive(uint8_t* buffer, size_t max_length);
void HandleCommand(uint8_t* buffer, size_t length);
void Ultrasonic_SetThreshold(uint8_t* data, size_t length);
void Ultrasonic_ReadSensors(uint8_t* data, size_t length);
void Color_Set(uint8_t* data, size_t length);
void Color_Get(uint8_t* data, size_t length);

#ifdef __cplusplus
}

#endif

#endif
