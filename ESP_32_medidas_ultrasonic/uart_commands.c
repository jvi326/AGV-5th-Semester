#include "uart_commands.h"
#include <string.h>
#include <stdio.h>
#include "globals.h"

// -------------------- UART Initialization --------------------
void USART_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, RX_BUFFER_SIZE, RX_BUFFER_SIZE, 0, NULL, 0);

    printf("[UART2] Initialized on TX=%d RX=%d baud=%d\n", UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);
}

// -------------------- Create Payload --------------------
Payload CreatePayload(uint8_t cmd_type, uint8_t sub_cmd, uint8_t* data, size_t data_len) {
    Payload p;
    p.length = 0;
    if (data_len > MAX_PAYLOAD - 2) data_len = MAX_PAYLOAD - 2;

    p.data[p.length++] = cmd_type;
    p.data[p.length++] = sub_cmd;
    for (size_t i=0; i<data_len; i++) p.data[p.length++] = data[i];
    return p;
}

// -------------------- Send Payload --------------------
void SendPayload(Payload* p) {
    uart_write_bytes(UART_NUM, (const char*)p->data, p->length);
}

// -------------------- Receive Payload --------------------
size_t USART_Receive(uint8_t* buffer, size_t max_length) {
    int len = uart_read_bytes(UART_NUM, buffer, max_length, 100 / portTICK_PERIOD_MS);
    return (len > 0) ? len : 0;
}

// -------------------- Command Interpreter --------------------
void HandleCommand(uint8_t* buffer, size_t length) {
    if (length < 2) return;

    uint8_t type = buffer[0];
    uint8_t sub  = buffer[1];
    uint8_t* data = &buffer[2];
    size_t data_len = length - 2;

    switch (type) {
        case CMD_ULTRASONICOS:
            switch(sub) {
                case SUBCMD_UMBRAL:
                    Ultrasonic_SetThreshold(data, data_len);
                    break;
                case SUBCMD_MEDIDAS:
                    Ultrasonic_ReadSensors(data, data_len);
                    break;
            }
            break;
        case CMD_COLOR:
            switch(sub) {
                case SUBCMD_COLOR_SET:
                    Color_Set(data, data_len);
                    break;
                case SUBCMD_COLOR_GET:
                    Color_Get(data, data_len);
                    break;
            }
            break;
        case CMD_REVOLUCIONES:
            break;
    }
}

// -------------------- Command Handlers -------------------
void Ultrasonic_SetThreshold(uint8_t* data, size_t length) {
    if (length >= 1) {
        uint8_t threshold_hex = data[0]; //Recibe en hex
        threshold_cm = (uint16_t)threshold_hex; //Transforma a decimal
        uint8_t ack = CMD_Acknowledge;  //Valor deacknowledge
        new_threshold = 1; 
        uart_write_bytes(UART_NUM, &ack, 1); //Manda valor deacknowledge
        uart_write_bytes(UART_NUM, &threshold_hex, 1); //Manda valor hex recibido
    }
}


void Ultrasonic_ReadSensors(uint8_t* data, size_t length) {
    // Aquí puedes enviar distancias si quieres
}

void Color_Set(uint8_t* data, size_t length) {
    if(length>=3){
        // ejemplo: data[0]=R, data[1]=G, data[2]=B
    }
}

void Color_Get(uint8_t* data, size_t length) {
    // ejemplo: enviar color actual
}
