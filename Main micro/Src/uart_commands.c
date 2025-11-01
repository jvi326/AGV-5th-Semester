#include "uart_commands.h"

// --- Buffer circular para recepción ---
#define UART_RX_BUFFER_SIZE 64
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_rx_head = 0;
static volatile uint8_t uart_rx_tail = 0;

static UART_Role current_role = ROLE_SLAVE;

extern uint32_t SystemCoreClock;

void UART_SetRole(UART_Role role) {
    current_role = role;
}

UART_Role UART_GetRole(void) {
    return current_role;
}

// ======================================================
//               CONFIGURACIÓN USART
// ======================================================
void USART1_Init(void)
{
    // --- Habilitar relojes ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;     // PB6, PB7
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // --- Configurar PB6 (TX) y PB7 (RX) como AF0 ---
    GPIOB->MODER  &= ~((3u << (6 * 2)) | (3u << (7 * 2)));   // limpiar
    GPIOB->MODER  |=  ((2u << (6 * 2)) | (2u << (7 * 2)));   // modo AF
    GPIOB->AFR[0] &= ~((0xFu << (6 * 4)) | (0xFu << (7 * 4))); // AF0 = USART1
    GPIOB->OTYPER &= ~((1u << 6) | (1u << 7));               // push-pull
    GPIOB->OSPEEDR |= (3u << (6 * 2)) | (3u << (7 * 2));     // alta velocidad
    GPIOB->PUPDR  &= ~((3u << (6 * 2)) | (3u << (7 * 2)));   // limpiar pull
    GPIOB->PUPDR  |=  ((1u << (6 * 2)) | (1u << (7 * 2)));   // pull-up

    // --- Configurar Baudrate ---
    USART1->BRR = SystemCoreClock / USART_BAUDRATE;

    // --- Habilitar USART con interrupción RXNE ---
    USART1->CR1 = 0;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    // Esperar a que esté listo
    while (!(USART1->ISR & USART_ISR_TEACK)) {}
    while (!(USART1->ISR & USART_ISR_REACK)) {}

    // --- Habilitar interrupción USART1 en NVIC ---
    NVIC_EnableIRQ(USART1_IRQn);
}


// ======================================================
//              ENVÍO Y RECEPCIÓN DE BYTES
// ======================================================
void USART1_SendByte(uint8_t data)
{
    uint32_t timeout = 100000;
    while (!(USART1->ISR & USART_ISR_TXE))
    {
        if (--timeout == 0) return; // evita bloqueo total
    }
    USART1->TDR = data;
}

// Lee un byte del buffer circular
uint8_t USART1_ReadByte(void)
{
    if (uart_rx_head == uart_rx_tail)
        return 0; // buffer vacío

    uint8_t data = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return data;
}

uint8_t USART1_DataAvailable(void)
{
    return (uart_rx_head != uart_rx_tail);
}

// ======================================================
//              INTERRUPCIÓN USART1 RX
// ======================================================
void USART1_IRQHandler(void)
{
    if (USART1->ISR & USART_ISR_RXNE)
    {
        uint8_t data = (uint8_t)USART1->RDR;  // Leer dato recibido limpia RXNE
        uint8_t next_head = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;

        // Si el buffer no está lleno, guardar el byte
        if (next_head != uart_rx_tail)
        {
            uart_rx_buffer[uart_rx_head] = data;
            uart_rx_head = next_head;
        }
        // Si está lleno, simplemente se descarta (o podrías manejar un flag de overflow)
    }
}

// ======================================================
//        FUNCIONES DE ENVÍO (ROL MAESTRO)
// ======================================================

// Enviar un comando/subcomando
void Master_SendCommand(uint8_t cmd, uint8_t subcmd)
{
    USART1_SendByte(cmd);
    USART1_SendByte(subcmd);
}

// ======================================================
//        PROCESAMIENTO DE COMANDOS (ROL ESCLAVO)
// ======================================================

void Slave_ProcessCommand(uint8_t cmd, uint8_t subcmd)
{
    switch (cmd)
    {
        case C_COLOR:
            if (subcmd == RETRIEVE_COLOR)
            {
                uint8_t color_detected = COLOR_GREEN;
                USART1_SendByte(ACKNOWLEDGE);
                USART1_SendByte(cmd);
                USART1_SendByte(color_detected);
            }
            else if (subcmd == CHECK_COLOR)
            {
                uint8_t color_checked = COLOR_RED;
                uint8_t is_true = FALSE;

                USART1_SendByte(ACKNOWLEDGE);
                USART1_SendByte(cmd);
                USART1_SendByte(color_checked);
                USART1_SendByte(is_true);
            }
            break;

        case C_TEMPERATURE:
            if (subcmd == RETRIEVE_TEMP)
            {
                uint8_t temperature = 0x19; // 25°C
                USART1_SendByte(ACKNOWLEDGE);
                USART1_SendByte(cmd);
                USART1_SendByte(temperature);
            }
            break;

        default:
            break;
    }
}

// ======================================================
//        FUNCIONES DE COMUNICACIÓN (ROL MAESTRO)
// ======================================================

// Solicita la temperatura actual al esclavo
void Master_RequestTemperature(void)
{
    Master_SendCommand(C_TEMPERATURE, RETRIEVE_TEMP);
}

// Solicita el color detectado al esclavo
void Master_RequestColor(void)
{
    Master_SendCommand(C_COLOR, RETRIEVE_COLOR);
}

// Verifica si el color detectado coincide con uno específico
// (COLOR_RED, COLOR_GREEN o COLOR_BLUE)
void Master_CheckColor(uint8_t color)
{
    USART1_SendByte(C_COLOR);
    USART1_SendByte(CHECK_COLOR);
    USART1_SendByte(color);
}

// Envía un umbral para sensores ultrasónicos
void Master_SetThreshold(uint8_t value)
{
    USART1_SendByte(C_ULTRASONIC);
    USART1_SendByte(SET_THRESHOLD);
    USART1_SendByte(value);
}

// Solicita lectura de distancia ultrasónica
void Master_RequestDistance(void)
{
    Master_SendCommand(C_ULTRASONIC, READ_DISTANCE);
}

uint8_t USART1_PeekByte(uint16_t index)
{
    uint16_t pos = (uart_rx_tail + index) % UART_RX_BUFFER_SIZE;
    return uart_rx_buffer[pos];
}

void USART1_ClearBuffer(void)
{
    uart_rx_head = uart_rx_tail;
}

uint8_t USART1_AvailableBytes(void)
{
    if (uart_rx_head >= uart_rx_tail)
        return uart_rx_head - uart_rx_tail;
    else
        return UART_RX_BUFFER_SIZE - uart_rx_tail + uart_rx_head;
}
