/*
 * Bluetooth_USART2.c
 *
 *  Created on: Oct 8, 2025
 *      Author: javie
 */

/* System Initialization Functions */

/*
Peripherials used:
PA2 TX
PA3 RX
PA7 State

 */


#include "stm32f051x8.h"
#include "Bluetooth_USART2.h"
#include "motor_controller.h"
#include "chassis.h"
#include <stdint.h>
#include <stdbool.h>

#define RX_BUF_SIZE 32
#define MAX_MENSAJE 20

// ===================== GLOBAL DEFINITIONS =====================
volatile NumberArray parsedArrays[MAX_ARRAYS];
volatile uint8_t numParsedArrays = 0;

volatile Numeros indicacionesArray[MAX_NUMBERS];
volatile uint8_t numindicaciones = 0;

volatile float mensaje[MAX_MENSAJE];
volatile uint8_t mensaje_size = 0;

volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_pos = 0;
volatile uint8_t rx_ready = 0;
volatile uint8_t bt_ready = 0;

void System_Ready_Indicator(void) {
  //  USART2_SendString("\r\n=== STM32 Bluetooth Chassis Demo ===\r\n");
   // USART2_SendString("Send numbers csv for instructions\r\n");
}

void USART2_Init_Interrupt(void) {
	// Enable clocks
    RCC->AHBENR  |= (1 << 17);  // GPIOA
    RCC->APB1ENR |= (1 << 17);  // USART2

    // Configure PA2 (TX) and PA3 (RX)
    GPIOA->MODER &= ~((3 << 2*2) | (3 << 2*3));
    GPIOA->MODER |=  ((2 << 2*2) | (2 << 2*3));  // Alternate function mode

    // Set AF1 for USART1
    GPIOA->AFR[0] &= ~((0xF << 4*2) | (0xF << 4*3));
    GPIOA->AFR[0] |=  ((1 << 4*2) | (1 << 4*3));

    // Baud rate 9600 (8MHz clock)
    USART2->BRR = (8000000 / 9600);

    // Enable USART with interrupts
    USART2->CR1 |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5);
    // UE: USART Enable
    // RE: Receiver Enable
    // TE: Transmitter Enable
    // RXNEIE: RX Not Empty Interrupt Enable
    USART2->CR1 &= ~(1 << 6);  // Disable TC interrupt, no interruptions when TX is used

    // NVIC configuration
    USART2->ICR = 0xFFFFFFFF;			//Clear all interrups flags
    NVIC_EnableIRQ(USART2_IRQn);		//Enable USART1 global interrupt
    NVIC_SetPriority(USART2_IRQn, 0);	//Set priority
}

/* Communication Functions */
void USART2_IRQHandler(void){
    if(USART2->ISR & (1 << 5)) {
        uint8_t c = USART2->RDR; // Read and clear RXNE flag

        if(rx_pos < RX_BUF_SIZE-1) {
            rx_buf[rx_pos++] = c;
            if(c == '\n' || c == '\r') {
                rx_buf[rx_pos] = '\0';
                rx_ready = 1;
            }
        } else {
            rx_pos = 0; // Reset on overflow
        }
    }
}

void USART2_HandleMessage(CHASSIS* AGV_Chassis) {


	//USART2_SendString("Echo: ");
	//USART2_SendString((char*)rx_buf);

	uint8_t numArrays = parseCSV(rx_buf, rx_pos, parsedArrays);
	uint8_t numArrayIntFloat = arrayToArrayIntOrFloat(parsedArrays, numArrays, indicacionesArray);
	decideDir(AGV_Chassis, indicacionesArray, numArrayIntFloat);

	// Reset for next message
	rx_pos = 0;
	rx_ready = 0;
}


void USART2_SendChar(char c) {
    while (!(USART2->ISR & (1 << 7)));
    USART2->TDR = c;
}

void USART2_SendString(const char *str) {
    while (*str) USART2_SendChar(*str++);
}

void USART2_SendFloat(float value, uint8_t decimalPlaces) {
    // Handle negative numbers
    if (value < 0) {
        USART2_SendChar('-');
        value = -value;
    }

    // Extract integer part
    int integerPart = (int)value;
    char buffer[12];
    USART2_SendString(itoa(integerPart, buffer, 10));  // Send integer part

    // Only send decimal point if needed
    if (decimalPlaces > 0) {
        USART2_SendChar('.');

        // Extract and send fractional part
        float fractionalPart = value - integerPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            USART2_SendChar('0' + digit);
            fractionalPart -= digit;
        }
    }
}

/* Utility Functions */
void delay_ms(uint32_t ms) {
	/**/
    RCC->APB1ENR |= (1<<0); //Enable clock for TIM2

    TIM2->PSC = 8000000/1000 - 1; //Set 1kHz period

    TIM2->ARR = ms;	//Set goal, user defined, by miliseconds

    TIM2->CNT = 0; //Clears the counter

    TIM2->CR1 |= (1<<0); //Count enable

    while (!(TIM2->SR & TIM_SR_UIF)); //Bit set by hardware when the registers are updated

    TIM2->SR &= ~(1<<0); //Cleared by software
    TIM2->CR1 &= ~(1<<0); //Disable counter
    RCC->APB1ENR &= ~(1<<0); //Disable timer TIM2
}

// Array to integer
int atoi(uint8_t* data, int size) {
    int result = 0;
    bool isNegative = false;
    int i = 0;

    if (size == 0) {
        return 0;  // Buffer vacío
    }

    // Verificar signo negativo (si el primer byte es '-')
    if (data[0] == '-') {
        isNegative = true;
        i = 1;
    }

    // Procesar cada byte hasta encontrar '\n', '\r' o fin del buffer
    for (; i < size; ++i) {
        // Si encuentra un fin de línea, terminar
        if (data[i] == '\n' || data[i] == '\r') {
            break;
        }

        // Verificar que sea un dígito válido (0-9)
        if (data[i] < '0' || data[i] > '9') {
            return 0;  // Carácter no válido
        }
        result = result * 10 + (data[i] - '0');
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}

// Integer to array
char* itoa(int num, char* str, int base) {
    int i = 0;
    bool isNegative = false;

    // Handle 0 explicitly
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // Handle negative numbers (only for base 10)
    if (num < 0 && base == 10) {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    // Append negative sign (if needed)
    if (isNegative) {
        str[i++] = '-';
    }

    // Reverse the string
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }

    // Null-terminate the string
    str[i] = '\0';
    return str;
}

// Array to float
float atof(volatile uint8_t* data, int size) {
    float result = 0.0;
    bool isNegative = false;
    bool hasDecimal = false;
    float fractionMultiplier = 0.1;
    int i = 0;

    if (size == 0) {
        return 0.0;  // Empty buffer
    }

    // Check for sign
    if (data[0] == '-') {
        isNegative = true;
        i = 1;
    }

    // Process each byte
    for (; i < size; ++i) {
        // If we find a line terminator, stop
        if (data[i] == '\n' || data[i] == '\r') {
            break;
        }

        // Check for decimal point
        if (data[i] == '.') {
            if (hasDecimal) {
                return 0.0;  // Multiple decimal points, invalid
            }
            hasDecimal = true;
            continue;
        }

        // Verify it's a valid digit
        if (data[i] < '0' || data[i] > '9') {
            return 0.0;  // Invalid character
        }

        if (hasDecimal) {
            // Fractional part
            result += (data[i] - '0') * fractionMultiplier;
            fractionMultiplier *= 0.1;
        } else {
            // Integer part
            result = result * 10.0 + (data[i] - '0');
        }
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}

// String comparition
int strncmp(const char *s1, const char *s2, int n) {
    while (n-- && *s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

// Function to parse rx_buf into separate arrays
uint8_t parseCSV(const volatile uint8_t* rx_received, uint16_t buf_size, volatile NumberArray* result){
	uint8_t array_count = 0;
	uint8_t digit_pos = 0;
	bool new_number = true;

	// Initialize first array
	result[array_count].length = 0;

	for (uint16_t i = 0; i < buf_size; i++) {
		uint8_t c = rx_received[i];

		// Skip leading whitespace (optional)
		if (c == ' ' && new_number) continue;

		// Handle digits
		if ((c >= '0' && c <= '9')||(c=='.')||(c=='-')) {
			if (digit_pos < MAX_DIGITS) {
				result[array_count].array[digit_pos++] = c;
				new_number = false;
			}
		}
		// Handle comma or newline
		else if (c == ',' || c == '\n' || c == '\r') {
			if (!new_number) {  // Only close array if we have a number
				// Add terminator
				result[array_count].array[digit_pos] = '\n';
				result[array_count].length = digit_pos + 1;
				array_count++;

				// Prepare next array
				if (array_count < MAX_ARRAYS) {
					digit_pos = 0;
					result[array_count].length = 0;
					new_number = true;
				} else {
					break;  // Reached maximum arrays
				}
			}

			if (c == '\n' || c == '\r') {
				// Ensure we create a new array even if no digits before newline
				if (array_count > 0 && result[array_count-1].array[result[array_count-1].length-1] != '\n') {
					result[array_count].array[0] = '\n';
					result[array_count].length = 1;
					array_count++;
				}
			}
		}
	}

	// Handle last number if buffer ends without comma/newline
	if (!new_number && array_count < MAX_ARRAYS) {
		result[array_count].array[digit_pos] = '\n';
		result[array_count].length = digit_pos + 1;
		array_count++;
	}

	return array_count;
}

// Turn texts array for USART communication to int and floats array for motor controlling
uint8_t arrayToArrayIntOrFloat(volatile NumberArray* numbersAsArray, uint8_t numberOfArrays, volatile Numeros* result){
	uint8_t array_count = 0;
	float numero;

	// Process the parsed arrays
	for (uint8_t i = 0; i < numberOfArrays; i++) {
		//USART2_SendString("Number ");
		//USART2_SendChar('0' + i);
		//USART2_SendString(": ");
		//USART2_SendString((const char*)numbersAsArray[i].array);

		numero = atof(numbersAsArray[i].array, numbersAsArray[i].length);

		if ((i < 2) || (i > 3)){
			result[i].i = (int)numero;
		} else {
			result[i].f = numero;
		}
		array_count++;
	}
	return array_count;
}

//Decide if negative o positive
void decideNegPos(volatile Numeros* numeros, uint8_t count) {
 	for (uint8_t i = 0; i < count; i++){
 		float value;

 		if (i==0){
 			value = (float)numeros[i].i;
 		} else {
 			value = numeros[i].f;
 		}

		if (value < 0) {
			GPIOC->ODR |= (1<<7);
			GPIOC->ODR &= ~(1<<6);
			USART2_SendString("NEGATIVE\r\n");
		} else {
			GPIOC->ODR |= (1<<6);
			GPIOC->ODR &= ~(1<<7);
			USART2_SendString("POSITIVE\r\n");
		}
		delay_ms(200);
		GPIOC->ODR &= ~(1<<7);
		GPIOC->ODR &= ~(1<<6);
		delay_ms(200);
	}
}

void USART2_SendSensorData(const volatile bool *sensorStates, uint8_t count, float floatValue, int intValue) {
    char buffer[64];
    char *ptr = buffer;

    // Write sensor states as comma-separated 0/1
    for (uint8_t i = 0; i < count; i++) {
        *ptr++ = sensorStates[i] ? '1' : '0';
        if (i < count - 1) *ptr++ = ',';
    }

    *ptr++ = ',';

    // ----- FLOAT TO STRING -----
    if (floatValue < 0) {
        *ptr++ = '-';
        floatValue = -floatValue;
    }

    int intPart = (int)floatValue;
    float frac = floatValue - intPart;

    // Integer part
    char temp[12];
    char *p = itoa(intPart, temp, 10);
    while (*p) *ptr++ = *p++;

    *ptr++ = '.';

    // Fractional part with 2 decimals
    frac *= 100;
    int fracInt = (int)(frac + 0.5f);
    if (fracInt < 10) *ptr++ = '0'; // leading zero if < 0.1
    p = itoa(fracInt, temp, 10);
    while (*p) *ptr++ = *p++;

    // ----- INTEGER VALUE -----
    *ptr++ = ',';
    p = itoa(intValue, temp, 10);
    while (*p) *ptr++ = *p++;

    *ptr++ = '\n';
    *ptr = '\0';

    USART2_SendString(buffer);
}

void SendObstacleStatusFloat(float detected) {
    char buffer[20];
    char *ptr = buffer;

    // Copy message text
    const char *msg = "Less than 50: ";
    while (*msg) *ptr++ = *msg++;

    // Convert float (0.0 or 1.0) to character
    *ptr++ = detected >= 0.5f ? '1' : '0';
    *ptr++ = '\n';
    *ptr = '\0';

    USART2_SendString(buffer);
}
