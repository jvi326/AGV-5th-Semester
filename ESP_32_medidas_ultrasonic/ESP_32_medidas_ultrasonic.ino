#include "ultrasonic.h"
#include "uart_commands.h"
#include "globals.h"

#define TRIG1_PIN  18
#define ECHO1_PIN  19
#define OUT1_PIN   23

#define TRIG2_PIN  5
#define ECHO2_PIN  4
#define OUT2_PIN   22

#define BUZZER_PIN   21


float threshold_cm = 30.0;
bool new_threshold = 0;
bool ext_trig1 = 0;
bool ext_trig2 = 0;

UltrasonicSensor sensor1; 
UltrasonicSensor sensor2;

uint8_t rx_buffer[RX_BUFFER_SIZE];

void PlayBuzzerPattern() {
  int shortToneFreq = 2800;  // tono agudo para "ti"
  int longToneFreq  = 2400;  // tono un poco más bajo para "tiii"

  // --- 5 sonidos rápidos "ti-ti-ti-ti-ti" ---
  for (int i = 0; i < 4; i++) {
    tone(BUZZER_PIN, shortToneFreq, 90);  // 90 ms de tono
    delay(130);                           // 40 ms silencio entre cada uno
  }

  delay(250); // pequeña pausa entre los dos bloques

  // --- 2 sonidos largos "tiii-tiii" ---
  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, longToneFreq, 250);  // tono más largo (~250 ms)
    delay(350);
  }

  noTone(BUZZER_PIN);
}

void setup() {
    Serial.begin(115200);   // monitor USB
    delay(500);

    USART_Init();           // inicializa UART2

    Ultrasonic_Init(&sensor1, TRIG1_PIN, ECHO1_PIN); 
    Ultrasonic_Init(&sensor2, TRIG2_PIN, ECHO2_PIN); 
    
    pinMode(OUT1_PIN, OUTPUT); 
    pinMode(OUT2_PIN, OUTPUT);

    // --- Enviar bytes de inicio ---
    uint8_t startup_bytes[] = {0x01, 0x02, 0x03};
    uart_write_bytes(UART_NUM, (const char*)startup_bytes, sizeof(startup_bytes));
    Serial.println("Startup bytes sent on UART2");

    Serial.println("Ready!");
}


void loop() {
  // Leer distancias 
  float d1 = Ultrasonic_ReadDistance(&sensor1); 
  float d2 = Ultrasonic_ReadDistance(&sensor2); 
  
  if(d1>0 && d1<threshold_cm){
    digitalWrite(OUT1_PIN, 1);
    ext_trig1 = 1; 
  } else {
    digitalWrite(OUT1_PIN, 0);
  }
  if(d2>0 && d2<threshold_cm){
    digitalWrite(OUT2_PIN, 1);
    ext_trig2 = 1; 
  } else {
    digitalWrite(OUT2_PIN, 0);
  }

  if(ext_trig1 == 1 || ext_trig2 == 1){
    PlayBuzzerPattern(); //Encender Buzzer
    if(ext_trig1 == 1){
      Serial.print("Extern trig_1: ");
      Serial.print(ext_trig1);
      Serial.print(",");
      Serial.println(d1);
      ext_trig1 = 0;
    }
    if(ext_trig2 == 1){
      Serial.print("Extern trig_2: ");
      Serial.print(ext_trig2);
      Serial.print(",");
      Serial.println(d2);
      ext_trig2 = 0;
    }
  }

  if(new_threshold == 1) {
    Serial.print("New Threshold_CM: ");
    Serial.println(threshold_cm);
    new_threshold = 0;
  }

  // leer UART
  size_t rx_len = USART_Receive(rx_buffer, RX_BUFFER_SIZE);
  if (rx_len >= 2) { // al menos tipo + sub-tipo
      HandleCommand(rx_buffer, rx_len);
  }

  delay(100);
}
