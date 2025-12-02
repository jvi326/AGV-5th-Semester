#include "ultrasonic.h"
#include "uart_commands.h"
#include "globals.h"
#include "ws2812.h"

#define TRIG1_PIN  18
#define ECHO1_PIN  19
#define OUT1_PIN   23

#define TRIG2_PIN  5
#define ECHO2_PIN  4
#define OUT2_PIN   22

// --- Sensor 3 (nuevo) ---
#define TRIG3_PIN  25
#define ECHO3_PIN  26
#define OUT3_PIN   3

#define BUZZER_PIN   21

float threshold_cm = 30.0;
bool new_threshold = 0;
bool ext_trig1 = 0;
bool ext_trig2 = 0;
bool ext_trig3 = 0;   // NUEVO

UltrasonicSensor sensor1; 
UltrasonicSensor sensor2;
UltrasonicSensor sensor3;  // NUEVO

uint8_t rx_buffer[RX_BUFFER_SIZE];

void PlayBuzzerPattern() {
  int shortToneFreq = 2800;
  int longToneFreq  = 2400;

  for (int i = 0; i < 4; i++) {
    tone(BUZZER_PIN, shortToneFreq, 90);
    delay(130);
  }

  delay(250);

  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, longToneFreq, 250);
    delay(350);
  }

  noTone(BUZZER_PIN);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    USART_Init();

    Ultrasonic_Init(&sensor1, TRIG1_PIN, ECHO1_PIN);
    Ultrasonic_Init(&sensor2, TRIG2_PIN, ECHO2_PIN);
    Ultrasonic_Init(&sensor3, TRIG3_PIN, ECHO3_PIN);  // NUEVO
    
    pinMode(OUT1_PIN, OUTPUT);
    pinMode(OUT2_PIN, OUTPUT);
    pinMode(OUT3_PIN, OUTPUT);  // NUEVO

    uint8_t startup_bytes[] = {0x01, 0x02, 0x03};
    uart_write_bytes(UART_NUM, (const char*)startup_bytes, sizeof(startup_bytes));
    Serial.println("Startup bytes sent on UART2");

    WS2812_Init();
    WS2812_SetTwoColors(39, 40, strip.Color(0,255,0), strip.Color(0,255,0));

    Serial.println("Ready!");
}

void loop() {
  // Leer distancias 
  float d1 = Ultrasonic_ReadDistance(&sensor1);
  float d2 = Ultrasonic_ReadDistance(&sensor2);
  float d3 = Ultrasonic_ReadDistance(&sensor3);  // NUEVO
  
  // ---------------- Sensor 1 ----------------
  if(d1 > 0 && d1 < threshold_cm){
    digitalWrite(OUT1_PIN, 1);
    ext_trig1 = 1;
  }

  // ---------------- Sensor 2 ----------------
  if(d2 > 0 && d2 < threshold_cm){
    digitalWrite(OUT2_PIN, 1);
    ext_trig2 = 1;
  } else {
    digitalWrite(OUT2_PIN, 0);
  }

  // ---------------- Sensor 3 (nuevo) ----------------
  if(d3 > 0 && d3 < threshold_cm){
    digitalWrite(OUT3_PIN, 1);
    ext_trig3 = 1;
  } else {
    digitalWrite(OUT3_PIN, 0);
  }

  // ---------------- Cualquier trigger ----------------
  if(ext_trig1 || ext_trig2 || ext_trig3){
    WS2812_SetTwoColors(39, 40, strip.Color(255,0,0), strip.Color(255,0,0));
    digitalWrite(OUT1_PIN, 1);
    PlayBuzzerPattern();

    if(ext_trig1){
      Serial.print("Extern trig_1: ");
      Serial.print(ext_trig1); Serial.print(",");
      Serial.println(d1);
      ext_trig1 = 0;
    }

    if(ext_trig2){
      Serial.print("Extern trig_2: ");
      Serial.print(ext_trig2); Serial.print(",");
      Serial.println(d2);
      ext_trig2 = 0;
    }

    if(ext_trig3){   // NUEVO
      Serial.print("Extern trig_3: ");
      Serial.print(ext_trig3); Serial.print(",");
      Serial.println(d3);
      ext_trig3 = 0;
    }

  } else {
    WS2812_SetTwoColors(39, 40, strip.Color(0,255,0), strip.Color(0,255,0));
    digitalWrite(OUT1_PIN, 0);
  }

  // Notificación cuando cambia threshold
  if(new_threshold == 1) {
    Serial.print("New Threshold_CM: ");
    Serial.println(threshold_cm);
    new_threshold = 0;
  }

  // Leer UART
  size_t rx_len = USART_Receive(rx_buffer, RX_BUFFER_SIZE);
  if (rx_len >= 2) {
      HandleCommand(rx_buffer, rx_len);
  }

  delay(100);
}
