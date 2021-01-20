/*
 - Se dispondrá de un LCD que mostrará la temperatura actual de la estancia y la temperatura deseada
(establecida por el ángulo rotatorio). Esta información se actualizará 4 veces por segundo.
 - Si en la estancia no hay luz, el sistema se apagará por completo. Por lo tanto, el termostato sólo
funcionará cuando exista presencia de luz. Esta comprobación se realizará cada 5 segundos.

Aspectos a tener en cuenta: colas, semáforos y mutexes
*/


#include <Arduino.h>
#include <analogWrite.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Wire.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define btnEncendido D2
#define btnApagado D3
#define redLed D13
#define blueLed D12
#define rotation A0
#define luz D9
#define DHTTYPE DHT11
#define DHTPIN D4
#define LIGHTSENSORPIN A1

DHT dht(DHTPIN, DHTTYPE);
QueueHandle_t xMutex;

static volatile bool g_flash = false;
static volatile bool encendido = false;
int outputValue = 0;
float temperaturaGlobal = 0.0;
float temperaturaEscogida = 0.0;

void IRAM_ATTR on_handleInterrupt(){
  digitalWrite(D12, 1);
  g_flash = true;
  encendido = true;
}

void IRAM_ATTR off_handleInterrupt(){
  digitalWrite(D12, 0);
  digitalWrite(D13, 0);
  g_flash = false;
  encendido = false;
}

void encenderCaldera() {
  digitalWrite(D12, 0);
  digitalWrite(D13, 1);
}

void apagarCaldera() {
  digitalWrite(D12, 1);
  digitalWrite(D13, 0); 
}

/*
  Tarea para la lectura de la temperatura global y escogida y arrancado de termostato
*/
void vTask1 (void* param) {
  for(;;){
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      if (g_flash) {
        temperaturaGlobal= dht.readTemperature();
        if (isnan(temperaturaGlobal)) {
          Serial.println(F("Error de lectura del sensor DHT!"));
          return;
        }
        int val = analogRead(rotation);  
        outputValue = map(val, 0, 1023, 16, 20); //Transformamos los valores analógicos de entrada del knub en los que queramos para la temperatura
        temperaturaEscogida = outputValue;

        if (outputValue > temperaturaGlobal) {
          encenderCaldera();
        } else {
          apagarCaldera();
        }
      }
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 100);
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para la lectura de la luz ambiente y accionado y apagado en función de la 1intensidad
*/
void vTask2 (void* param) {
  for(;;){
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { 
      TickType_t xLastWakeTime = xTaskGetTickCount();
      float reading = analogRead(LIGHTSENSORPIN); 
      if (reading > 1000 && encendido) {
        g_flash = true;
      } else{
        g_flash = false;
        digitalWrite(D12, 0);
        digitalWrite(D13, 0);
        temperaturaEscogida = 0.0;
      }
    
      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 5000);
      
    }
  }
  vTaskDelete(NULL);
}

/*
  Tarea para la impresión de los parametros por el LCD
*/
void vTask3 (void* param) {
  for(;;){
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("SSD1306 allocation failed\n"));
      }
      
      display.clearDisplay();      
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(1,0);
      display.clearDisplay();
      display.print("Temperatura ambiente: ");
      display.println(temperaturaGlobal);
      display.print("Temperatura escogida: ");
      display.println(temperaturaEscogida);
      display.display();

      xSemaphoreGive(xMutex);
      vTaskDelayUntil(&xLastWakeTime, 4000);
    }
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  digitalWrite(D12, 0);
  digitalWrite(D13, 0);

  pinMode(btnEncendido, INPUT);
  pinMode(btnApagado, INPUT);
  attachInterrupt(digitalPinToInterrupt(btnEncendido), &on_handleInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnApagado), &off_handleInterrupt, FALLING);
  pinMode(redLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(rotation, INPUT);
  pinMode(LIGHTSENSORPIN, INPUT);

  dht.begin();
  temperaturaGlobal = dht.readTemperature();

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed\n"));
  }
  display.clearDisplay();

  xMutex = xSemaphoreCreateMutex(); 
  if (xMutex != NULL){
    xTaskCreatePinnedToCore(vTask1, "Tarea lectura temperatura", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTask2, "Tarea lectura luz", 1500, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTask3, "Tarea impresion", 1500, NULL, 1, NULL, 0);
  }
}

void loop() {
}