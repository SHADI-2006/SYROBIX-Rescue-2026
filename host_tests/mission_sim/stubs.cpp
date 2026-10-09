#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <freertos/semphr.h>
#include "Adafruit_PWMServoDriver.h"
FakePca g_chip;
static uint8_t g_txAddr = 0, g_rxAddr = 0; static int g_txCount = 0; static bool g_rxPending = false;
uint32_t g_ms = 1000; int g_button = HIGH;
unsigned long millis(){return g_ms;} unsigned long micros(){return g_ms*1000UL;}
void delay(uint32_t m){g_ms+=m;} void delayMicroseconds(uint32_t){}
void pinMode(uint8_t,uint8_t){} void digitalWrite(uint8_t,uint8_t){}
int digitalRead(uint8_t p){ return p==Pins::START_BUTTON ? g_button : HIGH; }
uint16_t analogRead(uint8_t){return 0;} void analogReadResolution(uint8_t){}
void analogSetPinAttenuation(uint8_t, adc_attenuation_t){}
unsigned long pulseIn(uint8_t,uint8_t,unsigned long){return 0;}
int digitalPinToInterrupt(int p){return p;} void attachInterrupt(int, void(*)(void), int){}
void attachInterruptArg(int, void(*)(void*), void*, int){}
bool ledcSetup(uint8_t, uint32_t, uint8_t){return true;} void ledcAttachPin(uint8_t, uint8_t){}
bool ledcAttachChannel(uint8_t, uint32_t, uint8_t, uint8_t){return true;}
void ledcWrite(uint8_t, uint32_t){} double ledcWriteTone(uint8_t, uint32_t){return 0;}
HardwareSerialStub Serial; TwoWire Wire;
bool TwoWire::begin(int,int,uint32_t){return true;}
void TwoWire::beginTransmission(uint8_t a){ g_txAddr = a; g_txCount = 0; }
size_t TwoWire::write(uint8_t b){ if (g_txAddr == 0x40 && g_txCount++ == 0) g_chip.regPtr = b; return 1; }
uint8_t TwoWire::endTransmission(bool){ return (g_txAddr == 0x40 && g_chip.present) ? 0 : 2; }
uint8_t TwoWire::requestFrom(uint8_t a,uint8_t n){ if (a == 0x40 && g_chip.present && n == 1) { g_rxAddr = a; g_rxPending = true; return 1; } return 0; }
int TwoWire::read(){ if (!g_rxPending) return -1; g_rxPending = false;
  return g_chip.regPtr == 0x00 ? g_chip.mode1 : g_chip.regPtr == 0xFE ? g_chip.prescale : 0; }
int TwoWire::available(){ return g_rxPending ? 1 : 0; }
void vTaskDelay(TickType_t t){g_ms+=t;}
static int mtx; SemaphoreHandle_t xSemaphoreCreateMutex(){return &mtx;}
BaseType_t xSemaphoreTake(SemaphoreHandle_t,TickType_t){return pdTRUE;} BaseType_t xSemaphoreGive(SemaphoreHandle_t){return pdTRUE;}
BaseType_t xTaskCreatePinnedToCore(void(*)(void*), const char*, uint32_t, void*, unsigned, TaskHandle_t*, int){return pdPASS;}
TickType_t xTaskGetTickCount(){return g_ms;} void vTaskDelayUntil(TickType_t*, TickType_t){} void vTaskDelete(TaskHandle_t){}
