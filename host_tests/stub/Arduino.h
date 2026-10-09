#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#include "esp_attr.h"
#define F(x) x
typedef uint8_t byte;
unsigned long millis(); unsigned long micros();
void delay(uint32_t); void delayMicroseconds(uint32_t);
void pinMode(uint8_t,uint8_t); void digitalWrite(uint8_t,uint8_t); int digitalRead(uint8_t);
uint16_t analogRead(uint8_t); void analogReadResolution(uint8_t);
typedef enum { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db } adc_attenuation_t;
void analogSetPinAttenuation(uint8_t, adc_attenuation_t);
unsigned long pulseIn(uint8_t,uint8_t,unsigned long);
#include "freertos/FreeRTOS.h"
#define CHANGE 3
#define RISING 1
#define FALLING 2
typedef struct { int x; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) (void)(m)
#define portEXIT_CRITICAL(m) (void)(m)
#define portENTER_CRITICAL_ISR(m) (void)(m)
#define portEXIT_CRITICAL_ISR(m) (void)(m)
int digitalPinToInterrupt(int p);
void attachInterrupt(int, void(*)(void), int);
void attachInterruptArg(int, void(*)(void*), void*, int);
bool ledcSetup(uint8_t ch, uint32_t f, uint8_t res);
void ledcAttachPin(uint8_t pin, uint8_t ch);
bool ledcAttachChannel(uint8_t pin, uint32_t f, uint8_t res, uint8_t ch);
void ledcWrite(uint8_t chOrPin, uint32_t duty);
double ledcWriteTone(uint8_t chOrPin, uint32_t f);
struct HardwareSerialStub { void begin(unsigned long){} int printf(const char*, ...){return 0;} void println(const char* = ""){} void print(char){} void print(const char*){} operator bool() const {return true;} };
extern HardwareSerialStub Serial;
inline long map(long x, long a, long b, long c, long d){ return (x - a) * (d - c) / (b - a) + c; }
