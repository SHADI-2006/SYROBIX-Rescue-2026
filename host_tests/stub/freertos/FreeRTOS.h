#pragma once
#include <stdint.h>
typedef void* SemaphoreHandle_t; typedef int BaseType_t; typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
void vTaskDelay(TickType_t);
typedef void* TaskHandle_t;
#define pdPASS 1
BaseType_t xTaskCreatePinnedToCore(void(*)(void*), const char*, uint32_t, void*, unsigned, TaskHandle_t*, int);
TickType_t xTaskGetTickCount(); void vTaskDelayUntil(TickType_t*, TickType_t); void vTaskDelete(TaskHandle_t);
