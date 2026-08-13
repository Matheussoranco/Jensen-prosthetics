// Compile-only stub — see Arduino.h in this directory.
#pragma once

#include <cstdint>

typedef int esp_err_t;

esp_err_t esp_task_wdt_init(uint32_t timeoutSeconds, bool panic);
esp_err_t esp_task_wdt_add(void* handle);
esp_err_t esp_task_wdt_reset();
