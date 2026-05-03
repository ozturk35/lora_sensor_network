#pragma once

#include "esp_sleep.h"
#include <stdint.h>

void                      power_manager_sleep(int64_t active_start_us, uint32_t period_ms);
esp_sleep_wakeup_cause_t  power_manager_wakeup_cause(void);
