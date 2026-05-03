#include "power_manager.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "pwr";

void power_manager_sleep(int64_t active_start_us, uint32_t period_ms)
{
    int64_t elapsed_us  = esp_timer_get_time() - active_start_us;
    int64_t sleep_us    = (int64_t)period_ms * 1000LL - elapsed_us;
    if (sleep_us < 1000000LL) sleep_us = 1000000LL; /* minimum 1 s */

    ESP_LOGI(TAG, "active=%lld ms, sleeping %lld ms",
             elapsed_us / 1000, sleep_us / 1000);

    esp_sleep_enable_timer_wakeup((uint64_t)sleep_us);
    esp_deep_sleep_start();
    /* does not return */
}

esp_sleep_wakeup_cause_t power_manager_wakeup_cause(void)
{
    return esp_sleep_get_wakeup_cause();
}
