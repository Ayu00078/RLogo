#include "comm_scheduler.h"
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PowerRune_Events.h"
#include "device_registry.h"

extern esp_event_loop_handle_t pr_events_loop_handle;

static const char* TAG = "comm_sched";
#define HEARTBEAT_INTERVAL_MS 1000

static void send_heartbeat_to(uint8_t addr)
{
    RESPONSE_EVENT_DATA resp = {};
    resp.address = addr;
    resp.data_len = sizeof(RESPONSE_EVENT_DATA);
    esp_event_post_to(pr_events_loop_handle, PRC, RESPONSE_EVENT, &resp, sizeof(resp), 0);
}

static void comm_scheduler_task(void*)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));

        uint8_t mask = device_registry_armour_mask();
        for (uint8_t i = 0; i < 5; i++) {
            if (mask & (1 << i)) {
                send_heartbeat_to(i);
            }
        }
        if (device_registry_motor_online()) {
            send_heartbeat_to(MOTOR);
        }
    }
}

void comm_scheduler_start(void)
{
    xTaskCreate(comm_scheduler_task, "comm_sched", 3072, nullptr, 6, nullptr);
    ESP_LOGI(TAG, "comm scheduler started");
}