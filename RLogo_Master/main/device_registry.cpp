#include "device_registry.h"

#include "esp_log.h"
#include "esp_event.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "PowerRune_Events.h"
#include "pr_cmd_events.h"
#include "pr_types.h"
#include "LED.h"
#include "LED_Strip.h"
#include "firmware.h"
#include "tcp_protocol.h"
#include "hit_filter.h"

extern esp_event_loop_handle_t pr_events_loop_handle;
extern LED* led;
extern LED_Strip* led_strip;

static const char* TAG = "devreg";

static constexpr uint8_t ARMOUR_COUNT = PR_ARMOUR_COUNT;
static constexpr uint8_t ARMOUR_MASK_ALL = (1U << ARMOUR_COUNT) - 1U;

// ====================== 内部状态 ======================

static uint8_t s_armour_mask = 0;            // 已上线 armour 位图（位 i = armour 地址 i 上线）
static bool    s_motor_online = false;
static bool    s_ready = false;              // 电机在线即可进入 READY，Armour 数量可为 0~5
static volatile uint8_t s_armour_acked_mask = 0; // 已 ACK 参数下发的 armour 位图
static volatile uint8_t s_provision_target_mask = 0; // 本轮需要等待 ACK 的 Armour

static TaskHandle_t s_provision_task = nullptr;
static TaskHandle_t s_status_task = nullptr;
static volatile bool s_provision_done = false;
static TaskHandle_t s_link_monitor_task = nullptr;
static TickType_t s_last_armour_ping[ARMOUR_COUNT] = {};
static TickType_t s_last_motor_ping = 0;
static bool s_link_seen[6] = {};
static bool s_safe_stop_latched = false;

static constexpr TickType_t LINK_TIMEOUT_TICKS = pdMS_TO_TICKS(3000);

static uint8_t armour_online_count(uint8_t mask)
{
    return static_cast<uint8_t>(__builtin_popcount(static_cast<unsigned>(mask & ARMOUR_MASK_ALL)));
}

static void start_provision_task(void);

uint8_t device_registry_armour_mask(void) { return s_armour_mask; }
bool device_registry_motor_online(void) { return s_motor_online; }
bool device_registry_all_ready(void) { return s_ready; }

// ====================== 默认传感器参数（在这里改） ======================
//
// 想调参就改这里的数，然后只重烧 RLogo 即可，5 块装甲板不用动。
// 这些值会在 5 装甲+电机全部连上后自动广播给所有装甲板。

static PRA_SET_SENSOR_PARAM_DATA build_default_sensor_params(void)
{
    PRA_SET_SENSOR_PARAM_DATA d = {};
    d.address             = 0xFF;            // 广播给所有 armour
    d.data_len            = sizeof(d);
    d.ema_alpha_big       = 0.93f;           // 大符 alpha（需跟上旋转重力漂移）
    d.ema_alpha_small     = 0.97f;          // 小符 alpha（高灵敏度）
    d.hit_threshold_small = 28.9f;           // 小符：静止干净，阈值低些更灵敏
    d.hit_threshold_big   = 39.5f;           // 大符：电机震动大，阈值高些防误触
    d.refractory_ms       = 400;             // 防抖时间（ms）：持续安静≥此值才重新武装
    d.channel_count       = 10;              // 通道数
    return d;
}

// ====================== 状态灯任务 ======================

static void online_armour_blink_task(void*)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (s_ready) continue;

        uint8_t online_mask = s_armour_mask;
        ESP_LOGW(TAG, "INCOMPLETE: blink online armours, mask=0x%02X motor=%d",
                 online_mask, s_motor_online);
         for (uint8_t address = 0; address < ARMOUR_COUNT && !s_ready; ++address) {
            if ((online_mask & (1U << address)) == 0) continue;

            PRA_COMPLETE_EVENT_DATA event = {};
            event.address = address;
            event.data_len = sizeof(event);
            esp_event_post_to(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT,
                              &event, sizeof(event), portMAX_DELAY);
            ESP_LOGI(TAG, "IDENTIFY: blink online armour %u", address + 1);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static void status_indicator_task(void*)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        const uint8_t online_mask = s_armour_mask & ARMOUR_MASK_ALL;
        const uint8_t online_count = armour_online_count(online_mask);
        const uint8_t missing_count = ARMOUR_COUNT - online_count;
        if (missing_count > 0) {
            ESP_LOGW(TAG, "ARMOUR WARNING: %u/%u online, %u armour(s) missing (mask=0x%02X)",
                     online_count, ARMOUR_COUNT, missing_count, online_mask);
            TcpProtocol::controller_send_log(
                PR_CONTROLLER_LOG_WARN,
                "ARMOUR WARNING online=%u/%u missing=%u mask=0x%02X",
                online_count, ARMOUR_COUNT, missing_count, online_mask);
        }

        if (!s_ready) {
            uint8_t missing_ids[6] = {};
            uint8_t missing_count = 0;
            for (uint8_t id = 1; id <= 6; ++id) {
                bool online = (id <= ARMOUR_COUNT) ? ((s_armour_mask & (1U << (id - 1))) != 0) : s_motor_online;
                if (!online) missing_ids[missing_count++] = id;
            }

            for (uint8_t index = 0; index < missing_count && !s_ready; ++index) {
                uint8_t id = missing_ids[index];
                ESP_LOGW(TAG, "OFFLINE: device %u missing (%s)", id, id == 6 ? "motor" : "armour");
                for (uint8_t n = 0; n < id && !s_ready; ++n) {
                    led_strip->set_color(0, 0, 50);
                    led_strip->refresh();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    led_strip->clear_pixels();
                    led_strip->refresh();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                if (!s_ready) {
                    bool last_missing = (index + 1 == missing_count);
                    if (last_missing) {
                        led_strip->set_color(50, 0, 0);
                    } else {
                        led_strip->clear_pixels();
                    }
                    led_strip->refresh();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }
    }
}

// ====================== Provision 任务（闪烁绿 + 下发 + 等 ACK） ======================

static void provision_task(void*)
{
    PRA_SET_SENSOR_PARAM_DATA data = build_default_sensor_params();

    // 先点亮绿色（作为闪烁的第一个亮帧）并立刻广播一次参数
    led_strip->set_color(0, 50, 0);
    led_strip->refresh();
    bool blink_on = true;

    ESP_LOGI(TAG, "PROVISION START: broadcast sensor params, waiting for current Armour ACKs (target_mask=0x%02X, count=%u; alpha_big=%.4f alpha_small=%.4f thr_small=%.1f thr_big=%.1f refract=%u ch=%u)",
             s_provision_target_mask, armour_online_count(s_provision_target_mask),
             data.ema_alpha_big, data.ema_alpha_small, data.hit_threshold_small,
             data.hit_threshold_big, (unsigned)data.refractory_ms, (unsigned)data.channel_count);
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "SENSOR provision start target_mask=0x%02X count=%u threshold_small=%.1f threshold_big=%.1f",
        s_provision_target_mask, armour_online_count(s_provision_target_mask),
        data.hit_threshold_small, data.hit_threshold_big);
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_SET_SENSOR_PARAM_EVENT,
                      &data, sizeof(data), portMAX_DELAY);

    const TickType_t blink_period = pdMS_TO_TICKS(300);
    const TickType_t retry_period = pdMS_TO_TICKS(2000);
    TickType_t last_push = xTaskGetTickCount();

    while (s_ready && !s_provision_done) {
        // 只等待本轮已经上线的 Armour；0 块 Armour 时无需等待 ACK。
        const uint8_t target_mask = s_provision_target_mask & ARMOUR_MASK_ALL;
        if ((s_armour_acked_mask & target_mask) == target_mask) {
            s_provision_done = true;
            break;
        }

        // 闪烁：每 blink_period 翻转一次
        vTaskDelay(blink_period);
        blink_on = !blink_on;
        if (blink_on) {
            led_strip->set_color(0, 50, 0);
        } else {
            led_strip->clear_pixels();
        }
        led_strip->refresh();

        // 2 秒未收齐则重发广播（TCP 一般不会丢，纯粹为了 armour 启动晚或临时掉线的兜底）
        if ((xTaskGetTickCount() - last_push) >= retry_period) {
            ESP_LOGW(TAG, "PROVISION: retry (armour_mask=0x%02X acked=0x%02X)",
                     s_armour_mask, s_armour_acked_mask);
            esp_event_post_to(pr_events_loop_handle, PRA, PRA_SET_SENSOR_PARAM_EVENT,
                              &data, sizeof(data), portMAX_DELAY);
            last_push = xTaskGetTickCount();
        }
    }

    if (s_ready && s_provision_done) {
        // 完成：绿色常亮
        led_strip->set_color(0, 50, 0);
        led_strip->refresh();
        ESP_LOGI(TAG, "PROVISION DONE: %u Armour(s) acked sensor params, LED = solid GREEN",
                 armour_online_count(s_provision_target_mask));
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_INFO,
            "SENSOR provision done acked=%u LED=GREEN",
            armour_online_count(s_provision_target_mask));
    } else {
        ESP_LOGW(TAG, "PROVISION ABORTED: device set is no longer ready");
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                         "SENSOR provision aborted: device set changed");
    }

    s_provision_task = nullptr;
    vTaskDelete(nullptr);
}

static void start_provision_task(void)
{
    if (s_provision_task == nullptr && s_ready && !s_provision_done) {
        xTaskCreate(provision_task, "provision", 4096, nullptr, 5, &s_provision_task);
    }
}

// ====================== 事件回调 ======================

static void try_ready(void)
{
    if (!s_motor_online) return;

    if (!s_ready) {
        s_ready = true;
        s_provision_target_mask = s_armour_mask & ARMOUR_MASK_ALL;
        s_armour_acked_mask = 0;
        s_provision_done = false;
        const uint8_t online_count = armour_online_count(s_provision_target_mask);
        const uint8_t missing_count = ARMOUR_COUNT - online_count;
        if (missing_count > 0) {
            ESP_LOGW(TAG, "READY: motor online, running with %u/%u armours; %u armour(s) missing",
                     online_count, ARMOUR_COUNT, missing_count);
            TcpProtocol::controller_send_log(
                PR_CONTROLLER_LOG_WARN,
                "READY motor online; armours=%u/%u missing=%u",
                online_count, ARMOUR_COUNT, missing_count);
        } else {
            ESP_LOGI(TAG, "READY: all %u armours + motor online", ARMOUR_COUNT);
            TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                             "READY all armours + motor online");
        }
        led->set_mode(LED_MODE_ON, 1);

        start_provision_task();
    } else if (s_provision_task == nullptr && !s_provision_done) {
        start_provision_task();
    }
}

static void safe_stop_for_link_loss(void)
{
    if (s_safe_stop_latched) {
        return;
    }
    s_safe_stop_latched = true;
    s_ready = false;
    s_armour_mask = 0;
    s_motor_online = false;
    s_armour_acked_mask = 0;
    s_provision_target_mask = 0;
    s_provision_done = false;
    memset(s_link_seen, 0, sizeof(s_link_seen));

    ESP_LOGE(TAG, "LINK LOSS: forcing game and all devices to STOP");
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                     "LINK LOSS: forcing game and devices to STOP");
    // Stop the local game worker immediately, then preserve the original
    // wire STOP events for every reachable Armour/Motor.
    esp_event_post_to(pr_events_loop_handle, PRAPP, PRAPP_STOP_CMD_EVENT,
                      nullptr, 0, portMAX_DELAY);

    PRA_STOP_EVENT_DATA armour_stop = {};
    armour_stop.address = 0xFF;
    armour_stop.data_len = sizeof(armour_stop);
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT,
                      &armour_stop, sizeof(armour_stop), portMAX_DELAY);

    PRM_STOP_EVENT_DATA motor_stop = {};
    motor_stop.address = MOTOR;
    motor_stop.data_len = sizeof(motor_stop);
    esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT,
                      &motor_stop, sizeof(motor_stop), portMAX_DELAY);
}

static void link_monitor_task(void*)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!s_ready) {
            continue;
        }

        const TickType_t now = xTaskGetTickCount();
        for (uint8_t address = 0; address < ARMOUR_COUNT; ++address) {
            if (s_link_seen[address] &&
                (now - s_last_armour_ping[address]) > LINK_TIMEOUT_TICKS) {
                ESP_LOGE(TAG, "Armour %u heartbeat timeout", address);
                TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                                 "Armour %u heartbeat timeout", address + 1);
                safe_stop_for_link_loss();
                break;
            }
        }
        if (s_ready && s_link_seen[MOTOR] &&
            (now - s_last_motor_ping) > LINK_TIMEOUT_TICKS) {
            ESP_LOGE(TAG, "Motor heartbeat timeout");
            TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                             "Motor heartbeat timeout");
            safe_stop_for_link_loss();
        }
    }
}

static void on_pra_ping(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_PING_EVENT_DATA*)event_data;
    if (d->address < ARMOUR_COUNT) {
        const uint8_t bit = static_cast<uint8_t>(1U << d->address);
        const bool newly_online = (s_armour_mask & bit) == 0;
        s_last_armour_ping[d->address] = xTaskGetTickCount();
        s_link_seen[d->address] = true;
        s_armour_mask |= bit;
        s_safe_stop_latched = false;
        if (newly_online && s_ready) {
            s_provision_target_mask |= bit;
            s_armour_acked_mask &= static_cast<uint8_t>(~bit);
            s_provision_done = false;
            ESP_LOGI(TAG, "ARMOUR %u joined after READY; provisioning it (online=%u/%u)",
                     d->address + 1, armour_online_count(s_armour_mask), ARMOUR_COUNT);
            TcpProtocol::controller_send_log(
                PR_CONTROLLER_LOG_INFO,
                "ARMOUR %u connected after READY; provisioning it (online=%u/%u)",
                d->address + 1, armour_online_count(s_armour_mask), ARMOUR_COUNT);
        }
        try_ready();
        if (newly_online) {
            const esp_err_t threshold_err = hit_filter_sync_armour(d->address);
            if (threshold_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to queue threshold sync for Armour %u: %s",
                         d->address + 1U, esp_err_to_name(threshold_err));
            }
        }
    }
}

static void on_prm_ping(void*, esp_event_base_t, int32_t, void*)
{
    s_last_motor_ping = xTaskGetTickCount();
    s_link_seen[MOTOR] = true;
    s_motor_online = true;
    s_safe_stop_latched = false;
    try_ready();
}

static void on_pra_sensor_param_ack(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_SENSOR_PARAM_ACK_DATA*)event_data;
    if (d->address <= 4) {
        s_armour_acked_mask |= (1 << d->address);
        ESP_LOGI(TAG, "ACK: armour %d ack sensor params (status=%d), acked_mask=0x%02X/target=0x%02X",
                 d->address, (int)d->status, s_armour_acked_mask, s_provision_target_mask);
    } else {
        ESP_LOGW(TAG, "ACK: ignore invalid address=%d", d->address);
    }
}

// ====================== 对外接口 ======================

void device_registry_init(void)
{
    s_armour_mask = 0;
    s_motor_online = false;
    s_ready = false;
    s_armour_acked_mask = 0;
    s_provision_target_mask = 0;
    s_provision_done = false;
    memset(s_last_armour_ping, 0, sizeof(s_last_armour_ping));
    s_last_motor_ping = 0;
    memset(s_link_seen, 0, sizeof(s_link_seen));
    s_safe_stop_latched = false;
}

void device_registry_start(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_PING_EVENT, on_pra_ping, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_PING_EVENT, on_prm_ping, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_SENSOR_PARAM_ACK_EVENT, on_pra_sensor_param_ack, nullptr));

    if (s_status_task == nullptr) {
        xTaskCreate(status_indicator_task, "status_led", 3072, nullptr, 4, &s_status_task);
    }
    if (s_link_monitor_task == nullptr) {
        xTaskCreate(link_monitor_task, "link_monitor", 3072, nullptr, 7,
                    &s_link_monitor_task);
    }
}
