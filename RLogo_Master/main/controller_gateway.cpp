#include "controller_gateway.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "device_registry.h"
#include "hit_filter.h"
#include "LED_Strip.h"
#include "mech_engine.h"
#include "pr_cmd_events.h"
#include "pr_protocol_legacy.h"
#include "tcp_protocol.h"

extern esp_event_loop_handle_t pr_events_loop_handle;
extern LED_Strip* led_strip;

static const char *TAG = "controller";

static pr_controller_run_config_t s_run_config = {
    PR_CONTROLLER_ADDRESS,
    0, // red
    0, // big rune
    0, // no loop
    0, // clockwise
};

static bool s_debug_enabled = false;
static bool s_redled_mode = false;

bool controller_gateway_debug_enabled(void)
{
    return s_debug_enabled;
}

static bool valid_controller_address(const void *event_data)
{
    return event_data != nullptr &&
           ((const uint8_t *)event_data)[0] == PR_CONTROLLER_ADDRESS;
}

static void report_status(void)
{
    const uint8_t mask = device_registry_armour_mask();
    const uint8_t online = (uint8_t)__builtin_popcount((unsigned)mask);
    const uint8_t missing = PR_ARMOUR_COUNT - online;
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "STATUS armour=%u/%u missing=%u mask=0x%02X motor=%s ready=%s",
        online, PR_ARMOUR_COUNT, missing, mask,
        device_registry_motor_online() ? "online" : "offline",
        device_registry_all_ready() ? "yes" : "no");
}

static void on_hello(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *hello = (const pr_controller_hello_t *)event_data;
    ESP_LOGI(TAG, "Controller hello: protocol=%u", hello->protocol_version);
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                     "Controller hello accepted; protocol=%u",
                                     hello->protocol_version);
    report_status();
}

static void on_ping(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
}

static void on_config(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *config = (const pr_controller_run_config_t *)event_data;
    if (config->color > 1 || config->mode > 1 || config->loop > 1 ||
        config->dir > PR_RUN_DIR_CS) {
        ESP_LOGW(TAG, "Reject invalid Controller config: color=%u mode=%u loop=%u dir=%u",
                 config->color, config->mode, config->loop, config->dir);
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "CONFIG rejected: dir must be 0, 1 or 2; other values must be 0 or 1");
        return;
    }

    s_run_config = *config;
    ESP_LOGI(TAG, "CONFIG accepted: color=%u mode=%u loop=%u dir=%u",
             config->color, config->mode, config->loop, config->dir);
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "CONFIG accepted color=%u mode=%u loop=%u dir=%u",
        config->color, config->mode, config->loop, config->dir);
}

static void on_start(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    if (s_redled_mode) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "START rejected: REDLED mode is active; send REDLED OFF first");
        return;
    }
    pr_run_cmd_t cmd = {
        s_run_config.color,
        s_run_config.mode,
        s_run_config.loop,
        s_run_config.dir,
    };
    ESP_LOGI(TAG, "START forwarded: color=%u mode=%u loop=%u dir=%u",
             cmd.color, cmd.mode, cmd.loop, cmd.dir);
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                     "START accepted; forwarding to game engine");
    esp_event_post_to(pr_events_loop_handle, PRAPP, PRAPP_RUN_CMD_EVENT,
                      &cmd, sizeof(cmd), portMAX_DELAY);
}

static void on_stop(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    ESP_LOGI(TAG, "STOP forwarded");
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO, "STOP accepted");
    esp_event_post_to(pr_events_loop_handle, PRAPP, PRAPP_STOP_CMD_EVENT,
                      nullptr, 0, portMAX_DELAY);
}

static void on_unlock(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    if (s_redled_mode) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "UNLOCK rejected: REDLED mode is active; send REDLED OFF first");
        return;
    }
    if (s_run_config.dir == PR_RUN_DIR_CS) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "UNLOCK rejected: dir=cs keeps Motor stationary");
        return;
    }
    ESP_LOGI(TAG, "UNLOCK forwarded");
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO, "UNLOCK accepted");
    esp_event_post_to(pr_events_loop_handle, PRAPP, PRAPP_UNLK_CMD_EVENT,
                      nullptr, 0, portMAX_DELAY);
}

static void on_ota(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    ESP_LOGI(TAG, "OTA forwarded");
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO, "OTA accepted");
    esp_event_post_to(pr_events_loop_handle, PRAPP, PRAPP_OTA_CMD_EVENT,
                      nullptr, 0, portMAX_DELAY);
}

static void on_status(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    report_status();
}

static void on_debug(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *debug = (const pr_controller_debug_t *)event_data;
    s_debug_enabled = debug->enabled != 0;
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO, "DEBUG %s (Master candidate details %s)",
        s_debug_enabled ? "ON" : "OFF",
        s_debug_enabled ? "enabled" : "hidden");
}

static void on_redled_on(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    if (mech_engine_is_running()) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "REDLED rejected: game is running; STOP first");
        return;
    }

    s_redled_mode = true;
    if (led_strip != nullptr) {
        led_strip->set_color(50, 0, 0);
        led_strip->refresh();
    }
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "REDLED ON accepted; ballistic debug mode active");
}

static void on_redled_off(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;

    s_redled_mode = false;
    if (led_strip != nullptr) {
        led_strip->clear_pixels();
        led_strip->refresh();
    }
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "REDLED OFF accepted; Master light is off");
}

static void report_threshold(uint8_t address)
{
    bool enabled = false;
    uint16_t threshold = 0;
    if (!hit_filter_get(address, &enabled, &threshold)) return;
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "THRESHOLD armour=%u enabled=%u value=%u.%02u",
        address + 1U, enabled ? 1U : 0U, threshold / 100U, threshold % 100U);
}

static void on_threshold_set(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *request = (const pr_controller_threshold_set_t *)event_data;
    if (request->armour_id < 1 || request->armour_id > PR_ARMOUR_COUNT ||
        request->threshold_x100 > 25500U) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "THRESHOLD rejected: armour=1..%u peak=0.00..255.00",
                                         PR_ARMOUR_COUNT);
        return;
    }
    if (mech_engine_is_running()) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                         "THRESHOLD rejected: game is running; STOP first");
        return;
    }

    const uint8_t address = request->armour_id - 1U;
    const esp_err_t err = hit_filter_set(address, true, request->threshold_x100);
    if (err != ESP_OK) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "THRESHOLD armour=%u save failed: %s",
                                         request->armour_id, esp_err_to_name(err));
        return;
    }
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                     "THRESHOLD armour=%u enabled=1 value=%u.%02u saved; sync queued",
                                     request->armour_id,
                                     request->threshold_x100 / 100U,
                                     request->threshold_x100 % 100U);
    if (hit_filter_sync_armour(address) != ESP_OK) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "THRESHOLD armour=%u saved but sync could not be queued",
            request->armour_id);
    }
}

static void on_threshold_off(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *request = (const pr_controller_threshold_set_t *)event_data;
    if (request->armour_id < 1 || request->armour_id > PR_ARMOUR_COUNT) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "THRESHOLD OFF rejected: armour=1..%u",
                                         PR_ARMOUR_COUNT);
        return;
    }
    if (mech_engine_is_running()) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                         "THRESHOLD OFF rejected: game is running; STOP first");
        return;
    }

    const uint8_t address = request->armour_id - 1U;
    const esp_err_t err = hit_filter_set(address, false, 0);
    if (err != ESP_OK) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "THRESHOLD OFF armour=%u save failed: %s",
                                         request->armour_id, esp_err_to_name(err));
        return;
    }
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                     "THRESHOLD armour=%u enabled=0 saved; sync queued",
                                     request->armour_id);
    if (hit_filter_sync_armour(address) != ESP_OK) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "THRESHOLD armour=%u saved but sync could not be queued",
            request->armour_id);
    }
}

static void on_threshold_ack(void *, esp_event_base_t, int32_t, void *event_data)
{
    const auto *ack = (const PRA_HIT_THRESHOLD_ACK_EVENT_DATA *)event_data;
    if (!ack || ack->address >= PR_ARMOUR_COUNT) return;

    bool master_enabled = false;
    uint16_t master_threshold = 0;
    hit_filter_get(ack->address, &master_enabled, &master_threshold);
    const bool matches = ack->enabled == (master_enabled ? 1U : 0U) &&
                         ack->threshold_x100 == master_threshold;
    if (ack->status != 0 || !matches) {
        ESP_LOGW(TAG, "Armour %u threshold sync failed/status=%u value=%u.%02u",
                 ack->address + 1U, ack->status,
                 ack->threshold_x100 / 100U, ack->threshold_x100 % 100U);
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "THRESHOLD armour=%u sync failed status=%u value=%u.%02u",
            ack->address + 1U, ack->status,
            ack->threshold_x100 / 100U, ack->threshold_x100 % 100U);
    } else if (controller_gateway_debug_enabled()) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_DEBUG,
            "THRESHOLD armour=%u synced value=%u.%02u enabled=%u",
            ack->address + 1U, ack->threshold_x100 / 100U,
            ack->threshold_x100 % 100U, ack->enabled);
    }
}

static void on_threshold_show(void *, esp_event_base_t, int32_t, void *event_data)
{
    if (!valid_controller_address(event_data)) return;
    const auto *request = (const pr_controller_threshold_query_t *)event_data;
    if (request->armour_id == 0) {
        for (uint8_t address = 0; address < PR_ARMOUR_COUNT; ++address) {
            report_threshold(address);
        }
        return;
    }
    if (request->armour_id > PR_ARMOUR_COUNT) {
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_ERROR,
                                         "THRESHOLD SHOW rejected: armour=0..%u",
                                         PR_ARMOUR_COUNT);
        return;
    }
    report_threshold(request->armour_id - 1U);
}

void controller_gateway_start(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_HELLO, on_hello, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_PING, on_ping, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_CONFIG, on_config, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_START, on_start, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_STOP, on_stop, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_UNLOCK, on_unlock, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_OTA, on_ota, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_STATUS_REQUEST, on_status, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_DEBUG, on_debug, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_REDLED_ON, on_redled_on, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_REDLED_OFF, on_redled_off, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_THRESHOLD_SET, on_threshold_set, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_THRESHOLD_OFF, on_threshold_off, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRC, PRC_EVT_CONTROLLER_THRESHOLD_SHOW, on_threshold_show, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRA, PRA_HIT_THRESHOLD_ACK_EVENT, on_threshold_ack, nullptr));
    ESP_LOGI(TAG, "Controller command gateway ready");
}
