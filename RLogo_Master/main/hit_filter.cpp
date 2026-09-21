#include "hit_filter.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "PowerRune_Events.h"
#include "pr_types.h"

extern esp_event_loop_handle_t pr_events_loop_handle;

static const char *TAG = "hit_filter";
static constexpr uint8_t kArmourCount = PR_ARMOUR_COUNT;

static bool s_enabled[kArmourCount] = {};
static uint16_t s_threshold_x100[kArmourCount] = {};
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool valid_address(uint8_t address)
{
    return address < kArmourCount;
}

static void make_key(char *key, size_t key_size, const char *prefix, uint8_t address)
{
    snprintf(key, key_size, "%s%u", prefix, (unsigned)(address + 1U));
}

void hit_filter_init(void)
{
    nvs_handle_t nvs = 0;
    const esp_err_t open_err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (open_err != ESP_OK) {
        ESP_LOGI(TAG, "no saved second thresholds; all disabled");
        return;
    }

    for (uint8_t address = 0; address < kArmourCount; ++address) {
        char enabled_key[8] = {};
        char threshold_key[8] = {};
        make_key(enabled_key, sizeof(enabled_key), "hf_e", address);
        make_key(threshold_key, sizeof(threshold_key), "hf_t", address);

        uint8_t enabled = 0;
        uint16_t threshold = 0;
        const esp_err_t enabled_err = nvs_get_u8(nvs, enabled_key, &enabled);
        const esp_err_t threshold_err = nvs_get_u16(nvs, threshold_key, &threshold);
        if (enabled_err == ESP_OK && threshold_err == ESP_OK && threshold <= 25500U) {
            s_enabled[address] = enabled != 0;
            s_threshold_x100[address] = threshold;
        }
    }
    nvs_close(nvs);

    for (uint8_t address = 0; address < kArmourCount; ++address) {
        ESP_LOGI(TAG, "armour=%u second_threshold=%s %.2f",
                 address + 1U, s_enabled[address] ? "ON" : "OFF",
                 s_threshold_x100[address] / 100.0f);
    }
}

bool hit_filter_get(uint8_t address, bool *enabled, uint16_t *threshold_x100)
{
    if (!valid_address(address)) return false;
    portENTER_CRITICAL(&s_mux);
    if (enabled) *enabled = s_enabled[address];
    if (threshold_x100) *threshold_x100 = s_threshold_x100[address];
    portEXIT_CRITICAL(&s_mux);
    return true;
}

esp_err_t hit_filter_set(uint8_t address, bool enabled, uint16_t threshold_x100)
{
    if (!valid_address(address) || threshold_x100 > 25500U) {
        return ESP_ERR_INVALID_ARG;
    }

    bool old_enabled = false;
    uint16_t old_threshold = 0;
    hit_filter_get(address, &old_enabled, &old_threshold);

    portENTER_CRITICAL(&s_mux);
    s_enabled[address] = enabled;
    s_threshold_x100[address] = threshold_x100;
    portEXIT_CRITICAL(&s_mux);

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        char enabled_key[8] = {};
        char threshold_key[8] = {};
        make_key(enabled_key, sizeof(enabled_key), "hf_e", address);
        make_key(threshold_key, sizeof(threshold_key), "hf_t", address);
        err = nvs_set_u8(nvs, enabled_key, enabled ? 1U : 0U);
        if (err == ESP_OK) err = nvs_set_u16(nvs, threshold_key, threshold_x100);
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }

    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_mux);
        s_enabled[address] = old_enabled;
        s_threshold_x100[address] = old_threshold;
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGE(TAG, "save armour=%u threshold failed: %s",
                 address + 1U, esp_err_to_name(err));
    }
    return err;
}

esp_err_t hit_filter_sync_armour(uint8_t address)
{
    if (!valid_address(address) || pr_events_loop_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    bool enabled = false;
    uint16_t threshold_x100 = 0;
    hit_filter_get(address, &enabled, &threshold_x100);

    PRA_HIT_THRESHOLD_CONFIG_EVENT_DATA config = {};
    config.address = address;
    config.data_len = sizeof(config);
    config.enabled = enabled ? 1U : 0U;
    config.threshold_x100 = threshold_x100;
    return esp_event_post_to(pr_events_loop_handle, PRA,
                             PRA_HIT_THRESHOLD_CONFIG_EVENT,
                             &config, sizeof(config), 0);
}
