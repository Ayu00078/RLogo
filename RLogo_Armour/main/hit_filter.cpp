#include "hit_filter.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "pr_types.h"

static const char *TAG = "hit_filter";

static bool s_enabled = false;
static uint16_t s_threshold_x100 = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void hit_filter_init(void)
{
    nvs_handle_t nvs = 0;
    const esp_err_t err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        uint8_t enabled = 0;
        uint16_t threshold = 0;
        if (nvs_get_u8(nvs, "hf_e", &enabled) == ESP_OK &&
            nvs_get_u16(nvs, "hf_t", &threshold) == ESP_OK &&
            enabled <= 1U && threshold <= 25500U) {
            s_enabled = enabled != 0;
            s_threshold_x100 = threshold;
        }
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "local second threshold=%s %.2f",
             s_enabled ? "ON" : "OFF", s_threshold_x100 / 100.0f);
}

bool hit_filter_get(bool *enabled, uint16_t *threshold_x100)
{
    portENTER_CRITICAL(&s_mux);
    if (enabled) *enabled = s_enabled;
    if (threshold_x100) *threshold_x100 = s_threshold_x100;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

esp_err_t hit_filter_set(bool enabled, uint16_t threshold_x100)
{
    if (threshold_x100 > 25500U) return ESP_ERR_INVALID_ARG;

    bool old_enabled = false;
    uint16_t old_threshold = 0;
    hit_filter_get(&old_enabled, &old_threshold);

    portENTER_CRITICAL(&s_mux);
    s_enabled = enabled;
    s_threshold_x100 = threshold_x100;
    portEXIT_CRITICAL(&s_mux);

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "hf_e", enabled ? 1U : 0U);
        if (err == ESP_OK) err = nvs_set_u16(nvs, "hf_t", threshold_x100);
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }

    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_mux);
        s_enabled = old_enabled;
        s_threshold_x100 = old_threshold;
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGE(TAG, "save local threshold failed: %s", esp_err_to_name(err));
    }
    return err;
}
