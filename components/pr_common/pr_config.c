#include "pr_config_schema.h"

#include <string.h>

#include "nvs.h"

void pr_config_defaults(pr_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->version = PR_CONFIG_VERSION;
    config->groups = PR_GAME_GROUPS;
    config->first_hit_timeout_ms = PR_GAME_FIRST_HIT_TIMEOUT_MS;
    config->second_hit_timeout_ms = PR_GAME_SECOND_HIT_TIMEOUT_MS;
    config->calibration_ms = PR_GAME_CALIBRATION_MS;
    config->sensor_baseline_samples = PR_SENSOR_BASELINE_SAMPLES;
    config->sensor_threshold = PR_SENSOR_HIT_THRESHOLD;
    config->sensor_hit_window_samples = PR_SENSOR_HIT_WINDOW_SAMPLES;
    config->small_rpm = PR_SMALL_RPM;
    config->sine_amplitude = 1000;
    config->sine_omega_milli = 1000;
    config->sine_offset = 0;
    const uint8_t mapping[PR_SENSOR_ADC_COUNT] = {6, 7, 8, 9, 10, 1, 2, 3, 4, 5};
    memcpy(config->adc_to_ring, mapping, sizeof(mapping));
}

bool pr_config_validate(const pr_config_t *config)
{
    if (config == NULL || config->version != PR_CONFIG_VERSION ||
        config->groups != PR_GAME_GROUPS || config->first_hit_timeout_ms < 100U ||
        config->first_hit_timeout_ms > 10000U ||
        config->second_hit_timeout_ms < 100U ||
        config->second_hit_timeout_ms > 5000U || config->calibration_ms < 1000U ||
        config->calibration_ms > 60000U || config->sensor_baseline_samples == 0U ||
        config->sensor_baseline_samples > 100U || config->sensor_threshold == 0U ||
        config->sensor_threshold > 255U || config->sensor_hit_window_samples == 0U ||
        config->sensor_hit_window_samples > 200U || config->small_rpm != PR_SMALL_RPM) {
        return false;
    }
    for (size_t i = 0; i < PR_SENSOR_ADC_COUNT; ++i) {
        if (config->adc_to_ring[i] == 0U || config->adc_to_ring[i] > 10U) {
            return false;
        }
    }
    return true;
}

bool pr_config_load(pr_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    pr_config_defaults(config);

    nvs_handle_t handle;
    if (nvs_open(PR_CONFIG_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    pr_config_t stored;
    size_t length = sizeof(stored);
    const esp_err_t result = nvs_get_blob(handle, PR_CONFIG_KEY, &stored, &length);
    nvs_close(handle);
    if (result != ESP_OK || length != sizeof(stored) || !pr_config_validate(&stored)) {
        return false;
    }
    *config = stored;
    return true;
}

bool pr_config_save(const pr_config_t *config)
{
    if (!pr_config_validate(config)) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(PR_CONFIG_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const esp_err_t result = nvs_set_blob(handle, PR_CONFIG_KEY, config, sizeof(*config));
    const esp_err_t commit_result = result == ESP_OK ? nvs_commit(handle) : result;
    nvs_close(handle);
    return commit_result == ESP_OK;
}
