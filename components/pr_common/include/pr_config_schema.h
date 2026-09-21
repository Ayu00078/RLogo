#ifndef PR_CONFIG_SCHEMA_H
#define PR_CONFIG_SCHEMA_H

#include <stdbool.h>
#include <stdint.h>

#include "pr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PR_CONFIG_NAMESPACE "PowerRune"
#define PR_CONFIG_KEY "pr_config"
#define PR_CONFIG_VERSION 1U

typedef struct __attribute__((packed)) {
    uint16_t version;
    uint8_t groups;
    uint8_t reserved;
    uint32_t first_hit_timeout_ms;
    uint32_t second_hit_timeout_ms;
    uint32_t calibration_ms;
    uint16_t sensor_baseline_samples;
    uint16_t sensor_threshold;
    uint16_t sensor_hit_window_samples;
    int16_t small_rpm;
    int16_t sine_amplitude;
    int16_t sine_omega_milli;
    int16_t sine_offset;
    uint8_t adc_to_ring[PR_SENSOR_ADC_COUNT];
} pr_config_t;

void pr_config_defaults(pr_config_t *config);
bool pr_config_validate(const pr_config_t *config);
bool pr_config_load(pr_config_t *config);
bool pr_config_save(const pr_config_t *config);

#ifdef __cplusplus
}
#endif

#endif
