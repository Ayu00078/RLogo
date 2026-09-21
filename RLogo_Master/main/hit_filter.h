#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void hit_filter_init(void);
bool hit_filter_get(uint8_t address, bool *enabled, uint16_t *threshold_x100);
esp_err_t hit_filter_set(uint8_t address, bool enabled, uint16_t threshold_x100);
esp_err_t hit_filter_sync_armour(uint8_t address);

#ifdef __cplusplus
}
#endif
