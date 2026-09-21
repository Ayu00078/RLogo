#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void hit_filter_init(void);
bool hit_filter_get(bool *enabled, uint16_t *threshold_x100);
esp_err_t hit_filter_set(bool enabled, uint16_t threshold_x100);

#ifdef __cplusplus
}
#endif
