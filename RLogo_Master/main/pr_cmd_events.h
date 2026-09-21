#pragma once
#include <stdint.h>
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(PRAPP);

enum {
    PRAPP_RUN_CMD_EVENT = 1,
    PRAPP_STOP_CMD_EVENT,
    PRAPP_UNLK_CMD_EVENT,
    PRAPP_OTA_CMD_EVENT,
};

typedef struct __attribute__((packed)) {
    uint8_t color; // 0 red, 1 blue
    uint8_t mode;  // 0 big,  1 small
    uint8_t loop;  // 0/1
    uint8_t dir;   // 0 cw, 1 ccw, 2 cs (Motor stationary)
} pr_run_cmd_t;

#ifdef __cplusplus
}
#endif
