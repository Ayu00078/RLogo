#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void device_registry_init(void);
void device_registry_start(void);

uint8_t device_registry_armour_mask(void);
bool device_registry_motor_online(void);
bool device_registry_all_ready(void);

#ifdef __cplusplus
}
#endif