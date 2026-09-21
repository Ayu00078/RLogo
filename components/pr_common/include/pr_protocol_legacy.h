#ifndef PR_PROTOCOL_LEGACY_H
#define PR_PROTOCOL_LEGACY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The TCP wire format is frozen by the original rlogo-master firmware. */
#define PR_TCP_HEADER 0x24DFU
#define PR_TCP_DATA_LEN 200U
#define PR_TCP_ADDR_LEN 6U
#define PR_TCP_FRAME_SIZE 219U

#define PR_BASE_COMMON "PRC"
#define PR_BASE_ARMOUR "PRA"
#define PR_BASE_MOTOR "PRM"

/* Controller is a TCP peer, but it must not occupy Armour/Motor addresses. */
#define PR_CONTROLLER_ADDRESS 0xFEU
#define PR_CONTROLLER_LOG_MAX_TEXT (PR_TCP_DATA_LEN - 2U)

/* Original event numbers. Do not renumber these values. */
enum {
    PR_EVT_STOP = 0,
    PR_EVT_START = 1,
    PR_EVT_HIT = 2,
    PR_EVT_COMPLETE = 3,
    PR_EVT_PING = 4,
    PR_EVT_NOTARGET = 5,
    PR_EVT_SET_SENSOR_PARAM = 6,
    PR_EVT_SENSOR_PARAM_ACK = 7,
    PR_EVT_CALIBRATE = 8,
    PR_EVT_ASSIGN_ID = 9,
    PR_EVT_HIT_DECISION = 10,
    PR_EVT_HIT_DECISION_ACK = 11,
    PR_EVT_HIT_TIMEOUT = 12,
    PR_EVT_HIT_THRESHOLD_CONFIG = 13,
    PR_EVT_HIT_THRESHOLD_ACK = 14,
};

enum {
    PRM_EVT_UNLOCK = 0,
    PRM_EVT_UNLOCK_DONE = 1,
    PRM_EVT_START = 2,
    PRM_EVT_START_DONE = 3,
    PRM_EVT_SPEED_STABLE = 4,
    PRM_EVT_STOP = 5,
    PRM_EVT_DISCONNECT = 6,
    PRM_EVT_PING = 7,
};

enum {
    PRC_EVT_OTA_BEGIN = 0,
    PRC_EVT_OTA_COMPLETE = 1,
    PRC_EVT_CONFIG = 2,
    PRC_EVT_CONFIG_COMPLETE = 3,
    PRC_EVT_RESPONSE = 4,
    PRC_EVT_BEACON_TIMEOUT = 5,
    PRC_EVT_CONTROLLER_HELLO = 6,
    PRC_EVT_CONTROLLER_PING = 7,
    PRC_EVT_CONTROLLER_CONFIG = 8,
    PRC_EVT_CONTROLLER_START = 9,
    PRC_EVT_CONTROLLER_STOP = 10,
    PRC_EVT_CONTROLLER_UNLOCK = 11,
    PRC_EVT_CONTROLLER_OTA = 12,
    PRC_EVT_CONTROLLER_STATUS_REQUEST = 13,
    PRC_EVT_CONTROLLER_DEBUG = 14,
    PRC_EVT_CONTROLLER_LOG = 15,
    PRC_EVT_CONTROLLER_THRESHOLD_SET = 16,
    PRC_EVT_CONTROLLER_THRESHOLD_OFF = 17,
    PRC_EVT_CONTROLLER_THRESHOLD_SHOW = 18,
    PRC_EVT_CONTROLLER_REDLED_ON = 19,
    PRC_EVT_CONTROLLER_REDLED_OFF = 20,
};

enum {
    PR_CONTROLLER_LOG_INFO = 1,
    PR_CONTROLLER_LOG_WARN = 2,
    PR_CONTROLLER_LOG_ERROR = 3,
    PR_CONTROLLER_LOG_DEBUG = 4,
};

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t protocol_version;
} pr_controller_hello_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t color;
    uint8_t mode;
    uint8_t loop;
    uint8_t dir;
} pr_controller_run_config_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t enabled;
} pr_controller_debug_t;

typedef struct __attribute__((packed)) {
    uint16_t header;
    uint16_t pack_id;
    uint16_t crc16;
    char event_base[4];
    uint16_t event_data_len;
    uint8_t event_id;
    uint8_t event_data[PR_TCP_DATA_LEN];
    uint8_t dest_mac[PR_TCP_ADDR_LEN];
} pr_wire_packet_t;

typedef struct {
    uint16_t pack_id;
    uint16_t event_data_len;
    uint8_t event_id;
    char event_base[4];
    uint8_t event_data[PR_TCP_DATA_LEN];
    uint8_t dest_mac[PR_TCP_ADDR_LEN];
} pr_packet_t;

#if defined(__cplusplus)
static_assert(sizeof(pr_wire_packet_t) == PR_TCP_FRAME_SIZE,
              "legacy TCP frame layout changed");
static_assert(PR_EVT_STOP == 0 && PR_EVT_START == 1 && PR_EVT_HIT == 2,
              "PRA event numbers changed");
static_assert(PRM_EVT_UNLOCK == 0 && PRM_EVT_START == 2 && PRM_EVT_STOP == 5,
              "PRM event numbers changed");
#else
_Static_assert(sizeof(pr_wire_packet_t) == PR_TCP_FRAME_SIZE,
               "legacy TCP frame layout changed");
_Static_assert(PR_EVT_STOP == 0 && PR_EVT_START == 1 && PR_EVT_HIT == 2,
               "PRA event numbers changed");
_Static_assert(PRM_EVT_UNLOCK == 0 && PRM_EVT_START == 2 && PRM_EVT_STOP == 5,
               "PRM event numbers changed");
#endif

void pr_packet_init(pr_packet_t *packet, const char *event_base, uint8_t event_id,
                    uint8_t address);
bool pr_packet_set_payload(pr_packet_t *packet, const void *payload, size_t length);
bool pr_packet_validate(const pr_packet_t *packet);
bool pr_packet_encode(const pr_packet_t *packet, uint8_t *buffer, size_t capacity,
                      size_t *encoded_size);
bool pr_packet_decode(const uint8_t *buffer, size_t length, pr_packet_t *packet);
bool pr_packet_is(const pr_packet_t *packet, const char *event_base, uint8_t event_id);
const char *pr_event_name(const char *event_base, uint8_t event_id);

#ifdef __cplusplus
}
#endif

#endif
