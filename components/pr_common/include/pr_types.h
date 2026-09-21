#ifndef PR_TYPES_H
#define PR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PR_PROTOCOL_VERSION 1U
#define PR_BROADCAST_ADDRESS 0xFFU
#define PR_ARMOUR_COUNT 5U
#define PR_MOTOR_ADDRESS 5U
#define PR_DEVICE_COUNT 6U

#define PR_WIFI_SSID "PowerRune"
#define PR_WIFI_PASSWORD "PowerRune26"
#define PR_MASTER_IP "192.168.4.1"
#define PR_TCP_PORT 8080

#define PR_SENSOR_BAUD_RATE 460800U
#define PR_SENSOR_FRAME_SIZE 18U
#define PR_SENSOR_ADC_COUNT 10U
#define PR_SENSOR_BASELINE_SAMPLES 20U
#define PR_SENSOR_HIT_THRESHOLD 20U
#define PR_SENSOR_HIT_WINDOW_SAMPLES 50U
#define PR_HIT_ACK_TIMEOUT_MS 100U

/* Controller run direction values. CS means Armour/game flow continues
 * while the Master deliberately keeps the Motor stationary. */
#define PR_RUN_DIR_CW 0U
#define PR_RUN_DIR_CCW 1U
#define PR_RUN_DIR_CS 2U

#define PR_GAME_GROUPS 5U
#define PR_GAME_FIRST_HIT_TIMEOUT_MS 2500U
#define PR_GAME_SECOND_HIT_TIMEOUT_MS 1000U
#define PR_GAME_CALIBRATION_MS 10000U
#define PR_SMALL_RPM 1140

typedef enum {
    PR_DEVICE_ARMOUR = 0,
    PR_DEVICE_MOTOR = 1,
    PR_DEVICE_MASTER = 2,
} pr_device_kind_t;

typedef enum {
    PR_GAME_LARGE = 0,
    PR_GAME_SMALL = 1,
} pr_game_mode_t;

typedef enum {
    PR_MASTER_BOOT = 0,
    PR_MASTER_DISCOVERING,
    PR_MASTER_PROVISIONING,
    PR_MASTER_READY,
    PR_MASTER_CALIBRATING,
    PR_MASTER_RUNNING_GROUP,
    PR_MASTER_COMPLETING,
    PR_MASTER_SAFE_STOP,
} pr_master_state_t;

typedef enum {
    PR_ARMOUR_BOOT = 0,
    PR_ARMOUR_CONNECTING,
    PR_ARMOUR_IDLE,
    PR_ARMOUR_TARGET,
    PR_ARMOUR_NOTARGET,
    PR_ARMOUR_HIT,
    PR_ARMOUR_COMPLETE,
    PR_ARMOUR_FAULT,
} pr_armour_state_t;

typedef enum {
    PR_MOTOR_LOCKED = 0,
    PR_MOTOR_UNLOCKED_IDLE,
    PR_MOTOR_STARTING_CONSTANT,
    PR_MOTOR_RUNNING_CONSTANT,
    PR_MOTOR_STARTING_SINE,
    PR_MOTOR_RUNNING_SINE,
    PR_MOTOR_STOPPING,
    PR_MOTOR_SAFE_DISABLED,
} pr_motor_state_t;

typedef struct __attribute__((packed)) {
    uint8_t color;
    uint8_t mode;
    uint8_t loop;
    uint8_t dir;
} pr_run_command_t;

typedef struct __attribute__((packed)) {
    uint8_t armour_address;
    uint8_t score;
} pr_hit_event_t;

/*
 * The Armour -> Master hit candidate is deliberately separate from the
 * legacy two-byte hit payload.  A candidate is reported after the local
 * detector has completed its peak window, regardless of the current game
 * target state.  peak_x100 is the normalized peak multiplied by 100.
 */
typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint32_t session_id;
    uint32_t sequence;
    uint8_t ring;
    uint16_t peak_x100;
    uint32_t detected_ms;
} pr_hit_candidate_t;

typedef enum {
    PR_HIT_DECISION_ACCEPT = 0,
    PR_HIT_DECISION_NO_GAME = 1,
    PR_HIT_DECISION_BELOW_THRESHOLD = 2,
    PR_HIT_DECISION_WRONG_TARGET = 3,
    PR_HIT_DECISION_AMBIGUOUS = 4,
    PR_HIT_DECISION_INVALID = 5,
} pr_hit_decision_reason_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint32_t session_id;
    uint32_t sequence;
    uint8_t score;
    uint8_t accepted;
    uint8_t reason;
    uint8_t threshold_enabled;
    uint16_t threshold_x100;
    uint32_t decided_ms;
} pr_hit_decision_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint32_t session_id;
    uint32_t sequence;
    uint8_t received;
    uint8_t applied;
} pr_hit_decision_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint32_t session_id;
    uint32_t sequence;
    uint32_t elapsed_ms;
} pr_hit_timeout_t;

/* Master -> Armour: the persisted second-stage peak threshold. */
typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint8_t enabled;
    uint16_t threshold_x100;
} pr_hit_threshold_config_t;

/* Armour -> Master: acknowledgement that the local threshold was persisted. */
typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t data_len;
    uint8_t enabled;
    uint16_t threshold_x100;
    uint8_t status;
} pr_hit_threshold_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t armour_id;
    uint8_t enabled;
    uint16_t threshold_x100;
} pr_controller_threshold_set_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t armour_id;
} pr_controller_threshold_query_t;

typedef struct __attribute__((packed)) {
    uint16_t baseline_samples;
    uint16_t threshold;
    uint16_t hit_window_samples;
    uint8_t adc_to_ring[PR_SENSOR_ADC_COUNT];
} pr_sensor_params_t;

typedef struct __attribute__((packed)) {
    uint8_t physical_id;
    uint8_t protocol_address;
    uint8_t kind;
} pr_device_hello_t;

#if defined(__cplusplus)
static_assert(sizeof(pr_run_command_t) == 4, "RUN payload must remain four bytes");
static_assert(sizeof(pr_hit_event_t) == 2, "HIT payload must remain two bytes");
static_assert(sizeof(pr_hit_candidate_t) == 17, "hit candidate layout changed");
static_assert(sizeof(pr_hit_decision_t) == 20, "hit decision layout changed");
static_assert(sizeof(pr_hit_decision_ack_t) == 12, "hit decision ack layout changed");
static_assert(sizeof(pr_hit_timeout_t) == 14, "hit timeout layout changed");
static_assert(sizeof(pr_hit_threshold_config_t) == 5, "hit threshold config layout changed");
static_assert(sizeof(pr_hit_threshold_ack_t) == 6, "hit threshold ack layout changed");
static_assert(sizeof(pr_controller_threshold_set_t) == 5, "threshold set layout changed");
static_assert(sizeof(pr_controller_threshold_query_t) == 2, "threshold query layout changed");
static_assert(sizeof(pr_device_hello_t) == 3, "device hello layout changed");
#else
_Static_assert(sizeof(pr_run_command_t) == 4, "RUN payload must remain four bytes");
_Static_assert(sizeof(pr_hit_event_t) == 2, "HIT payload must remain two bytes");
_Static_assert(sizeof(pr_hit_candidate_t) == 17, "hit candidate layout changed");
_Static_assert(sizeof(pr_hit_decision_t) == 20, "hit decision layout changed");
_Static_assert(sizeof(pr_hit_decision_ack_t) == 12, "hit decision ack layout changed");
_Static_assert(sizeof(pr_hit_timeout_t) == 14, "hit timeout layout changed");
_Static_assert(sizeof(pr_hit_threshold_config_t) == 5, "hit threshold config layout changed");
_Static_assert(sizeof(pr_hit_threshold_ack_t) == 6, "hit threshold ack layout changed");
_Static_assert(sizeof(pr_controller_threshold_set_t) == 5, "threshold set layout changed");
_Static_assert(sizeof(pr_controller_threshold_query_t) == 2, "threshold query layout changed");
_Static_assert(sizeof(pr_device_hello_t) == 3, "device hello layout changed");
#endif

#ifdef __cplusplus
}
#endif

#endif
