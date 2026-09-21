/**
 * @file PowerRune_Events.h
 * @brief 大符事件库
 * @version 1.9
 * @date 2026-04-06
 */
#pragma once
#include "firmware.h"
#include "esp_event.h"
#include "pr_types.h"

ESP_EVENT_DECLARE_BASE(PRC);
ESP_EVENT_DECLARE_BASE(PRA);
ESP_EVENT_DECLARE_BASE(PRM);

#define MOTOR 5

#pragma pack(1)
// 事件循环Handle
extern esp_event_loop_handle_t pr_events_loop_handle;
// ADDRESS
// 5个ESP32S3[0:4], 1个ESP32C3[5]
// 公有事件
enum
{
    OTA_BEGIN_EVENT,
    OTA_COMPLETE_EVENT,
    CONFIG_EVENT,
    CONFIG_COMPLETE_EVENT,
    RESPONSE_EVENT,
    BEACON_TIMEOUT_EVENT,
};

struct OTA_BEGIN_EVENT_DATA
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(OTA_BEGIN_EVENT_DATA);
};
struct OTA_COMPLETE_EVENT_DATA
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(OTA_COMPLETE_EVENT_DATA);
    esp_err_t status;
    uint8_t ota_type; // 1 for startup ota, 0&2 for manual ota
};
struct CONFIG_EVENT_DATA // 比较浪费，但是省事
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(CONFIG_EVENT_DATA);
    PowerRune_Common_config_info_t config_common_info;
    PowerRune_Armour_config_info_t config_armour_info;
    PowerRune_Motor_config_info_t config_motor_info;
    PowerRune_Rlogo_config_info_t config_rlogo_info;
};
struct CONFIG_COMPLETE_EVENT_DATA
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(CONFIG_COMPLETE_EVENT_DATA);
    esp_err_t status;
};
struct RESPONSE_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(RESPONSE_EVENT_DATA);
};
// Beacon Timeout不需要数据

// Armour事件
enum
{
    PRA_STOP_EVENT,
    PRA_START_EVENT,
    PRA_HIT_EVENT,
    PRA_COMPLETE_EVENT,
    PRA_PING_EVENT,
    PRA_NOTARGET_EVENT,
    PRA_SET_SENSOR_PARAM_EVENT, // RLogo -> Armour: 下发传感器参数
    PRA_SENSOR_PARAM_ACK_EVENT, // Armour -> RLogo: 已接收并应用
    PRA_CALIBRATE_EVENT,        // RLogo -> Armour: 电机启动同步校准
    PRA_ASSIGN_ID_EVENT,        // RLogo -> Armour: 按TCP连接顺序分配运行时地址
    PRA_HIT_DECISION_EVENT,
    PRA_HIT_DECISION_ACK_EVENT,
    PRA_HIT_TIMEOUT_EVENT,
    PRA_HIT_THRESHOLD_CONFIG_EVENT,
    PRA_HIT_THRESHOLD_ACK_EVENT,
};

enum RUNE_MODE
{
    PRA_RUNE_BIG_MODE,
    PRA_RUNE_SMALL_MODE,
};

enum RUNE_COLOR
{
    PR_RED,
    PR_BLUE,
};

struct PRA_PING_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_PING_EVENT_DATA);
    PowerRune_Armour_config_info_t config_info;
};

struct PRA_START_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_START_EVENT_DATA);
    uint8_t mode = PRA_RUNE_BIG_MODE;
    uint8_t color = PR_RED;
    uint8_t group=0;
};

struct PRA_NOTARGET_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_NOTARGET_DATA);
    uint8_t group = 0;
    uint8_t mode = PRA_RUNE_BIG_MODE;
    uint8_t color = PR_RED;
};

struct PRA_STOP_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_STOP_EVENT_DATA);
};

typedef pr_hit_candidate_t PRA_HIT_EVENT_DATA;
typedef pr_hit_decision_t PRA_HIT_DECISION_EVENT_DATA;
typedef pr_hit_decision_ack_t PRA_HIT_DECISION_ACK_EVENT_DATA;
typedef pr_hit_timeout_t PRA_HIT_TIMEOUT_EVENT_DATA;
typedef pr_hit_threshold_config_t PRA_HIT_THRESHOLD_CONFIG_EVENT_DATA;
typedef pr_hit_threshold_ack_t PRA_HIT_THRESHOLD_ACK_EVENT_DATA;

struct PRA_COMPLETE_EVENT_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_COMPLETE_EVENT_DATA);
};

// RLogo -> Armour: 下发传感器处理参数（默认 address=0xFF 广播给全部）
struct PRA_SET_SENSOR_PARAM_DATA
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(PRA_SET_SENSOR_PARAM_DATA);
    float   ema_alpha_big;       // 大符模式 EMA alpha
    float   ema_alpha_small;     // 小符模式 EMA alpha
    float   hit_threshold_small; // 小符模式单通道触发阈值（静止干净，可低些更灵敏）
    float   hit_threshold_big;   // 大符模式单通道触发阈值（电机震动，需高些防误触）
    uint32_t refractory_ms;      // 防抖时间(ms)：持续安静≥此值才重新武装
    uint16_t channel_count;      // 通道数（通常 10）
};

// Armour -> RLogo: 接收完成回执
struct PRA_SENSOR_PARAM_ACK_DATA
{
    uint8_t address; // 0..4，发送方 armour 的地址（armour_id - 1）
    uint8_t data_len = sizeof(PRA_SENSOR_PARAM_ACK_DATA);
    esp_err_t status;
};

struct PRA_CALIBRATE_EVENT_DATA
{
    uint8_t address = 0xFF;
    uint8_t data_len = sizeof(PRA_CALIBRATE_EVENT_DATA);
};

struct PRA_ASSIGN_ID_DATA
{
    uint8_t address;
    uint8_t data_len = sizeof(PRA_ASSIGN_ID_DATA);
};

// 电机事件
enum
{
    PRM_UNLOCK_EVENT,
    PRM_UNLOCK_DONE_EVENT,
    PRM_START_EVENT,
    PRM_START_DONE_EVENT,
    PRM_SPEED_STABLE_EVENT,
    PRM_STOP_EVENT,
    PRM_DISCONNECT_EVENT,
    PRM_PING_EVENT,
};

enum PRM_DIRECTION
{
    PRM_DIRECTION_CLOCKWISE,     // 顺时针
    PRM_DIRECTION_ANTICLOCKWISE, // 逆时针
};

struct PRM_PING_EVENT_DATA
{
    uint8_t address = 0x05;
    uint8_t data_len = sizeof(PRM_PING_EVENT_DATA);
    PowerRune_Motor_config_info_t config_info;
};

struct PRM_UNLOCK_EVENT_DATA
{
    uint8_t address = 0x05;
    uint8_t data_len = sizeof(PRM_UNLOCK_EVENT_DATA);
};

struct PRM_UNLOCK_DONE_EVENT_DATA
{
    uint8_t address = 0x06;
    uint8_t data_len = sizeof(PRM_UNLOCK_DONE_EVENT_DATA);
    esp_err_t status;
};

struct PRM_START_EVENT_DATA
{
    uint8_t address = 0x05;
    uint8_t data_len = sizeof(PRM_START_EVENT_DATA);
    uint8_t mode = PRA_RUNE_BIG_MODE;
    uint8_t clockwise = PRM_DIRECTION_CLOCKWISE;
    float amplitude = 1.045;
    float omega = 1.884;
    float offset = 1.045;
};

struct PRM_START_DONE_EVENT_DATA
{
    uint8_t address = 0x06;
    uint8_t data_len = sizeof(PRM_START_DONE_EVENT_DATA);
    esp_err_t status;
    uint8_t mode;
};

struct PRM_SPEED_STABLE_EVENT_DATA
{
    uint8_t address = 0x06;
    uint8_t data_len = sizeof(PRM_SPEED_STABLE_EVENT_DATA);
};

struct PRM_STOP_EVENT_DATA
{
    uint8_t address = 0x05;
    uint8_t data_len = sizeof(PRM_STOP_EVENT_DATA);
};

struct PRM_DISCONNECT_EVENT_DATA
{
    uint8_t address = 0x06;
    uint8_t data_len = sizeof(PRM_DISCONNECT_EVENT_DATA);
};

#pragma pack()
