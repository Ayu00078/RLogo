/**
 * @file ble_sensor_monitor.cpp
 * @brief BLE 传感器监控模块实现
 *
 * 使用 Nordic UART Service（NUS）协议，通过 TX 特征值（Notify）向电脑推送
 * 传感器数据和命中事件，完全不影响现有 TCP 通信。
 *
 * NUS UUID：
 *   Service:        6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   TX Char(Notify): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 *
 * ESP32-S3 说明：
 *   - 仅有 BLE（无经典蓝牙），不需要 esp_bt_controller_mem_release
 *   - WiFi+BLE 共存通过 esp_wifi_set_ps(WIFI_PS_MIN_MODEM) 保证
 */

// ============================================================
//  ★★★  调参入口  ★★★
//
//  BLE_MON_FORCE_MODE — 跳过 RLogo，直接强制设置传感器检测模式
//
//   0  = 由 RLogo/APP 控制（正式比赛时使用）
//   1  = 强制大符模式（电机震动，alpha 低、阈值高）
//   2  = 强制小符模式（静止环境，alpha 高、阈值低）  ← 调参默认
//
//  修改后重新编译烧录即可，无需手机 APP 也无需 RLogo 在线。
// ============================================================
#define BLE_MON_FORCE_MODE  0

#include "ble_sensor_monitor.h"
#include "sensor_processor.h"   // sensor_processor_set_mode

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_wifi.h"

#include "PowerRune_Events.h"   // PRA_RUNE_BIG_MODE / PRA_RUNE_SMALL_MODE

static const char* TAG = "ble_mon";

// ============================================================
// NUS 128-bit UUID（小端序字节数组）
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// ============================================================
static const uint8_t k_nus_svc_uuid[ESP_UUID_LEN_128] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (TX, Notify)
static const uint8_t k_nus_tx_uuid[ESP_UUID_LEN_128] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

// ============================================================
// GATT 属性表下标
// ============================================================
enum {
    MON_IDX_SVC = 0,    // Primary Service Declaration
    MON_IDX_TX_CHAR,    // TX Characteristic Declaration
    MON_IDX_TX_VAL,     // TX Characteristic Value (notify to client)
    MON_IDX_TX_CFG,     // Client Characteristic Configuration Descriptor (CCCD)
    MON_IDX_NB,
};

#define MON_TX_MAX_LEN  200     // raw+diff 一行约 150 字节，留余量
#define MON_SVC_INST_ID 0
#define MON_APP_ID      0xAB    // 任意应用 ID，不与 RLogo 冲突

// ============================================================
// 属性表静态数据
// ============================================================
static const uint16_t k_primary_svc_uuid   = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t k_char_decl_uuid     = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t k_cccd_uuid          = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  k_prop_notify        = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t        s_tx_val[MON_TX_MAX_LEN] = {0};
static uint8_t        s_cccd_val[2]            = {0, 0};

static const esp_gatts_attr_db_t s_mon_gatt_db[MON_IDX_NB] = {

    // ─── Primary Service Declaration ───────────────────────────
    [MON_IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&k_primary_svc_uuid,
         ESP_GATT_PERM_READ,
         ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t*)k_nus_svc_uuid}
    },

    // ─── TX Characteristic Declaration ─────────────────────────
    [MON_IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&k_char_decl_uuid,
         ESP_GATT_PERM_READ,
         sizeof(k_prop_notify), sizeof(k_prop_notify), (uint8_t*)&k_prop_notify}
    },

    // ─── TX Characteristic Value ────────────────────────────────
    [MON_IDX_TX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t*)k_nus_tx_uuid,
         ESP_GATT_PERM_READ,
         MON_TX_MAX_LEN, 1, s_tx_val}
    },

    // ─── Client Characteristic Configuration Descriptor (CCCD) ─
    [MON_IDX_TX_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&k_cccd_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(s_cccd_val), s_cccd_val}
    },
};

// ============================================================
// 运行时状态
// ============================================================
static bool        s_inited         = false;
static bool        s_connected      = false;
static bool        s_notify_enabled = false;
static uint16_t    s_conn_id        = 0xFFFF;
static esp_gatt_if_t s_gatts_if     = ESP_GATT_IF_NONE;
static uint16_t    s_handle_table[MON_IDX_NB];
static uint16_t    s_mtu            = 23;   // 初始保守值，连接后更新

// 设备名缓冲（"PR-Arm-X\0"，最长 10 字节）
static char s_device_name[12] = "PR-Arm-?";

// ============================================================
// 广播参数（BLE 5.0 Extended Advertising，Legacy IND PDU）
// ESP32-S3 默认启用 BLE 5.0，传统 BLE 4.2 广播 API 不可用；
// 使用 ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND 等效于 ADV_IND，
// 可连接、可扫描，与所有 BLE 4.0+ 中央设备兼容。
// ============================================================
#define MON_EXT_ADV_INST    0   // 扩展广播集句柄

static esp_ble_gap_ext_adv_params_t s_ext_adv_params = {
    .type           = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min   = 0x100,    // 160 ms
    .interval_max   = 0x200,    // 320 ms
    .channel_map    = ADV_CHNL_ALL,
    .own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr      = {0},
    .filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power       = EXT_ADV_TX_PWR_NO_PREFERENCE,
    .primary_phy    = ESP_BLE_GAP_PRI_PHY_1M,
    .max_skip       = 0,
    .secondary_phy  = ESP_BLE_GAP_PHY_1M,
    .sid            = 0,
    .scan_req_notif = false,
};

// 广播使能参数（持续广播，不限次数）
static const esp_ble_gap_ext_adv_t s_ext_adv_en = {
    .instance   = MON_EXT_ADV_INST,
    .duration   = 0,
    .max_events = 0,
};

// 原始 AD 结构体缓冲（Flags + 完整设备名，≤31 字节）
static uint8_t  s_adv_raw[31];
static uint16_t s_adv_raw_len = 0;

// 根据当前设备名构建广播原始数据
static void build_adv_raw(void)
{
    uint8_t name_len = (uint8_t)strlen(s_device_name);
    uint8_t pos = 0;
    s_adv_raw[pos++] = 0x02;    // Flags 长度
    s_adv_raw[pos++] = 0x01;    // AD 类型: Flags
    s_adv_raw[pos++] = 0x06;    // LE General Discoverable | BR/EDR Not Supported
    s_adv_raw[pos++] = (uint8_t)(name_len + 1);
    s_adv_raw[pos++] = 0x09;    // AD 类型: Complete Local Name
    memcpy(s_adv_raw + pos, s_device_name, name_len);
    pos += name_len;
    s_adv_raw_len = pos;
}

// 启动/重启扩展广播（参数与数据已提前配置）
static void start_ext_advertising(void)
{
    esp_err_t ret = esp_ble_gap_ext_adv_start(1, &s_ext_adv_en);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ext_adv_start failed: %s", esp_err_to_name(ret));
    }
}

// ============================================================
// 内部发送辅助（自动截断到 MTU - 3）
// ============================================================
static void notify_send(const char* buf, uint16_t len)
{
    if (!s_connected || !s_notify_enabled) return;
    if (s_gatts_if == ESP_GATT_IF_NONE) return;

    // ATT 通知最大有效负载 = MTU - 3
    uint16_t max_payload = (s_mtu > 3) ? (s_mtu - 3) : 20;
    if (len > max_payload) {
        len = max_payload;
    }

    esp_err_t ret = esp_ble_gatts_send_indicate(
        s_gatts_if, s_conn_id,
        s_handle_table[MON_IDX_TX_VAL],
        len, (uint8_t*)buf,
        false   // false = Notification（不需要 ACK）
    );
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "send_indicate failed: %s", esp_err_to_name(ret));
    }
}

// ============================================================
// GAP 回调（BLE 5.0 扩展广播控制）
// 流程：set_params → config_data → start
// ============================================================
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t* param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "ext_adv_set_params failed: %d",
                     param->ext_adv_set_params.status);
        } else {
            build_adv_raw();
            esp_ble_gap_config_ext_adv_data_raw(MON_EXT_ADV_INST,
                                                s_adv_raw_len, s_adv_raw);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "ext_adv_data_set failed: %d",
                     param->ext_adv_data_set.status);
        } else {
            start_ext_advertising();
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "ext_adv_start failed: %d",
                     param->ext_adv_start.status);
        } else {
            ESP_LOGI(TAG, "advertising started: %s", s_device_name);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        ESP_LOGD(TAG, "ext adv stopped");
        break;

    default:
        break;
    }
}

// ============================================================
// GATTS 回调（服务和连接管理）
// ============================================================
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t* param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            s_gatts_if = gatts_if;
            ESP_LOGI(TAG, "GATTS reg OK, if=%d", gatts_if);
            esp_ble_gap_set_device_name(s_device_name);
            // 启动 BLE 5.0 扩展广播配置流程（set_params → data → start）
            esp_ble_gap_ext_adv_set_params(MON_EXT_ADV_INST, &s_ext_adv_params);
            esp_ble_gatts_create_attr_tab(s_mon_gatt_db, gatts_if,
                                          MON_IDX_NB, MON_SVC_INST_ID);
        } else {
            ESP_LOGE(TAG, "GATTS reg FAILED: %d", param->reg.status);
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "attr_tab create failed: 0x%x",
                     param->add_attr_tab.status);
        } else {
            memcpy(s_handle_table, param->add_attr_tab.handles,
                   sizeof(s_handle_table));
            esp_ble_gatts_start_service(s_handle_table[MON_IDX_SVC]);
            ESP_LOGI(TAG, "service started, tx_val_handle=%d",
                     s_handle_table[MON_IDX_TX_VAL]);
        }
        break;

    case ESP_GATTS_MTU_EVT:
        s_mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "MTU negotiated: %d", s_mtu);
        break;

    case ESP_GATTS_CONNECT_EVT: {
        s_conn_id   = param->connect.conn_id;
        s_connected = true;
        s_notify_enabled = false;
        ESP_LOGI(TAG, "client connected, conn_id=%d", s_conn_id);
        esp_ble_conn_update_params_t conn_params = {};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency  = 0;
        conn_params.max_int  = 0x0014;
        conn_params.min_int  = 0x000A;
        conn_params.timeout  = 400;
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected      = false;
        s_notify_enabled = false;
        s_conn_id        = 0xFFFF;
        ESP_LOGI(TAG, "client disconnected, restarting adv");
        start_ext_advertising();
        break;

    case ESP_GATTS_WRITE_EVT: {
        // 检查是否是对 CCCD 的写入（客户端订阅 / 取消订阅 Notify）
        if (!param->write.is_prep &&
            param->write.handle == s_handle_table[MON_IDX_TX_CFG] &&
            param->write.len >= 2) {
            uint16_t cccd_val = (uint16_t)(param->write.value[0]) |
                                ((uint16_t)(param->write.value[1]) << 8);
            s_notify_enabled = (cccd_val == 0x0001);
            ESP_LOGI(TAG, "CCCD write: 0x%04x -> notify %s",
                     cccd_val, s_notify_enabled ? "ENABLED" : "DISABLED");
        }
        break;
    }

    default:
        break;
    }
}

// ============================================================
// 公开接口实现
// ============================================================

void ble_sensor_monitor_init(uint8_t armour_id)
{
    if (s_inited) return;

    snprintf(s_device_name, sizeof(s_device_name), "PR-Arm-%d", (int)armour_id);
    ESP_LOGI(TAG, "init: device_name=%s", s_device_name);

    // ── BLE 控制器初始化（ESP32-S3 仅有 BLE，无经典蓝牙）─────────────
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // ── Bluedroid 协议栈初始化 ────────────────────────────────────────
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // ── WiFi+BLE 共存：让 WiFi 进入最小省电模式，给 BLE 留出时隙 ──────
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    // ── 注册回调并启动服务 ────────────────────────────────────────────
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(MON_APP_ID));

    // 设置本地 MTU 上限，允许客户端协商更大的 MTU（最大 517）
    esp_ble_gatt_set_local_mtu(244);

    s_inited = true;

    // ── 强制模式（跳过 RLogo）──────────────────────────────────────────
#if BLE_MON_FORCE_MODE == 1
    sensor_processor_set_mode(PRA_RUNE_BIG_MODE);
    ESP_LOGI(TAG, "force mode: BIG RUNE (alpha=low, thr=high)");
#elif BLE_MON_FORCE_MODE == 2
    sensor_processor_set_mode(PRA_RUNE_SMALL_MODE);
    ESP_LOGI(TAG, "force mode: SMALL RUNE (alpha=high, thr=low)");
#else
    ESP_LOGI(TAG, "mode: controlled by RLogo");
#endif

    ESP_LOGI(TAG, "init done");
}

void ble_sensor_monitor_send_frame(const uint8_t* adc, const float* diffs, size_t n,
                                   float max_diff, bool armed, uint8_t mode)
{
    if (!s_inited || !s_connected || !s_notify_enabled) return;
    if (!adc || !diffs || n == 0) return;

    char buf[MON_TX_MAX_LEN];
    char mode_ch = (mode == PRA_RUNE_BIG_MODE) ? 'B' : 'S';
    size_t ch = (n < 10) ? n : 10;

    // 格式: "$D <B/S> <armed> <raw0..raw9> <dif0..dif9> <max_diff>\n"
    int pos = snprintf(buf, sizeof(buf), "$D %c %d", mode_ch, (int)armed);

    // 原始 ADC（整数）
    for (size_t i = 0; i < ch && pos < (int)sizeof(buf) - 8; ++i) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %3d", (int)adc[i]);
    }

    // 差分（保留1位小数，正值=信号突起，接近阈值时说明即将命中）
    for (size_t i = 0; i < ch && pos < (int)sizeof(buf) - 8; ++i) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %5.1f", diffs[i]);
    }

    // 最大差分（当前帧峰值，与阈值比较判断是否命中）
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %.1f\n", max_diff);

    notify_send(buf, (uint16_t)pos);
}

void ble_sensor_monitor_send_hit(uint8_t ring, float max_diff, uint8_t mode)
{
    if (!s_inited || !s_connected || !s_notify_enabled) return;

    char buf[48];
    char mode_ch = (mode == PRA_RUNE_BIG_MODE) ? 'B' : 'S';

    // 格式: "$H <B/S> <ring> <max_diff>\n"
    int len = snprintf(buf, sizeof(buf), "$H %c %d %.1f\n",
                       mode_ch, (int)ring, max_diff);

    notify_send(buf, (uint16_t)len);
}
