#include "armour_app.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "firmware.h"
#include "tcp_protocol.h"
#include "PowerRune_Events.h"

#include "hit_reporter.h"
#include "hit_filter.h"
#include "sensor_processor.h"
#include "uart_handler.h"
#include "led_controller.h"
#include "ble_sensor_monitor.h"

#include <inttypes.h>

static const char* TAG = "armour";

TcpProtocol* tcp_protocol = nullptr;

static const char* state_name(int s) {
    switch(s) {
        case 0: return "IDLE";
        case 1: return "TARGET";
        case 2: return "HIT";
        case 3: return "BLINK";
        case 4: return "NOTARGET";
        case 5: return "DEBUG";
        default: return "?";
    }
}

// ==================== 状态机定义 ====================

enum LED_STRIP_STATE {
    LED_STRIP_IDLE,
    LED_STRIP_TARGET,
    LED_STRIP_HIT,
    LED_STRIP_BLINK,
    LED_STRIP_NOTARGET, // 专门为非目标状态添加
    LED_STRIP_DEBUG
};

struct LED_Strip_FSM_t {
    LED_STRIP_STATE state=LED_STRIP_IDLE;
   // LED_STRIP_STATE state = LED_STRIP_DEBUG;
    uint8_t mode = PRA_RUNE_BIG_MODE;
    uint8_t color = PR_RED;
    uint8_t group = 1;
    uint8_t score = 0;
    bool small_rune_hit = false;
};

static LED_Strip_FSM_t s_state;
static SemaphoreHandle_t s_fsm_sem = nullptr;
// 保护 s_state 多任务并发读写（led_update_task 与事件回调任务同时访问）
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

#define STATE_LOCK()   portENTER_CRITICAL(&s_state_mux)
#define STATE_UNLOCK() portEXIT_CRITICAL(&s_state_mux)

bool armour_app_is_target(void)
{
    STATE_LOCK();
    const bool target = (s_state.state == LED_STRIP_TARGET);
    STATE_UNLOCK();
    return target;
}

// ==================== LED 控制辅助函数 ====================

static void clear_armour() {
    led_strip_clear(led_arm);
    led_strip_clear(led_strip);
    led_strip_clear(led_board);
    led_strip_refresh(led_arm);
    led_strip_refresh(led_strip);
    led_strip_refresh(led_board);
}

// ==================== LED 更新主任务 ====================

static void led_update_task(void *pvParameter)
{
    LED_Strip_FSM_t local_state;
    LED_STRIP_STATE previous_state = LED_STRIP_IDLE;
    while (1) {
        STATE_LOCK();
        local_state = s_state; // 原子快照，避免读到半更新的结构体
        STATE_UNLOCK();

        if (local_state.state != previous_state) {
            ESP_LOGI(TAG, "FSM: %s -> %s (mode=%d color=%d group=%d score=%d hit=%d)",
                     state_name(previous_state), state_name(local_state.state),
                     local_state.mode, local_state.color, local_state.group,
                     local_state.score, local_state.small_rune_hit);
        }

        switch (local_state.state) {
            case LED_STRIP_IDLE:
                STATE_LOCK();
                s_state.small_rune_hit = false;
                STATE_UNLOCK();
                clear_armour();
                xSemaphoreTake(s_fsm_sem, portMAX_DELAY);
                break;

            case LED_STRIP_TARGET:
                if (previous_state != LED_STRIP_TARGET) {
                    led_strip_clear(led_strip);
                    led_strip_refresh(led_strip);
                }
                show_target(local_state.color);
                show_arrow(local_state.color);
                xSemaphoreTake(s_fsm_sem, pdMS_TO_TICKS(100));
                break;

            case LED_STRIP_NOTARGET:
                if(local_state.mode == PRA_RUNE_BIG_MODE) {
                    ESP_LOGD(TAG, "NOTARGET: big mode, show_arm group=%d", local_state.group);
                    led_strip_clear(led_board);
                    led_strip_refresh(led_board);
                    show_arm(local_state.color, local_state.group);
                } else if (local_state.small_rune_hit) {
                    ESP_LOGD(TAG, "NOTARGET: small mode hit, show_result score=%d", local_state.score);
                    show_result(local_state.color, 1);
                    show_arm(local_state.color, 5);
                } else {
                    ESP_LOGD(TAG, "NOTARGET: small mode no hit, clear all");
                    led_strip_clear(led_board);
                    led_strip_refresh(led_board);
                    led_strip_clear(led_arm);
                    led_strip_refresh(led_arm);
                    led_strip_clear(led_strip);
                    led_strip_refresh(led_strip);
                }
                xSemaphoreTake(s_fsm_sem, portMAX_DELAY);
                break;

            case LED_STRIP_HIT:
                if (local_state.mode == PRA_RUNE_SMALL_MODE) {
                    ESP_LOGI(TAG, "HIT: small mode, score=%d -> show_arm(5)", local_state.score);
                    STATE_LOCK();
                    s_state.small_rune_hit = true;
                    STATE_UNLOCK();
                    if (local_state.score > 0) {
                        show_result(local_state.color, 1);
                    }
                    show_arm(local_state.color, 5);
                    
                } else {
                    ESP_LOGI(TAG, "HIT: big mode, show_arm group=%d", local_state.group);
                    led_strip_clear(led_board);
                    led_strip_refresh(led_board);
                    show_arm(local_state.color, local_state.group);
                }
                xSemaphoreTake(s_fsm_sem, portMAX_DELAY);
                break;

            case LED_STRIP_BLINK: {
                uint8_t my_id = config->get_config_info_pt()->armour_id;
                ESP_LOGI(TAG, "BLINK: armour_id=%d (simultaneous)", my_id);

                for (uint8_t i = 0; i < 5; i++) {
                    clear_armour();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    // show_target(local_state.color);
                    // show_arm(local_state.color, 5);
                    show_success(local_state.color);
                    vTaskDelay(pdMS_TO_TICKS(100));

                    if (xSemaphoreTake(s_fsm_sem, 0) == pdTRUE) {
                        STATE_LOCK();
                        bool still_blink = (s_state.state == LED_STRIP_BLINK);
                        STATE_UNLOCK();
                        if (!still_blink) break;
                    }
                }

                STATE_LOCK();
                s_state.state = LED_STRIP_IDLE;
                STATE_UNLOCK();
                break;
            }

            case LED_STRIP_DEBUG:
                show_target(local_state.color);
                xSemaphoreTake(s_fsm_sem, pdMS_TO_TICKS(100));
                break;
        }
        previous_state = local_state.state;
    }
}

// ==================== 事件回调 ====================

static uint8_t get_my_id() {
    return TcpProtocol::runtime_armour_address();
}

static void on_pra_start(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_START_EVENT_DATA*)event_data;
    uint8_t my = get_my_id();

    if (d->address == my) {
        sensor_processor_set_mode(d->mode);

        STATE_LOCK();
        s_state.color = d->color;
        s_state.group = d->group;
        s_state.mode  = d->mode;
        s_state.state = LED_STRIP_TARGET;
        STATE_UNLOCK();

        ESP_LOGI(TAG, "<<< START MATCH: my_id=%d mode=%d color=%d group=%d",
                 my, d->mode, d->color, d->group);
        xSemaphoreGive(s_fsm_sem);
    } else {
        ESP_LOGD(TAG, "<<< START SKIP: target=%d my_id=%d", d->address, my);
    }
}

static void on_pra_notarget(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_NOTARGET_DATA*)event_data;
    uint8_t my = get_my_id();
    if (d->address == my) {
        STATE_LOCK();
        s_state.group = d->group;
        s_state.mode  = d->mode;
        s_state.color = d->color;
        s_state.state = LED_STRIP_NOTARGET;
        STATE_UNLOCK();
        ESP_LOGI(TAG, "<<< NOTARGET MATCH: my_id=%d mode=%d group=%d color=%d", my, d->mode, d->group, d->color);
        xSemaphoreGive(s_fsm_sem);
    } else {
        ESP_LOGD(TAG, "<<< NOTARGET SKIP: target=%d my_id=%d", d->address, my);
    }
}

static void on_pra_hit(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_HIT_EVENT_DATA*)event_data;
    if (!d) return;

    bool threshold_enabled = false;
    uint16_t threshold_x100 = 0;
    hit_filter_get(&threshold_enabled, &threshold_x100);
    const bool threshold_pass = !threshold_enabled ||
                                d->peak_x100 >= threshold_x100;
    const bool ring_valid = d->ring != 0;
    bool applied = false;

    // The Armour owns the immediate light response.  Master still receives
    // every candidate and independently decides whether it counts in-game.
    STATE_LOCK();
    if (s_state.state == LED_STRIP_TARGET && ring_valid && threshold_pass) {
        s_state.score = d->ring;
        if (s_state.mode == PRA_RUNE_SMALL_MODE) {
            s_state.small_rune_hit = true;
        }
        s_state.state = LED_STRIP_HIT;
        applied = true;
    }
    STATE_UNLOCK();

    if (applied) {
        ESP_LOGI(TAG, "<<< LOCAL HIT LIGHT: ring=%u peak=%u.%02u seq=%" PRIu32,
                 d->ring, d->peak_x100 / 100U, d->peak_x100 % 100U,
                 d->sequence);
        xSemaphoreGive(s_fsm_sem);
    } else {
        ESP_LOGD(TAG, "<<< HIT CANDIDATE: ring=%u peak=%u.%02u seq=%" PRIu32
                 " threshold=%s:%u.%02u valid=%u",
                 d->ring, d->peak_x100 / 100U, d->peak_x100 % 100U,
                 d->sequence, threshold_enabled ? "ON" : "OFF",
                 threshold_x100 / 100U, threshold_x100 % 100U,
                 ring_valid ? 1U : 0U);
    }
}

static void on_pra_hit_decision(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_HIT_DECISION_EVENT_DATA*)event_data;
    if (!d) return;

    uint32_t elapsed_ms = 0;
    const bool on_time = sensor_processor_mark_hit_decision(d, &elapsed_ms);
    if (d->accepted && d->reason == PR_HIT_DECISION_ACCEPT && on_time) {
        ESP_LOGD(TAG, "<<< MASTER HIT CONFIRMED: ring=%u seq=%" PRIu32
                 " elapsed=%" PRIu32 "ms (light handled locally)",
                 d->score, d->sequence, elapsed_ms);
    } else if (!on_time) {
        ESP_LOGW(TAG, "<<< HIT decision arrived after deadline: seq=%" PRIu32
                 " accepted=%u elapsed=%" PRIu32 "ms",
                 d->sequence, d->accepted, elapsed_ms);
    } else {
        ESP_LOGD(TAG, "<<< HIT decision rejected: seq=%" PRIu32 " reason=%u peak-threshold=%u.%02u",
                 d->sequence, d->reason, d->threshold_x100 / 100U,
                 d->threshold_x100 % 100U);
    }

    PRA_HIT_DECISION_ACK_EVENT_DATA ack = {};
    ack.address = get_my_id();
    ack.data_len = sizeof(ack);
    ack.session_id = d->session_id;
    ack.sequence = d->sequence;
    ack.received = 1;
    ack.applied = (d->accepted &&
                   d->reason == PR_HIT_DECISION_ACCEPT && on_time) ? 1 : 0;
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_HIT_DECISION_ACK_EVENT,
                      &ack, sizeof(ack), 0);
}

static void on_pra_hit_threshold_config(void*, esp_event_base_t, int32_t,
                                        void* event_data)
{
    const auto *d = (PRA_HIT_THRESHOLD_CONFIG_EVENT_DATA *)event_data;
    const uint8_t my = get_my_id();
    PRA_HIT_THRESHOLD_ACK_EVENT_DATA ack = {};
    ack.address = my;
    ack.data_len = sizeof(ack);

    if (!d || d->data_len < sizeof(*d) || d->address != my ||
        d->enabled > 1U || d->threshold_x100 > 25500U) {
        ESP_LOGW(TAG, "<<< invalid hit threshold config");
        ack.status = 1;
    } else {
        const esp_err_t err = hit_filter_set(d->enabled != 0,
                                             d->threshold_x100);
        ack.enabled = d->enabled;
        ack.threshold_x100 = d->threshold_x100;
        ack.status = err == ESP_OK ? 0 : 1;
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "<<< HIT threshold synced: %s %u.%02u",
                     d->enabled ? "ON" : "OFF",
                     d->threshold_x100 / 100U,
                     d->threshold_x100 % 100U);
        }
    }

    esp_event_post_to(pr_events_loop_handle, PRA,
                      PRA_HIT_THRESHOLD_ACK_EVENT, &ack, sizeof(ack), 0);
}

static void on_pra_stop(void*, esp_event_base_t, int32_t, void*)
{
    STATE_LOCK();
    LED_STRIP_STATE prev = s_state.state;
    s_state.state = LED_STRIP_IDLE;
    STATE_UNLOCK();
    ESP_LOGI(TAG, "<<< STOP (was %s)", state_name(prev));
    sensor_processor_set_mode(PRA_RUNE_SMALL_MODE);
    xSemaphoreGive(s_fsm_sem);
}

// RLogo 下发的传感器参数：应用 + 回 ACK
static void on_pra_set_sensor_param(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_SET_SENSOR_PARAM_DATA*)event_data;
    uint8_t my = get_my_id();

    ESP_LOGI(TAG, "<<< SET SENSOR PARAM: alpha_big=%.4f alpha_small=%.4f thr_small=%.1f thr_big=%.1f refract=%u ch=%u (my_id=%d)",
             d->ema_alpha_big, d->ema_alpha_small, d->hit_threshold_small,
             d->hit_threshold_big, (unsigned)d->refractory_ms,
             (unsigned)d->channel_count, my);

    // 应用两套配置（会重置基线，仅在空闲时调用是安全的）
    // 内部默认起步小符模式，等 START 事件时再 set_mode 切到大符
    size_t ch = d->channel_count == 0 ? 10 : d->channel_count;
    sensor_processor_set_params(ch,
                                d->ema_alpha_big,
                                d->ema_alpha_small,
                                d->hit_threshold_big,
                                d->hit_threshold_small,
                                d->refractory_ms);

    // 回执 ACK 给 RLogo（仅自己的 address，让 RLogo 知道是哪一片回的）
    PRA_SENSOR_PARAM_ACK_DATA ack = {};
    ack.address  = my;
    ack.data_len = sizeof(ack);
    ack.status   = ESP_OK;
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_SENSOR_PARAM_ACK_EVENT,
                      &ack, sizeof(ack), portMAX_DELAY);
}

static void on_pra_complete(void*, esp_event_base_t, int32_t, void*)
{
    ESP_LOGI(TAG, "<<< COMPLETE -> BLINK");
    STATE_LOCK();
    s_state.state = LED_STRIP_BLINK;
    STATE_UNLOCK();
    xSemaphoreGive(s_fsm_sem);
}

static void on_pra_calibrate(void*, esp_event_base_t, int32_t, void*)
{
    ESP_LOGI(TAG, "<<< CALIBRATE: wait 2s then average 500ms");
    sensor_processor_start_calibration();
}

static void on_pra_assign_id(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* data = (PRA_ASSIGN_ID_DATA*)event_data;
    const uint8_t fixed_address = (uint8_t)(CONFIG_ARMOUR_ID - 1);
    if (data == nullptr || data->address != fixed_address) {
        ESP_LOGE(TAG, "<<< ASSIGN_ID rejected: fixed address=%u received=%u",
                 fixed_address, data == nullptr ? 0xFFU : data->address);
        return;
    }
    ESP_LOGI(TAG, "<<< ASSIGN_ID matches fixed address=%u; no runtime reassignment", fixed_address);
}

// ==================== 系统初始化 ====================

void armour_app_init(void)
{
    ESP_LOGI(TAG, "init");

    // ===== Create Event Loop early for Config/Firmware =====
    esp_event_loop_args_t loop_args = {
        .queue_size = 16,
        .task_name = "pr_events_loop",
        .task_priority = 4,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // ===== 1) 硬件与外设初始化 =====
    static Firmware firmware;
    (void)firmware;
    hit_filter_init();

    init_led();
    
    // FSM 信号量初始化
    s_fsm_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(s_fsm_sem); // 给定初始令牌以启动循环

    // TCP or Network is already initialized in Firmware::wifi_ota_init()
    // However firmware might set APSTA, while TCP constructor sets STA.
    // Just initialize TCP protocol
    tcp_protocol = new TcpProtocol();

    // ===== 3) 事件注册 =====
    // 向主控发送 HIT / 参数 ACK 事件
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_SENSOR_PARAM_ACK_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_DECISION_ACK_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_TIMEOUT_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_THRESHOLD_ACK_EVENT, TcpProtocol::tx_event_handler, nullptr));

    // 接收主控指令
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, on_pra_start, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_NOTARGET_EVENT, on_pra_notarget, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, on_pra_hit, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_DECISION_EVENT, on_pra_hit_decision, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_THRESHOLD_CONFIG_EVENT, on_pra_hit_threshold_config, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, on_pra_stop, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, on_pra_complete, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_SET_SENSOR_PARAM_EVENT, on_pra_set_sensor_param, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_CALIBRATE_EVENT, on_pra_calibrate, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_ASSIGN_ID_EVENT, on_pra_assign_id, nullptr));

    // ===== 4) 传感器与后台任务启动 =====

    sensor_processor_init();
    sensor_processor_set_raw_log(false);
    uart_handler_init();

    // ===== 5) BLE 监控模块（在 WiFi/TCP 就绪后初始化）=====
    // 广播名 "PR-Arm-X"，电脑用 ble_monitor.py 连接后可实时查看传感器数据
   // ble_sensor_monitor_init(config->get_config_info_pt()->armour_id);

    // 启动 FSM 动画任务
    xTaskCreate(led_update_task, "led_fsm_task", 6144, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "init done, armour_id=%d (address=%d)",
             config->get_config_info_pt()->armour_id, get_my_id());
}

void armour_app_start(void)
{
    // 启动 UART 开始接收压力传感器数据
    uart_handler_start();
}
