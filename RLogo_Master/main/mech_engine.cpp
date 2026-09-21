
#include "mech_engine.h"

#include <string.h>
#include <vector>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tcp_protocol.h"
#include "pr_cmd_events.h"
#include "PowerRune_Events.h"
#include "LED_Strip.h"
#include "device_registry.h"
#include "hit_filter.h"
#include "controller_gateway.h"
#include "pr_types.h"

extern esp_event_loop_handle_t pr_events_loop_handle;
extern LED_Strip* led_strip;

static const char* TAG = "mech";

typedef enum {
    QMSG_RUN = 1,
    QMSG_STOP,
    QMSG_UNLK,
    QMSG_HIT,
    QMSG_OTA,
} qmsg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t ring;
} hit_pod_t;

typedef struct {
    qmsg_type_t type;
    union {
        pr_run_cmd_t run;
        hit_pod_t    hit;
    } u;
} qmsg_t;

static QueueHandle_t s_q = nullptr;

typedef struct {
    bool running = false;
    bool rerun = false;
    pr_run_cmd_t cmd = {};
    uint16_t sum_ring = 0;
    std::vector<uint8_t> gpa_hist;
    
    // 缓存大符当前随机参数
    float cur_amp = 0;
    float cur_omega = 0;
    float cur_offset = 0;
} mech_ctx_t;

static mech_ctx_t s_ctx;

static bool s_hit_context_active = false;
static uint8_t s_expected_hit_a = 0xFF;
static uint8_t s_expected_hit_b = 0xFF;
static portMUX_TYPE s_hit_context_mux = portMUX_INITIALIZER_UNLOCKED;

struct pending_decision_t {
    bool active = false;
    TickType_t sent_tick = 0;
    PRA_HIT_EVENT_DATA candidate = {};
};

static constexpr size_t kPendingDecisionCount = 32;
static pending_decision_t s_pending_decisions[kPendingDecisionCount] = {};
static portMUX_TYPE s_pending_decision_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_decision_watchdog_task = nullptr;

static constexpr uint8_t ARMOUR_COUNT = PR_ARMOUR_COUNT;
static constexpr uint8_t ARMOUR_MASK_ALL = (1U << ARMOUR_COUNT) - 1U;

bool mech_engine_is_running(void)
{
    return s_ctx.running;
}

static void set_hit_context(bool active, uint8_t expected_a, uint8_t expected_b)
{
    portENTER_CRITICAL(&s_hit_context_mux);
    s_hit_context_active = active;
    s_expected_hit_a = expected_a;
    s_expected_hit_b = expected_b;
    portEXIT_CRITICAL(&s_hit_context_mux);
}

static void get_hit_context(bool *active, uint8_t *expected_a, uint8_t *expected_b)
{
    portENTER_CRITICAL(&s_hit_context_mux);
    if (active) *active = s_hit_context_active;
    if (expected_a) *expected_a = s_expected_hit_a;
    if (expected_b) *expected_b = s_expected_hit_b;
    portEXIT_CRITICAL(&s_hit_context_mux);
}

static const char *decision_reason_name(uint8_t reason)
{
    switch (reason) {
    case PR_HIT_DECISION_ACCEPT: return "ACCEPT";
    case PR_HIT_DECISION_NO_GAME: return "NO_GAME";
    case PR_HIT_DECISION_BELOW_THRESHOLD: return "BELOW_THRESHOLD";
    case PR_HIT_DECISION_WRONG_TARGET: return "WRONG_TARGET";
    case PR_HIT_DECISION_AMBIGUOUS: return "AMBIGUOUS";
    default: return "INVALID";
    }
}

static bool pending_decision_add(const PRA_HIT_EVENT_DATA *candidate)
{
    bool added = false;
    portENTER_CRITICAL(&s_pending_decision_mux);
    for (auto &pending : s_pending_decisions) {
        if (!pending.active) {
            pending.active = true;
            pending.sent_tick = xTaskGetTickCount();
            pending.candidate = *candidate;
            added = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pending_decision_mux);
    return added;
}

static bool pending_decision_ack(const PRA_HIT_DECISION_ACK_EVENT_DATA *ack,
                                 uint32_t *elapsed_ms)
{
    if (elapsed_ms) *elapsed_ms = 0;
    if (!ack) return false;
    const TickType_t now = xTaskGetTickCount();
    bool found = false;
    portENTER_CRITICAL(&s_pending_decision_mux);
    for (auto &pending : s_pending_decisions) {
        if (!pending.active ||
            pending.candidate.address != ack->address ||
            pending.candidate.session_id != ack->session_id ||
            pending.candidate.sequence != ack->sequence) {
            continue;
        }
        if (elapsed_ms) {
            *elapsed_ms = (uint32_t)pdTICKS_TO_MS(now - pending.sent_tick);
        }
        pending.active = false;
        found = true;
        break;
    }
    portEXIT_CRITICAL(&s_pending_decision_mux);
    return found;
}

static bool pending_decision_forget(uint8_t address, uint32_t session_id,
                                    uint32_t sequence)
{
    bool found = false;
    portENTER_CRITICAL(&s_pending_decision_mux);
    for (auto &pending : s_pending_decisions) {
        if (pending.active && pending.candidate.address == address &&
            pending.candidate.session_id == session_id &&
            pending.candidate.sequence == sequence) {
            pending.active = false;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pending_decision_mux);
    return found;
}

static void decision_watchdog_task(void *)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10));
        const TickType_t now = xTaskGetTickCount();
        pending_decision_t expired[kPendingDecisionCount] = {};
        size_t expired_count = 0;

        portENTER_CRITICAL(&s_pending_decision_mux);
        for (auto &pending : s_pending_decisions) {
            if (!pending.active) continue;
            if ((now - pending.sent_tick) >= pdMS_TO_TICKS(PR_HIT_ACK_TIMEOUT_MS)) {
                if (expired_count < kPendingDecisionCount) {
                    expired[expired_count++] = pending;
                }
                pending.active = false;
            }
        }
        portEXIT_CRITICAL(&s_pending_decision_mux);

        for (size_t i = 0; i < expired_count; ++i) {
            const uint32_t elapsed = (uint32_t)pdTICKS_TO_MS(
                now - expired[i].sent_tick);
            ESP_LOGW(TAG, "HIT_ACK_TIMEOUT armour=%u seq=%" PRIu32
                     " elapsed=%" PRIu32 "ms",
                     expired[i].candidate.address + 1U,
                     expired[i].candidate.sequence, elapsed);
            TcpProtocol::controller_send_log(
                PR_CONTROLLER_LOG_WARN,
                "HIT_ACK_TIMEOUT armour=%u seq=%" PRIu32 " elapsed=%" PRIu32 "ms",
                expired[i].candidate.address + 1U,
                expired[i].candidate.sequence, elapsed);
        }
    }
}

static uint8_t random_online_armour(uint8_t online_mask)
{
    const uint8_t online_count = static_cast<uint8_t>(__builtin_popcount(
        static_cast<unsigned>(online_mask & ARMOUR_MASK_ALL)));
    if (online_count == 0) return 0xFF;

    uint8_t selected = static_cast<uint8_t>(esp_random() % online_count);
    for (uint8_t address = 0; address < ARMOUR_COUNT; ++address) {
        if ((online_mask & (1U << address)) == 0) continue;
        if (selected-- == 0) return address;
    }
    return 0xFF;
}

static void randomize_motor_params(void) {
    if (s_ctx.cmd.mode == 0) { // PRA_RUNE_BIG_MODE
        //s_ctx.cur_amp = (esp_random() % 266 + 780) / 1000.0f;
        //s_ctx.cur_omega = (esp_random() % 116 + 1884) / 1000.0f;
        s_ctx.cur_amp    = 1.000f;
        s_ctx.cur_omega  = 1.900f;
        s_ctx.cur_offset = 2.090f - s_ctx.cur_amp;
    } else {
        s_ctx.cur_amp = 0;
        s_ctx.cur_omega = 0;
        s_ctx.cur_offset = 0;
    }
}

// =================== 发送指令系列 ===================

static void send_armour_start(uint8_t addr, uint8_t mode, uint8_t color, uint8_t group)
{
    PRA_START_EVENT_DATA e = {};
    e.address = addr;
    e.data_len = sizeof(PRA_START_EVENT_DATA);
    e.mode = mode;
    e.color = color;
    e.group = group;
    ESP_LOGI(TAG, ">>> START addr=%d mode=%d color=%d group=%d", addr, mode, color, group);
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_START_EVENT, &e, sizeof(e), portMAX_DELAY);
}

static void send_armour_notarget(uint8_t addr, uint8_t group, uint8_t mode, uint8_t color)
{
    PRA_NOTARGET_DATA e = {};
    e.address = addr;
    e.data_len = sizeof(PRA_NOTARGET_DATA);
    e.group = group;
    e.mode = mode;
    e.color = color;
    ESP_LOGD(TAG, ">>> NOTARGET addr=%d group=%d mode=%d color=%d", addr, group, mode, color);
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_NOTARGET_EVENT, &e, sizeof(e), portMAX_DELAY);
}

static void stop_armour_all(void)
{
    ESP_LOGI(TAG, ">>> STOP ALL armours");
    PRA_STOP_EVENT_DATA s = {};
    s.data_len = sizeof(PRA_STOP_EVENT_DATA);
    const uint8_t online_mask = device_registry_armour_mask() & ARMOUR_MASK_ALL;
    for (uint8_t i = 0; i < ARMOUR_COUNT; i++) {
        if ((online_mask & (1U << i)) == 0) continue;
        s.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &s, sizeof(s), portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // FIX(batch): 批量发完后统一等待，让事件循环将5条指令都发出去
    // for (uint8_t i = 0; i < 5; i++) {
    //     s.address = i;
    //     esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &s, sizeof(s), portMAX_DELAY);
    // }
    // vTaskDelay(pdMS_TO_TICKS(20));
}

static void motor_start(void)
{
    if (s_ctx.cmd.dir == PR_RUN_DIR_CS) {
        ESP_LOGI(TAG, ">>> MOTOR START skipped: dir=cs (Motor stationary)");
        return;
    }

    PRM_START_EVENT_DATA m = {};
    m.address = MOTOR;
    m.data_len = sizeof(PRM_START_EVENT_DATA);
    m.mode = (s_ctx.cmd.mode == 0) ? PRA_RUNE_BIG_MODE : PRA_RUNE_SMALL_MODE;
    m.clockwise = s_ctx.cmd.dir;

    if (m.mode == PRA_RUNE_BIG_MODE) {
        m.amplitude = s_ctx.cur_amp;
        m.omega = s_ctx.cur_omega;
        m.offset = s_ctx.cur_offset;
    }
    ESP_LOGI(TAG, ">>> MOTOR START mode=%d dir=%d amp=%.3f omega=%.3f offset=%.3f  [sock=%d]",
             m.mode, m.clockwise, m.amplitude, m.omega, m.offset,
             TcpProtocol::client_sockets[MOTOR]);
    esp_err_t err = esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &m, sizeof(m), portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!! esp_event_post_to FAILED for MOTOR START: %s", esp_err_to_name(err));
    }
}

static void motor_stop(void)
{
    ESP_LOGI(TAG, ">>> MOTOR STOP");
    PRM_STOP_EVENT_DATA m = {};
    m.address = MOTOR;
    m.data_len = sizeof(PRM_STOP_EVENT_DATA);
    esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &m, sizeof(m), portMAX_DELAY);
}

static void motor_unlock(void)
{
    ESP_LOGI(TAG, ">>> MOTOR UNLOCK  [sock=%d]", TcpProtocol::client_sockets[MOTOR]);
    PRM_UNLOCK_EVENT_DATA u = {};
    u.address = MOTOR;
    u.data_len = sizeof(PRM_UNLOCK_EVENT_DATA);
    esp_err_t err = esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, &u, sizeof(u), portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!! esp_event_post_to FAILED for MOTOR UNLOCK: %s", esp_err_to_name(err));
    }
}

static void motor_unlock_if_allowed(void)
{
    if (s_ctx.cmd.dir == PR_RUN_DIR_CS) {
        ESP_LOGI(TAG, ">>> MOTOR UNLOCK skipped: dir=cs (Motor stationary)");
        return;
    }
    motor_unlock();
}

static void calibrate_armour_all(void)
{
    PRA_CALIBRATE_EVENT_DATA e = {};
    e.address = 0xFF;
    e.data_len = sizeof(e);
    ESP_LOGI(TAG, ">>> CALIBRATE ALL armours (discard 0-2.5s, sample 2.5-10.0s)");
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_CALIBRATE_EVENT, &e, sizeof(e), portMAX_DELAY);
}

static void gpa_push_and_publish(uint8_t avg_ring)
{
    s_ctx.gpa_hist.push_back(avg_ring);
    uint8_t out[10] = {0};

    int n = (int)s_ctx.gpa_hist.size();
    for (int i = 0; i < 10; i++) {
        int idx = n - 1 - i;
        out[i] = (idx >= 0) ? s_ctx.gpa_hist[idx] : 0;
    }
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "GAME complete average_ring=%u history_latest=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        avg_ring, out[0], out[1], out[2], out[3], out[4],
        out[5], out[6], out[7], out[8], out[9]);
}

// =================== 核心逻辑处理 ===================

// 清洗队列中遗留的 HIT 消息 (防网络延迟及传感器抖动)
static void flush_stale_hits() {
    qmsg_t msg;
    while (xQueueReceive(s_q, &msg, 0) == pdTRUE) {
        if (msg.type != QMSG_HIT) {
            // 如果误掏出了控制指令，赶紧塞回队列头部
            xQueueSendToFront(s_q, &msg, 0);
            break;
        }
    }
}

// 击打等待逻辑
// 返回：-1 = 激活失败, 0 = 收到外部中断(Stop/Run), 1 = 本阶段通过
static int wait_hits(uint8_t exp_a, uint8_t exp_b, TickType_t to1, TickType_t to2, int* hit_count)
{
    qmsg_t msg;
    TickType_t start_time = xTaskGetTickCount();
    uint8_t hit_a = 0;
    uint8_t hit_b = 0;

    // ----- 阶段 1：等待第一击 (限时 2.5s) -----
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if (now - start_time >= to1) return -1; // 2.5秒超时未打中 -> 激活失败
        TickType_t rem = to1 - (now - start_time);
        
        if (xQueueReceive(s_q, &msg, rem) != pdTRUE) {
            return -1; // 超时 -> 激活失败
        }
        
        if (msg.type == QMSG_STOP) {
            s_ctx.running = false;
            return 0;
        } else if (msg.type == QMSG_RUN) {
            s_ctx.cmd = msg.u.run;
            s_ctx.rerun = true;
            return 0; 
        } else if (msg.type == QMSG_UNLK) {
            motor_unlock_if_allowed();
        } else if (msg.type == QMSG_OTA) {
            OTA_BEGIN_EVENT_DATA d = {};
            d.address = 0x06;
            d.data_len = sizeof(d);
            esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &d, sizeof(d), portMAX_DELAY);
            s_ctx.running = false;
            return 0;
        } else if (msg.type == QMSG_HIT) {
            uint8_t addr = msg.u.hit.addr;
            uint8_t ring = msg.u.hit.ring;
            ESP_LOGI(TAG, "Phase1 HIT: addr=%d ring=%d (exp_a=%d exp_b=%d hit_a=%d hit_b=%d)", addr, ring, exp_a, exp_b, hit_a, hit_b);
            
            if (addr == exp_a && !hit_a) {
                hit_a = 1;
                s_ctx.sum_ring += ring;
                (*hit_count)++;
                break;
            } else if (addr == exp_b && exp_b != 0xFF && !hit_b) {
                hit_b = 1;
                s_ctx.sum_ring += ring;
                (*hit_count)++;
                break;
            } else if (addr != exp_a && addr != exp_b) {
                ESP_LOGW(TAG, "Phase1 WRONG target hit! addr=%d not in {%d,%d}", addr, exp_a, exp_b);
                return -1; 
            }
        }
    }
    
    if (exp_b == 0xFF) return 1;

    // ----- 阶段 2 (仅大符)：等待第二击 (限时 1.0s) -----
    ESP_LOGD(TAG, "Phase2: waiting second hit (1s timeout)");
    start_time = xTaskGetTickCount();
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if (now - start_time >= to2) {
            ESP_LOGD(TAG, "Phase2 timeout, proceeding");
            return 1;
        }
        TickType_t rem = to2 - (now - start_time);
        
        if (xQueueReceive(s_q, &msg, rem) != pdTRUE) {
            return 1;
        }
        
        if (msg.type == QMSG_STOP) {
            s_ctx.running = false;
            return 0;
        } else if (msg.type == QMSG_RUN) {
            s_ctx.cmd = msg.u.run;
            s_ctx.rerun = true;
            return 0; 
        } else if (msg.type == QMSG_UNLK) {
            motor_unlock_if_allowed();
        } else if (msg.type == QMSG_OTA) {
            OTA_BEGIN_EVENT_DATA d = {};
            d.address = 0x06;
            d.data_len = sizeof(d);
            esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &d, sizeof(d), portMAX_DELAY);
            s_ctx.running = false;
            return 0;
        } else if (msg.type == QMSG_HIT) {
            uint8_t addr = msg.u.hit.addr;
            uint8_t ring = msg.u.hit.ring;
            ESP_LOGI(TAG, "Phase2 HIT: addr=%d ring=%d (exp_a=%d exp_b=%d hit_a=%d hit_b=%d)", addr, ring, exp_a, exp_b, hit_a, hit_b);
            
            if (addr == exp_a && !hit_a) {
                hit_a = 1;
                s_ctx.sum_ring += ring;
                (*hit_count)++;
                break;
            } else if (addr == exp_b && !hit_b) {
                hit_b = 1;
                s_ctx.sum_ring += ring;
                (*hit_count)++;
                break; 
            } else if (addr != exp_a && addr != exp_b) {
                ESP_LOGW(TAG, "Phase2 WRONG target hit! addr=%d not in {%d,%d}", addr, exp_a, exp_b);
                return -1; 
            }
        }
    }
    
    return 1;
}

static void run_without_armour(void)
{
    const bool motor_stationary = s_ctx.cmd.dir == PR_RUN_DIR_CS;
    ESP_LOGW(TAG, "No Armour online: %s and skip target rounds",
             motor_stationary ? "keep Motor stationary" : "keep Motor running");
    TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                     motor_stationary
                                         ? "No Armour online; Motor stationary without target rounds"
                                         : "No Armour online; Motor runs without target rounds");

    while (s_ctx.running) {
        qmsg_t msg;
        if (xQueueReceive(s_q, &msg, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        if (msg.type == QMSG_STOP) {
            s_ctx.running = false;
        } else if (msg.type == QMSG_RUN) {
            s_ctx.cmd = msg.u.run;
            s_ctx.rerun = true;
            s_ctx.running = false;
        } else if (msg.type == QMSG_UNLK) {
            motor_unlock_if_allowed();
        } else if (msg.type == QMSG_OTA) {
            OTA_BEGIN_EVENT_DATA d = {};
            d.address = 0x06;
            d.data_len = sizeof(d);
            esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &d, sizeof(d), portMAX_DELAY);
            s_ctx.running = false;
        }
    }
}

static void run_once(void)
{
    stop_armour_all();

    s_ctx.sum_ring = 0;
    int hit_count = 0;

    const uint8_t online_mask = device_registry_armour_mask() & ARMOUR_MASK_ALL;
    const uint8_t online_count = static_cast<uint8_t>(__builtin_popcount(
        static_cast<unsigned>(online_mask)));
    if (online_count == 0) {
        run_without_armour();
        return;
    }

    uint8_t available_targets[ARMOUR_COUNT] = {};
    uint8_t available_count = 0;
    for (uint8_t i = 0; i < ARMOUR_COUNT; ++i) {
        if (online_mask & (1U << i)) available_targets[available_count++] = i;
    }

    for (uint8_t group = 1; group <= 5; group++) {
        if (!s_ctx.running) break;

        flush_stale_hits();

        uint8_t a = 0xFF;
        uint8_t b = 0xFF;
        
        uint8_t mode  = (s_ctx.cmd.mode == 0) ? PRA_RUNE_BIG_MODE : PRA_RUNE_SMALL_MODE;
        uint8_t color = s_ctx.cmd.color;

        if (s_ctx.cmd.mode == 0) {
            a = random_online_armour(online_mask);
            if (online_count >= 2) {
                do { b = random_online_armour(online_mask); } while (b == a);
            }
        } else {
            if (available_count == 0) {
                for (uint8_t i = 0; i < ARMOUR_COUNT; ++i) {
                    if (online_mask & (1U << i)) available_targets[available_count++] = i;
                }
            }
            uint8_t idx = esp_random() % available_count;
            a = available_targets[idx];
            available_targets[idx] = available_targets[available_count - 1];
            available_count--;
        }

        ESP_LOGI(TAG, "===== Group %d/%d: target_a=%d target_b=%d mode=%s color=%d =====",
                 group, 5, a, (b == 0xFF) ? -1 : b,
                 (mode == PRA_RUNE_BIG_MODE) ? "BIG" : "SMALL", color);
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_DEBUG,
            "GROUP %u targets armour_a=%u armour_b=%d mode=%s color=%u",
            group, a + 1, (b == 0xFF) ? 0 : (int)b + 1,
            (mode == PRA_RUNE_BIG_MODE) ? "BIG" : "SMALL", color);

        for (uint8_t i = 0; i < ARMOUR_COUNT; i++) {
            if ((online_mask & (1U << i)) == 0) continue;
            if (i == a || i == b) continue;
            send_armour_notarget(i, group, mode, color);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        send_armour_start(a, mode, color, group);
        vTaskDelay(pdMS_TO_TICKS(5));
        if (b != 0xFF) {
            send_armour_start(b, mode, color, group);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        // FIX(batch): 先批量发非目标指令，再批量发目标指令，不在每条之间加延迟
        // for (uint8_t i = 0; i < 5; i++) {
        //     if (i == a || i == b) continue;
        //     send_armour_notarget(i, group, mode, color);
        // }
        // send_armour_start(a, mode, color, group);
        // if (b != 0xFF) {
        //     send_armour_start(b, mode, color, group);
        // }
        // vTaskDelay(pdMS_TO_TICKS(20)); // 全部指令批量入队后，等待事件循环将所有指令发出

        set_hit_context(true, a, b);
        int r = wait_hits(a, b, pdMS_TO_TICKS(2500), pdMS_TO_TICKS(1000), &hit_count);
        set_hit_context(false, 0xFF, 0xFF);
        ESP_LOGI(TAG, "Group %d result: r=%d (1=pass, 0=interrupted, -1=fail) hits=%d", group, r, hit_count);
        TcpProtocol::controller_send_log(
            r < 0 ? PR_CONTROLLER_LOG_WARN : PR_CONTROLLER_LOG_INFO,
            "GROUP %u result=%d hits=%d", group, r, hit_count);
        
        if (r == 0) {
            stop_armour_all();
            return;
        }
        if (r < 0) {
            ESP_LOGW(TAG, "Group %d FAILED, resetting groups (motor keeps running)", group);
            TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                             "GROUP %u failed; resetting", group);
            stop_armour_all();
            return;
        }
    }

    if (s_ctx.running) {
        uint8_t avg = hit_count > 0 ? (uint8_t)((s_ctx.sum_ring + (hit_count / 2)) / hit_count) : 0;
        gpa_push_and_publish(avg);

        PRA_COMPLETE_EVENT_DATA c = {};
        c.data_len = sizeof(PRA_COMPLETE_EVENT_DATA);
        for (uint8_t i = 0; i < ARMOUR_COUNT; i++) {
            if ((online_mask & (1U << i)) == 0) continue;
            c.address = i;
            esp_event_post_to(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, &c, sizeof(c), portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    stop_armour_all();
}


// // 限时 5 秒等待击打，打中过关，不打 5 秒后自动过关
// static int wait_hits(TickType_t to, int* hit_count) {
//     qmsg_t msg;
//     TickType_t start_time = xTaskGetTickCount();
//     while (1) {
//         TickType_t now = xTaskGetTickCount();
//         if (now - start_time >= to) return 1; // 5秒超时，自动放行看下一关动画！
        
//         TickType_t rem = to - (now - start_time);
//         if (xQueueReceive(s_q, &msg, rem) != pdTRUE) return 1; // 超时自动放行
        
//         if (msg.type == QMSG_STOP) { s_ctx.running = false; return 0; }
//         else if (msg.type == QMSG_RUN) { s_ctx.cmd = msg.u.run; return 0; }
//         else if (msg.type == QMSG_UNLK) motor_unlock();
//         else if (msg.type == QMSG_HIT) {
//             // 【核心】：不校验到底是哪个板子发来的，只要收到 HIT，统统算过关！
//             s_ctx.sum_ring += msg.u.hit.ring;
//             (*hit_count)++;
//             return 1; 
//         }
//     }
// }

// static void run_once(void) {
//     stop_armour_all();
//     motor_start(&s_ctx.cmd);
//     s_ctx.sum_ring = 0; int hit_count = 0;

//     for (uint8_t group = 1; group <= 5; group++) {
//         if (!s_ctx.running) break;
//         flush_stale_hits();

//         uint8_t mode = (s_ctx.cmd.mode == 0) ? PRA_RUNE_BIG_MODE : PRA_RUNE_SMALL_MODE;
//         uint8_t color = s_ctx.cmd.color;

//         // 【核心】：获取当前活着的板子掩码
//         uint8_t mask = device_registry_armour_mask();
        
//         // 【核心】：海王模式，把所有活着的���子全部作为 Target 点亮！
//         // 这样你那块满配的板子绝对会亮起来。
//         for (uint8_t i = 0; i < 5; i++) {
//             if (mask & (1 << i)) {
//                 send_armour_start(i, mode, color, group);
//             }
//         }

//         // 等待击打，限时5秒
//         int r = wait_hits(pdMS_TO_TICKS(5000), &hit_count);
//         if (r <= 0) { motor_stop(); stop_armour_all(); return; }
//     }

//     if (s_ctx.running) {
//         uint8_t avg = hit_count > 0 ? (uint8_t)((s_ctx.sum_ring + (hit_count / 2)) / hit_count) : 0;
//         gpa_push_and_publish(avg);

//         PRA_COMPLETE_EVENT_DATA c = { .data_len = sizeof(PRA_COMPLETE_EVENT_DATA) };
//         for (uint8_t i = 0; i < 5; i++) {
//             c.address = i;
//             esp_event_post_to(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, &c, sizeof(c), portMAX_DELAY);
//         }
//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
//     stop_armour_all();
//     motor_stop();
// }
static void mech_task(void*)
{
    while (1) {
        qmsg_t msg;
        xQueueReceive(s_q, &msg, portMAX_DELAY);

        if (msg.type == QMSG_RUN) {
            s_ctx.cmd = msg.u.run;
            s_ctx.running = true;
            s_ctx.rerun = false;

            if (led_strip) {
                if (s_ctx.cmd.color == 0) {
                    led_strip->set_color(50, 0, 0);
                    ESP_LOGI(TAG, "led_strip set RED (50,0,0), ptr=%p", led_strip);
                } else {
                    led_strip->set_color(0, 0, 50);
                    ESP_LOGI(TAG, "led_strip set BLUE (0,0,50), ptr=%p", led_strip);
                }
                led_strip->refresh();
                ESP_LOGI(TAG, "led_strip refresh done");
                TcpProtocol::controller_send_log(
                    PR_CONTROLLER_LOG_INFO, "LED strip=%s",
                    s_ctx.cmd.color == 0 ? "RED" : "BLUE");
            } else {
                ESP_LOGW(TAG, "led_strip is NULL, skipping color set!");
            }

            randomize_motor_params();
            motor_unlock_if_allowed();
            vTaskDelay(pdMS_TO_TICKS(100));

            bool require_startup_calibration = true;
            do {
                s_ctx.rerun = false;
                motor_start(); // 安全幂等发包（新开局、断连重试、换局更新）
                if (require_startup_calibration) {
                    calibrate_armour_all();
                    // 标定期间不发送任何START：前2.5秒舍弃，后7.5秒仅采样，第10秒后才进入游戏。
                    vTaskDelay(pdMS_TO_TICKS(10000));
                    require_startup_calibration = false;
                }

                run_once();

                if (s_ctx.rerun) {
                    require_startup_calibration = true;
                    randomize_motor_params(); // 用户手动重发指令，重新变幻速度
                    if (led_strip) {
                        if (s_ctx.cmd.color == 0) {
                            led_strip->set_color(50, 0, 0);
                            ESP_LOGI(TAG, "led_strip set_color to red");
                        } else {
                            led_strip->set_color(0, 0, 50);
                            ESP_LOGI(TAG, "led_strip set_color to blue");
                        }
                        led_strip->refresh();
                    }
                    motor_unlock_if_allowed();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    // motor_start() 将在下次 do-while 循环首部执行
                } else if (s_ctx.running && s_ctx.cmd.loop) {
                    randomize_motor_params(); // 重新开始下一局，变换循环速度
                }
            } while (s_ctx.running && (s_ctx.rerun || s_ctx.cmd.loop));

            motor_stop();
            stop_armour_all();
            s_ctx.running = false;
            if (led_strip) {
                led_strip->clear_pixels();
                led_strip->refresh();
                TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_INFO,
                                                 "LED strip=OFF");
            }
        } else if (msg.type == QMSG_STOP) {
            s_ctx.running = false;
            stop_armour_all();
            motor_stop();
            if (led_strip) {
                led_strip->clear_pixels();
                led_strip->refresh();
            }
        } else if (msg.type == QMSG_UNLK) {
            motor_unlock_if_allowed();
        } else if (msg.type == QMSG_OTA) {
            OTA_BEGIN_EVENT_DATA d = {};
            d.address = 0x06;
            d.data_len = sizeof(d);
            esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &d, sizeof(d), portMAX_DELAY);
        }
    }
}

// =================== 事件监听桥接 ===================

static void push_msg(const qmsg_t* m)
{
    xQueueSend(s_q, m, 0);
}

static void on_run(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* cmd = (pr_run_cmd_t*)event_data;
    if (!cmd || cmd->dir > PR_RUN_DIR_CS) {
        ESP_LOGW(TAG, "<<< RUN rejected: invalid direction=%u", cmd ? cmd->dir : 0xFFU);
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                         "RUN rejected: direction must be cw, ccw or cs");
        return;
    }
    if (!device_registry_all_ready()) {
        ESP_LOGW(TAG, "<<< RUN rejected: device set is not healthy");
        TcpProtocol::controller_send_log(PR_CONTROLLER_LOG_WARN,
                                         "RUN rejected: motor or device link not ready");
        return;
    }
    ESP_LOGI(TAG, "<<< RUN cmd: color=%d mode=%d loop=%d dir=%d", cmd->color, cmd->mode, cmd->loop, cmd->dir);
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_INFO,
        "RUN color=%u mode=%u loop=%u dir=%u",
        cmd->color, cmd->mode, cmd->loop, cmd->dir);
    if (cmd->dir == PR_RUN_DIR_CS) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_INFO,
            "RUN dir=CS; Motor remains stationary");
    }
    qmsg_t m = {};
    m.type = QMSG_RUN;
    m.u.run = *cmd;
    push_msg(&m);
}

static void on_stop(void*, esp_event_base_t, int32_t, void*)
{
    ESP_LOGI(TAG, "<<< STOP cmd");
    qmsg_t m = {};
    m.type = QMSG_STOP;
    push_msg(&m);
}

static void on_unlk(void*, esp_event_base_t, int32_t, void*)
{
    ESP_LOGI(TAG, "<<< UNLOCK cmd");
    qmsg_t m = {};
    m.type = QMSG_UNLK;
    push_msg(&m);
}

static void on_ota(void*, esp_event_base_t, int32_t, void*)
{
    ESP_LOGI(TAG, "<<< OTA cmd");
    qmsg_t m = {};
    m.type = QMSG_OTA;
    push_msg(&m);
}

static void send_hit_decision(const PRA_HIT_EVENT_DATA *candidate,
                              bool accepted, uint8_t reason,
                              bool threshold_enabled, uint16_t threshold_x100)
{
    PRA_HIT_DECISION_EVENT_DATA decision = {};
    decision.address = candidate->address;
    decision.data_len = sizeof(decision);
    decision.session_id = candidate->session_id;
    decision.sequence = candidate->sequence;
    decision.score = candidate->ring;
    decision.accepted = accepted ? 1 : 0;
    decision.reason = reason;
    decision.threshold_enabled = threshold_enabled ? 1 : 0;
    decision.threshold_x100 = threshold_x100;
    decision.decided_ms = esp_log_timestamp();

    if (!pending_decision_add(candidate)) {
        ESP_LOGE(TAG, "decision pending table full: armour=%u seq=%" PRIu32,
                 candidate->address + 1U, candidate->sequence);
    }
    const esp_err_t err = esp_event_post_to(
        pr_events_loop_handle, PRA, PRA_HIT_DECISION_EVENT,
        &decision, sizeof(decision), 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to post hit decision: %s", esp_err_to_name(err));
    }
}

static void on_hit(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* ev = (PRA_HIT_EVENT_DATA*)event_data;
    if (!ev || ev->data_len < sizeof(PRA_HIT_EVENT_DATA) ||
        ev->address >= ARMOUR_COUNT) {
        ESP_LOGW(TAG, "<<< invalid HIT candidate");
        return;
    }

    bool game_active = false;
    uint8_t expected_a = 0xFF;
    uint8_t expected_b = 0xFF;
    get_hit_context(&game_active, &expected_a, &expected_b);

    bool threshold_enabled = false;
    uint16_t threshold_x100 = 0;
    hit_filter_get(ev->address, &threshold_enabled, &threshold_x100);

    const bool threshold_pass = !threshold_enabled ||
                                ev->peak_x100 >= threshold_x100;
    bool accepted = false;
    uint8_t reason = PR_HIT_DECISION_INVALID;
    if (ev->ring == 0) {
        reason = PR_HIT_DECISION_AMBIGUOUS;
    } else if (!threshold_pass) {
        reason = PR_HIT_DECISION_BELOW_THRESHOLD;
    } else if (!game_active) {
        reason = PR_HIT_DECISION_NO_GAME;
    } else if (ev->address != expected_a && ev->address != expected_b) {
        reason = PR_HIT_DECISION_WRONG_TARGET;
    } else {
        accepted = true;
        reason = PR_HIT_DECISION_ACCEPT;
    }

    send_hit_decision(ev, accepted, reason, threshold_enabled, threshold_x100);

    if (controller_gateway_debug_enabled()) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_DEBUG,
            "HIT candidate armour=%u ring=%u peak=%u.%02u game=%u target=%u,%u threshold=%s:%u.%02u result=%s seq=%" PRIu32,
            ev->address + 1U, ev->ring, ev->peak_x100 / 100U,
            ev->peak_x100 % 100U, game_active ? 1U : 0U,
            expected_a < ARMOUR_COUNT ? expected_a + 1U : 0U,
            expected_b < ARMOUR_COUNT ? expected_b + 1U : 0U,
            threshold_enabled ? "ON" : "OFF", threshold_x100 / 100U,
            threshold_x100 % 100U, decision_reason_name(reason), ev->sequence);
    }

    if (accepted) {
        ESP_LOGI(TAG, "<<< HIT ACCEPTED: armour=%u ring=%u peak=%u.%02u seq=%" PRIu32,
                 ev->address + 1U, ev->ring, ev->peak_x100 / 100U,
                 ev->peak_x100 % 100U, ev->sequence);
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_INFO,
            "HIT armour=%u ring=%u peak=%u.%02u seq=%" PRIu32,
            ev->address + 1U, ev->ring, ev->peak_x100 / 100U,
            ev->peak_x100 % 100U, ev->sequence);
    } else if (reason == PR_HIT_DECISION_WRONG_TARGET) {
        ESP_LOGW(TAG, "<<< HIT rejected: wrong target armour=%u ring=%u peak=%u.%02u",
                 ev->address + 1U, ev->ring, ev->peak_x100 / 100U,
                 ev->peak_x100 % 100U);
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_WARN,
            "HIT rejected wrong target armour=%u ring=%u peak=%u.%02u",
            ev->address + 1U, ev->ring, ev->peak_x100 / 100U,
            ev->peak_x100 % 100U);
    }

    // Preserve the original game behavior for a sufficiently strong hit on
    // the wrong target: wait_hits() will fail the current group.  Candidates
    // below the second threshold never enter the game queue.
    if (accepted || reason == PR_HIT_DECISION_WRONG_TARGET) {
        qmsg_t m = {};
        m.type = QMSG_HIT;
        m.u.hit.addr = ev->address;
        m.u.hit.ring = ev->ring;
        push_msg(&m);
    }
}

static void on_hit_decision_ack(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* ack = (PRA_HIT_DECISION_ACK_EVENT_DATA*)event_data;
    if (!ack) return;
    uint32_t elapsed_ms = 0;
    const bool found = pending_decision_ack(ack, &elapsed_ms);
    if (!found) {
        ESP_LOGD(TAG, "late/duplicate HIT decision ACK: armour=%u seq=%" PRIu32,
                 ack->address + 1U, ack->sequence);
        return;
    }
    if (controller_gateway_debug_enabled()) {
        TcpProtocol::controller_send_log(
            PR_CONTROLLER_LOG_DEBUG,
            "HIT decision ACK armour=%u seq=%" PRIu32 " elapsed=%" PRIu32
            "ms received=%u applied=%u",
            ack->address + 1U, ack->sequence, elapsed_ms,
            ack->received, ack->applied);
    }
}

static void on_hit_timeout(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* timeout = (PRA_HIT_TIMEOUT_EVENT_DATA*)event_data;
    if (!timeout || timeout->address >= ARMOUR_COUNT) return;
    if (!pending_decision_forget(timeout->address, timeout->session_id,
                                 timeout->sequence)) {
        ESP_LOGD(TAG, "duplicate HIT_ACK_TIMEOUT from armour=%u seq=%" PRIu32,
                 timeout->address + 1U, timeout->sequence);
        return;
    }
    ESP_LOGW(TAG, "Armour reported HIT_ACK_TIMEOUT: armour=%u seq=%" PRIu32
             " elapsed=%" PRIu32 "ms",
             timeout->address + 1U, timeout->sequence, timeout->elapsed_ms);
    TcpProtocol::controller_send_log(
        PR_CONTROLLER_LOG_WARN,
        "HIT_ACK_TIMEOUT armour=%u seq=%" PRIu32 " elapsed=%" PRIu32 "ms",
        timeout->address + 1U, timeout->sequence, timeout->elapsed_ms);
}

void mech_engine_init(void)
{
    s_ctx = mech_ctx_t{};
    s_ctx.gpa_hist.reserve(32);
    for (auto &pending : s_pending_decisions) {
        pending = pending_decision_t{};
    }
    s_q = xQueueCreate(32, sizeof(qmsg_t));
    xTaskCreate(mech_task, "mech_task", 6144, nullptr, 10, nullptr);
    xTaskCreate(decision_watchdog_task, "hit_decision_watch", 4096,
                nullptr, 4, &s_decision_watchdog_task);
}

void mech_engine_start(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRAPP, PRAPP_RUN_CMD_EVENT,  on_run, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRAPP, PRAPP_STOP_CMD_EVENT, on_stop, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRAPP, PRAPP_UNLK_CMD_EVENT, on_unlk, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRAPP, PRAPP_OTA_CMD_EVENT,  on_ota, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, on_hit, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_DECISION_ACK_EVENT, on_hit_decision_ack, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_TIMEOUT_EVENT, on_hit_timeout, nullptr));
}
