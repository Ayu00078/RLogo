/**
 * @file test_sensor_threshold.cpp
 * @brief 传感器阈值调试模式（独立运行，无需 WiFi / TCP）
 *
 * ── 使用方法 ──────────────────────────────────────────────────────
 *   在 main/CMakeLists.txt 中：
 *     1. 注释掉正式固件那组 SRCS（app_main.cpp / armour_app.cpp 等）
 *     2. 取消注释下方「阈值调试模式」那组（含 led_controller.cpp）
 *   重新编译烧录即可。
 *
 * ── 灯效说明 ──────────────────────────────────────────────────────
 *   待机：灯板持续显示红色瞄准靶（show_target）
 *   命中：闪烁 5 次成功动画（show_success），之后恢复待机
 *   全程无需 WiFi / TCP，纯本地 LED 反馈。
 *
 * ── 调参流程 ──────────────────────────────────────────────────────
 *   1. 修改下方 PARAM_* 宏
 *   2. idf.py build flash monitor
 *   3. 用子弹打击传感器，观察串口输出：
 *      - "HIT" 行：确认命中（含 total_weight）
 *      - "STATS" 行：每 5 s 打印命中次数，判断漏检 / 误触率
 *      - "RAW" 行：每 DIAG_RAW_PRINT_INTERVAL 帧打印原始 ADC，
 *                  看静止时各通道数值，决定 hit_threshold 下限
 * ──────────────────────────────────────────────────────────────────
 */

// ================================================================
//  ★★★  调参入口 — 只需修改这里，其他不用动  ★★★
// ================================================================

/** 单通道触发门槛（winner-take-all 算法的唯一判定阈值）
 *  建议范围：10 ~ 40
 *  - 调低 → 更灵敏，但静止噪声也容易触发
 *  - 调高 → 更稳定，但弱击容易漏检
 *  实战中：小符模式 (静止) 用 25 左右；大符模式 (电机震动) 用 35~45 */
#define PARAM_HIT_THRESHOLD     17.5f

/** EMA 基线衰减系数
 *  - 小符模式 (静止) 推荐 0.995（高灵敏）
 *  - 大符模式 (电机震动) 推荐 0.97（基线跟得快，扛震） */
#define PARAM_EMA_ALPHA         0.995f

/** 命中后防抖时间 (ms)：必须持续安静 ≥ 此值才重新武装
 *  - 防止物理回弹/振铃产生的相邻峰被误判成第二次击中
 *  - 推荐 800ms */
#define PARAM_REFRACTORY_MS     800

/** 传感器通道数，对应压感板，勿改 */
#define PARAM_CHANNEL_COUNT     10

/** 每隔多少帧打印一次原始 ADC（0 = 关闭）
 *  开启后会大幅增加串口输出量，调完阈值后建议关闭 */
#define DIAG_RAW_PRINT_INTERVAL 300

/** 统计周期（ms）：每隔多久打印一次命中统计 */
#define DIAG_STATS_INTERVAL_MS  5000

// ================================================================
//  以下不需要修改
// ================================================================

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_timer.h"

#include "firmware.h"          // Config, pr_events_loop_handle, PRA event base
#include "PowerRune_Events.h"
#include "sensor_processor.h"
#include "uart_handler.h"
#include "led_controller.h"    // init_led, show_target, show_success, led_* handles

static const char* TAG = "thresh_test";

// ── LED 任务句柄（用于从 on_hit_event 发通知）─────────────────────
static TaskHandle_t s_led_task_handle = nullptr;

// ── LED 后台任务 ──────────────────────────────────────────────────
// 正常显示红色瞄准靶；收到命中通知后闪烁成功动画，再恢复待机。
static void led_task(void*)
{
    // 上电先等 LED 驱动稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        // 待机：持续刷新瞄准靶（100 ms 刷一次，阻塞等待命中通知）
        show_target(PR_RED);
        show_arrow(PR_RED);

        uint32_t score = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &score, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 命中动画：闪烁 5 次
            ESP_LOGI(TAG, "[LED] HIT ring=%d -> blink", (int)score);
            for (int i = 0; i < 5; i++) {
                led_strip_clear(led_board);
                led_strip_clear(led_arm);
                led_strip_clear(led_strip);
                led_strip_refresh(led_board);
                led_strip_refresh(led_arm);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(80));

                show_success(PR_RED);
                vTaskDelay(pdMS_TO_TICKS(80));

                // 若中途又有新命中通知，清掉避免动画叠加
                xTaskNotifyWait(0, ULONG_MAX, nullptr, 0);
            }
            // 动画结束，立即回到待机（下一轮循环刷新 show_target）
        }
    }
}

// ── 命中统计（原子操作用 portMUX） ──────────────────────────────
static volatile uint32_t s_hit_count_total = 0;
static volatile uint32_t s_hit_count_period = 0;
static portMUX_TYPE s_hit_mux = portMUX_INITIALIZER_UNLOCKED;

// ── 帧计数（用于 RAW 诊断，只在回调里写，无需加锁） ────────────
static uint32_t s_frame_count = 0;

// ── 命中事件监听器 ────────────────────────────────────────────────
static void on_hit_event(void*, esp_event_base_t, int32_t, void* event_data)
{
    auto* d = (PRA_HIT_EVENT_DATA*)event_data;

    portENTER_CRITICAL(&s_hit_mux);
    s_hit_count_total++;
    s_hit_count_period++;
    uint32_t total = s_hit_count_total;
    portEXIT_CRITICAL(&s_hit_mux);

    ESP_LOGI(TAG, ">>> HIT  ring=%d  addr=%d  (total=%" PRIu32 ")",
             d->ring, d->address, total);

    // 通知 LED 任务播放命中动画
    if (s_led_task_handle) {
        xTaskNotify(s_led_task_handle, (uint32_t)d->ring, eSetValueWithOverwrite);
    }
}

// ── 周期统计任务 ──────────────────────────────────────────────────
static void stats_task(void*)
{
    const TickType_t period = pdMS_TO_TICKS(DIAG_STATS_INTERVAL_MS);
    while (1) {
        vTaskDelay(period);

        portENTER_CRITICAL(&s_hit_mux);
        uint32_t period_hits = s_hit_count_period;
        uint32_t total_hits  = s_hit_count_total;
        s_hit_count_period = 0;
        portEXIT_CRITICAL(&s_hit_mux);

        ESP_LOGI(TAG,
                 "STATS  last_%ds=%" PRIu32 "  total=%" PRIu32
                 "  |  hit_thresh=%.1f  refrac=%dms",
                 DIAG_STATS_INTERVAL_MS / 1000,
                 period_hits, total_hits,
                 (float)PARAM_HIT_THRESHOLD,
                 PARAM_REFRACTORY_MS);
    }
}

// ── 原始 ADC 诊断包装回调 ─────────────────────────────────────────
// uart_handler 直接调用 sensor_processor_input_cb；这里把它包一层，
// 利用 weak 符号机制：只要本文件定义了同名强符号，链接器就会优先用本文件。
//
// 注意：本文件 *不* 再 include sensor_processor.cpp，
//       而是通过 extern 声明直接调原始函数 ——
//       为避免符号冲突，改用另一种方案：在 uart_handler.cpp 里，
//       原来写死调用 sensor_processor_input_cb 的地方不变，
//       我们在这里也不覆盖，而是在 sensor_processor.cpp 里
//       加 LOGI 就够了（已经有了）。
//       如果想看 RAW 数据，靠下面单独的诊断任务 + 串口打印即可。
//
// ── 实际上 sensor_processor_input_cb 已经打印了 HIT 行，
//    如果要看"近漏"原始值，把 DIAG_RAW_PRINT_INTERVAL 设为非零即可，
//    下面的包装函数会替代 sensor_processor.cpp 里的同名函数。
// ─────────────────────────────────────────────────────────────────

// 声明原始处理函数（来自 sensor_processor.cpp）
// 由于我们要包装它，需要用不同的名字 ── 但 uart_handler 写死了调用
// sensor_processor_input_cb，所以我们直接在这里重定义它（本文件
// 不再编译 sensor_processor.cpp 里该符号，而是整个替换）。
//
// ★ 因此 CMakeLists 测试模式里需要 **同时包含** sensor_processor.cpp，
//   编译器会报重定义错误。解决方法：把诊断逻辑内联在这里，
//   sensor_processor.cpp 的内部逻辑在测试时通过 set_params 传入即可。
// ─────────────────────────────────────────────────────────────────
// 结论：保持干净，不重定义 sensor_processor_input_cb。
//      RAW 诊断靠下面的独立诊断任务，通过读取一个全局"最近帧"缓冲区实现。

// 全局"最近一帧" ADC 缓冲（由 UART 任务写入，诊断任务读取）
static uint8_t  s_latest_adc[PARAM_CHANNEL_COUNT];
static bool     s_latest_valid = false;
static portMUX_TYPE s_adc_mux = portMUX_INITIALIZER_UNLOCKED;

// 替换 sensor_processor_input_cb：先缓存原始值，再调用原始处理链
// 为了不引入名字冲突，我们采用"注入"方式：
//   uart_handler 调用 sensor_processor_input_cb，
//   我们在本文件里提供这个符号（sensor_processor.cpp 就不编译了）。
//
// ★ 所以测试模式 CMakeLists 里【不包含 sensor_processor.cpp】，
//   但包含本文件。本文件在内部把阈值逻辑重新实现一遍，
//   这样可以加任意诊断而不影响正式固件。

// ── 从 sensor_processor.cpp 搬过来的核心逻辑（精简版，加了诊断）──

#include "esp_timer.h"
#include <stdlib.h>

static size_t   g_ch         = PARAM_CHANNEL_COUNT;
static float    g_alpha      = PARAM_EMA_ALPHA;
static float    g_hit_thresh = PARAM_HIT_THRESHOLD;
static uint32_t g_refrac     = PARAM_REFRACTORY_MS;

static float    g_baseline[PARAM_CHANNEL_COUNT]  = {};
// 最后一次"活跃帧"（max_diff 高于释放阈值）的时间戳；
// 必须从这一刻起连续 refractory_ms 都安静才允许重新武装。
// 比"瞬时低值即武装"更稳——能扛住物理回弹/振铃造成的高低震荡。
static uint32_t g_last_active_ts = 0;
// 是否已武装：每次命中后置 false，须经过持续安静期才能恢复 true
static bool     g_armed = true;
static bool     g_sp_inited = false;

static inline uint32_t now_ms_local()
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline uint8_t ch_to_ring(uint8_t ch)
{
    static const uint8_t map10[10] = {6, 7, 8, 9, 10, 1, 2, 3, 4, 5};
    return (ch < 10) ? map10[ch] : (uint8_t)(ch + 1);
}

// uart_handler 调用的入口（替代 sensor_processor.cpp 里的同名函数）
extern "C"
void sensor_processor_input_cb(const uint8_t* adc, size_t adc_len, void* /*ctx*/)
{
    if (adc == NULL) return;
    if (!g_sp_inited) {
        memset(g_baseline, 0, sizeof(g_baseline));
        g_last_active_ts = 0;
        g_armed = true;
        g_sp_inited = true;
    }

    size_t n = (adc_len < g_ch) ? adc_len : g_ch;
    if (n == 0) return;

    s_frame_count++;

    // 1) 更新基线 & 差值
    float diffs[PARAM_CHANNEL_COUNT];
    for (size_t i = 0; i < n; ++i) {
        float s = (float)adc[i];
        if (g_baseline[i] == 0.0f) g_baseline[i] = s;
        g_baseline[i] = g_alpha * g_baseline[i] + (1.0f - g_alpha) * s;
        diffs[i] = s - g_baseline[i];
    }

    // 预热：前 2000 帧只建立基线，不判断命中（避免上电瞬态误触）
    if (s_frame_count < 2000) return;

    // 2) 找峰值通道（winner-take-all）
    float max_diff = 0.0f;
    int   max_ch   = -1;
    for (size_t i = 0; i < n; ++i) {
        if (diffs[i] > max_diff) {
            max_diff = diffs[i];
            max_ch   = (int)i;
        }
    }

    // ── 诊断：定期打印原始 ADC + 当前峰值 ────────────────
#if DIAG_RAW_PRINT_INTERVAL > 0
    if (s_frame_count % DIAG_RAW_PRINT_INTERVAL == 0) {
        char buf[128];
        int  pos = 0;
        for (size_t i = 0; i < n && pos < 100; ++i) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%3d ", adc[i]);
        }
        ESP_LOGI(TAG, "RAW [frame=%" PRIu32 "] %s | max=%.1f ch=%d",
                 s_frame_count, buf, max_diff, max_ch);
    }
#endif

    // 3) 防抖：用"持续安静期"判定释放
    // 释放阈值（hysteresis）：低于此值的帧视为"安静帧"
    // 必须连续 refractory_ms 全部安静，才允许重新武装
    // ——这样物理回弹/振铃的高低震荡（即便瞬间低于阈值一半）也不会过早解锁
    uint32_t now = now_ms_local();
    const float release_threshold = g_hit_thresh * 0.5f;

    if (max_diff >= release_threshold) {
        g_last_active_ts = now;
    }

    if (!g_armed && g_last_active_ts != 0 &&
        (now - g_last_active_ts) >= g_refrac) {
        g_armed = true;
    }

    if (max_ch < 0 || max_diff < g_hit_thresh) return;
    if (!g_armed) return;

    // 4) 环号 = 峰值通道映射
    int ring = (int)ch_to_ring((uint8_t)max_ch);
    if (ring < 1)           ring = 1;
    if (ring > (int)g_ch)   ring = (int)g_ch;

    // 5) 触发
    ESP_LOGI(TAG, "HIT: ring=%d  ch=%d  max=%.1f  (thresh=%.1f)",
             ring, max_ch, max_diff, g_hit_thresh);

    PRA_HIT_EVENT_DATA evt = {};
    evt.address = (config && config->get_config_info_pt())
                  ? (config->get_config_info_pt()->armour_id - 1)
                  : 0;
    evt.score   = (uint8_t)ring;
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_HIT_EVENT,
                      &evt, sizeof(evt), portMAX_DELAY);

    // 立即解除武装；g_last_active_ts 上面已经更新（max_diff > s_thresh > release_threshold）
    g_armed = false;
}

// sensor_processor_init / set_params / reset / get_stats 的空壳
// （uart_handler 不调用这些，但其他模块可能声明依赖，提供弱符号即可）
extern "C" void sensor_processor_init(void) {}
extern "C" void sensor_processor_reset(void) {
    memset(g_baseline, 0, sizeof(g_baseline));
    g_last_active_ts = 0;
    g_armed = true;
}
extern "C" void sensor_processor_get_stats(uint32_t* hits) {
    if (hits) {
        portENTER_CRITICAL(&s_hit_mux);
        *hits = s_hit_count_total;
        portEXIT_CRITICAL(&s_hit_mux);
    }
}
// 与正式固件保持同样的接口形态（大/小符两套配置 + 模式切换）
// 测试模式下不区分大小符，统一用 PARAM_HIT_THRESHOLD/PARAM_EMA_ALPHA 上限调参
extern "C" void sensor_processor_set_params(
    size_t ch, float alpha_big, float alpha_small,
    float thr_big, float thr_small, uint32_t refrac)
{
    (void)alpha_big; (void)thr_big;  // 测试模式只用 small 那套
    if (ch > 0)           g_ch      = ch;
    if (alpha_small > 0 && alpha_small < 1) g_alpha = alpha_small;
    if (thr_small >= 0)   g_hit_thresh = thr_small;
    if (refrac > 0)       g_refrac   = refrac;
}

extern "C" void sensor_processor_set_mode(uint8_t /*mode*/)
{
    // 测试模式下不切换；保留接口仅为链接兼容
}

// ================================================================
//  app_main
// ================================================================
extern "C" void app_main(void)
{
    // ── 1. NVS ────────────────────────────────────────────────────
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // ── 2. Config（读取 NVS 里的 armour_id，失败时用 Kconfig 默认值）
    static Config cfg;
    config = &cfg;

    // ── 3. 事件循环 ────────────────────────────────────────────────
    esp_event_loop_args_t lp_args = {
        .queue_size      = 8,
        .task_name       = "thresh_evloop",
        .task_priority   = 4,
        .task_stack_size = 4096,
        .task_core_id    = tskNO_AFFINITY,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&lp_args, &pr_events_loop_handle));
    ESP_ERROR_CHECK(esp_event_handler_register_with(
        pr_events_loop_handle, PRA, PRA_HIT_EVENT, on_hit_event, nullptr));

    // ── 3b. LED 初始化 ─────────────────────────────────────────────
    init_led();
    xTaskCreate(led_task, "thresh_led", 6144, nullptr, 3, &s_led_task_handle);

    // ── 4. 打印当前参数 ────────────────────────────────────────────
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  传感器阈值调试模式");
    ESP_LOGI(TAG, "  hit_threshold    = %.1f", (float)PARAM_HIT_THRESHOLD);
    ESP_LOGI(TAG, "  ema_alpha        = %.4f", (float)PARAM_EMA_ALPHA);
    ESP_LOGI(TAG, "  refractory_ms    = %d",   PARAM_REFRACTORY_MS);
    ESP_LOGI(TAG, "  channel_count    = %d",   PARAM_CHANNEL_COUNT);
#if DIAG_RAW_PRINT_INTERVAL > 0
    ESP_LOGI(TAG, "  RAW 诊断每 %d 帧打印一次", DIAG_RAW_PRINT_INTERVAL);
#else
    ESP_LOGI(TAG, "  RAW 诊断已关闭");
#endif
    ESP_LOGI(TAG, "========================================");

    // ── 5. UART（接收压感数据） ────────────────────────────────────
    uart_handler_init();
    uart_handler_start();

    // ── 6. 统计任务 ────────────────────────────────────────────────
    xTaskCreate(stats_task, "thresh_stats", 2048, nullptr,
                tskIDLE_PRIORITY + 2, nullptr);

    ESP_LOGI(TAG, "就绪，等待传感器数据……（用子弹打击后观察串口）");
    vTaskSuspend(nullptr);
}
