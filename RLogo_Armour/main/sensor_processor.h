#ifndef __SENSOR_PROCESSOR_H__
#define __SENSOR_PROCESSOR_H__

#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"
#include "PowerRune_Events.h"
#include "firmware.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化（调用一次）
void sensor_processor_init(void);

// 设置检测参数（在 init 后调用，会重置基线）。
// 同时存大/小符两套 alpha 和阈值；当前生效的那一套由 set_mode 切换。
void sensor_processor_set_params(size_t channel_count,
                                 float ema_alpha_big,
                                 float ema_alpha_small,
                                 float hit_threshold_big,
                                 float hit_threshold_small,
                                 uint32_t refractory_ms);

// 切换当前模式（大/小符），不重置基线。
// mode 取 PRA_RUNE_BIG_MODE / PRA_RUNE_SMALL_MODE
void sensor_processor_set_mode(uint8_t mode);

// 电机启动时调用：前2.5秒完全舍弃，随后7.5秒仅采样运行噪声；满10秒后生成判据并启用检测。
void sensor_processor_start_calibration(void);

// Reset the reference detector after a STOP, parameter update, or fresh game.
void sensor_processor_reset(void);

// Python 波形采集开关。目标 Armour 进入大小符模式后开启，退出目标状态后关闭。
// 每帧输出 "$ADC,<序号>,<模式>,<校准有效>,<判定条件>,<ch0>...<ch9>"。
void sensor_processor_set_raw_log(bool enable);

// 供 sensor_parser 注册的回调
void sensor_processor_input_cb(const uint8_t* adc, size_t adc_len, void* user_ctx);

// 调试：获取内部统计（触发计数）
void sensor_processor_get_stats(uint32_t* hits_count);

// Match a Master decision to a locally pending candidate.  Returns true when
// the candidate was still inside the 100 ms display deadline.
bool sensor_processor_mark_hit_decision(
    const PRA_HIT_DECISION_EVENT_DATA *decision,
    uint32_t *elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_PROCESSOR_H__
