/**
 * @file ble_sensor_monitor.h
 * @brief BLE 传感器监控模块 —— 通过 BLE NUS 协议向电脑实时推送传感器数据
 *
 * 设计原则：
 *  - 完全独立，不影响现有 TCP 通信
 *  - 未连接时所有发送函数立即返回，零开销
 *  - 设备名格式 "PR-Arm-X"（X = armour_id 1~5）
 *
 * 数据格式（纯文本行，每行以 '\n' 结尾）：
 *
 *  传感器帧（每 25 帧约 20 Hz）：
 *    "$D <mode> <armed> <raw0..raw9> <dif0..dif9> <max_diff>\n"
 *    示例: "$D B 1 123 45 67 89 12 34 56 78 90 11 5.2 3.1 2.8 6.7 1.2 0.8 1.1 0.9 2.3 0.5 6.7\n"
 *           模式(B/S)  武装  ←── 原始 ADC(0~255) ──→  ←─── 差分=原始-EMA基线 ───→  最大差分
 *
 *    字段下标（split 后）：
 *      [0]    = "$D"
 *      [1]    = mode  'B'=大符 'S'=小符
 *      [2]    = armed  1=武装 0=冷却
 *      [3..12]  = raw ADC ch0..ch9（整数 0~255）
 *      [13..22] = 差分 d0..d9（浮点，正值=高于基线=信号突起）
 *      [23]   = max_diff（当前帧最大差分）
 *
 *  命中事件（立即推送）：
 *    "$H <mode> <ring> <max_diff>\n"
 *    示例: "$H B 7 35.2\n"
 *           模式(B/S)  环号(1~10)  峰值差分
 */
#pragma once
#ifndef __BLE_SENSOR_MONITOR_H__
#define __BLE_SENSOR_MONITOR_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 BLE 监控模块
 *
 * 必须在 WiFi / TCP 初始化完成后调用（保证射频共存正常工作）。
 * 调用后自动开始广播，等待电脑连接。
 *
 * @param armour_id 本装甲板编号 (1~5)，影响广播名 "PR-Arm-X"
 */
void ble_sensor_monitor_init(uint8_t armour_id);

/**
 * @brief 发送一帧传感器快照（在 sensor_processor_input_cb 内调用）
 *
 * 无客户端连接或未订阅时立即返回，不阻塞。
 *
 * @param adc      10 通道原始 ADC 数组（0~255）
 * @param diffs    对应通道差分数组（raw - EMA基线，正值表示信号突起）
 * @param n        有效通道数（通常 10）
 * @param max_diff 当前帧最大差值
 * @param armed    是否处于武装态
 * @param mode     PRA_RUNE_BIG_MODE / PRA_RUNE_SMALL_MODE
 */
void ble_sensor_monitor_send_frame(const uint8_t* adc, const float* diffs, size_t n,
                                   float max_diff, bool armed, uint8_t mode);

/**
 * @brief 发送命中事件（在确认命中时立即调用）
 *
 * @param ring     命中环号 (1~10)
 * @param max_diff 命中时的峰值差分
 * @param mode     PRA_RUNE_BIG_MODE / PRA_RUNE_SMALL_MODE
 */
void ble_sensor_monitor_send_hit(uint8_t ring, float max_diff, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif // __BLE_SENSOR_MONITOR_H__
