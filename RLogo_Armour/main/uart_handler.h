#ifndef __UART_HANDLER_H__
#define __UART_HANDLER_H__

/**
 * @file uart_handler.h
 * @brief UART 接收模块（只负责接收字节并把数据交给解析器）
 *
 * 职责：
 *  - 初始化 UART 驱动并创建接收任务；
 *  - 读取串口字节，将读到的字节块传给外部解析器函数 sensor_parser_feed_bytes();
 *  - 提供控制与统计 API（start/stop/stats）。
 *
 * 配置（可在编译时通过 -D 覆盖）：
 *  - UART_BAUD_RATE = 460800
 *  - UART_READ_CHUNK_SIZE = 256
 *  - UART_RX_BUF_SIZE = 4096
 *  - UART_RX_TIMEOUT_MS = 2
 *
 * 注意：本文件不做任何协议解析或判定，解析由 sensor_parser 模块完成。
 */

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_PORT_NUM
#define UART_PORT_NUM        UART_NUM_1
#endif

#ifndef UART_BAUD_RATE
#define UART_BAUD_RATE       460800
#endif

#ifndef UART_TX_PIN
#define UART_TX_PIN          4
#endif

#ifndef UART_RX_PIN
#define UART_RX_PIN          5
#endif

#ifndef UART_READ_CHUNK_SIZE
#define UART_READ_CHUNK_SIZE 256
#endif

#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE     4096
#endif

#ifndef UART_RX_TIMEOUT_MS
#define UART_RX_TIMEOUT_MS   100
#endif

// 初始化 UART 驱动（安装驱动、参数配置），不启动接收任务
void uart_handler_init(void);

// 启动接收任务（创建任务并开始读取）。如果已启动不会重复创建
void uart_handler_start(void);

// 停止接收任务（任务退出）
void uart_handler_stop(void);

// 卸载 UART 驱动并释放资源
void uart_handler_deinit(void);

// 获取统计信息（线程安全）
void uart_handler_get_stats(uint32_t* total_bytes, uint32_t* rx_chunks);



#ifdef __cplusplus
}
#endif

#endif // __UART_HANDLER_H__