#include "uart_handler.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <stdlib.h>
#include <string.h>
#include "sensor_processor.h"
#include <inttypes.h> 

static const char* TAG = "uart_handler";

static TaskHandle_t s_uart_task_handle = NULL;
static volatile bool s_running = false;

// stats
static uint32_t s_total_bytes = 0;
static uint32_t s_rx_chunks = 0;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

static void uart_rx_task(void* arg)
{
    uint8_t* buf = (uint8_t*)malloc(UART_READ_CHUNK_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "malloc failed");
        vTaskDelete(NULL);
        return;
    }
    
    // 缓冲区状态
    uint8_t packet_buffer[18];
    size_t packet_pos = 0;
    uint32_t packet_count = 0;
    uint32_t error_count = 0;
    
    while (s_running) {
        // uart_read_bytes 自带阻塞，没有数据会让出 CPU
        int len = uart_read_bytes(UART_PORT_NUM, buf, UART_READ_CHUNK_SIZE, pdMS_TO_TICKS(UART_RX_TIMEOUT_MS));
        if (len > 0) {
            portENTER_CRITICAL(&s_stats_mux);
            s_total_bytes += (uint32_t)len;
            s_rx_chunks++;
            portEXIT_CRITICAL(&s_stats_mux);

            // 处理每个字节
            for (int i = 0; i < len; i++) {
                packet_buffer[packet_pos++] = buf[i];
                
                // 检查是否凑齐了 18 字节的数据包
                if (packet_pos == 18) {
                    bool is_valid_packet = false;

                    // 1. 检查帧头
                    if (packet_buffer[0] == 0xA5 && packet_buffer[1] == 0x5A &&
                        packet_buffer[2] == 0x01 && packet_buffer[3] == 0x10 &&
                        packet_buffer[4] == 0x00) {
                        // 2. 检查包类型
                        if (packet_buffer[5] == 0x01) {
                            // 3. 计算校验和
                            uint16_t checksum = (packet_buffer[17] << 8) | packet_buffer[16];
                            uint16_t calculated_checksum = 0;
                            for (int j = 0; j < 16; j++) {
                                calculated_checksum += packet_buffer[j];
                            }
                            
                            if (checksum == calculated_checksum) {
                                is_valid_packet = true;
                                packet_count++;
                                // 提取压感数据并处理：data 从 packet_buffer[6] 起，共 10 字节
                                sensor_processor_input_cb(&packet_buffer[6], 10, NULL);
                                
                                if (packet_count % 500 == 0) {
                                    ESP_LOGI(TAG, "Total valid packets: %" PRIu32 ", Frame errors: %" PRIu32, packet_count, error_count);
                                }
                            }
                        }
                    }

                    if (is_valid_packet) {
                        // 解析成功，清空位置，准备接下一个完整包
                        packet_pos = 0;
                    } else {
                        // 【滑动窗口纠错机制】
                        // 如果解析失败（帧头错位、校验和错误），不能直接清零！
                        // 丢弃第 0 个字节，把后面的 17 个字节往前挪一位
                        memmove(packet_buffer, packet_buffer + 1, 17);
                        packet_pos = 17; // 下一次循环只读入 1 个字节就能再次凑齐 18 字节进行判断
                        error_count++;
                    }
                }
            }
            // 移除了原有的 vTaskDelay(1)，使得高波特率下的吞吐量最大化
        } else {
            // 超时无数据，稍微让出 CPU
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    free(buf);
    s_uart_task_handle = NULL;
    vTaskDelete(NULL);
}

void uart_handler_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "uart_handler_init port=%d baud=%d rx=%d tx=%d", UART_PORT_NUM, UART_BAUD_RATE, UART_RX_PIN, UART_TX_PIN);

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    portENTER_CRITICAL(&s_stats_mux);
    s_total_bytes = 0;
    s_rx_chunks = 0;
    portEXIT_CRITICAL(&s_stats_mux);

    ESP_LOGI(TAG, "uart_handler init done");
}

void uart_handler_start(void)
{
    if (s_running) {
        ESP_LOGW(TAG, "uart already running");
        return;
    }
    s_running = true;
    uart_flush_input(UART_PORT_NUM);
    BaseType_t r = xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, tskIDLE_PRIORITY + 5, &s_uart_task_handle);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create uart_rx_task");
        s_running = false;
    }
}

void uart_handler_stop(void)
{
    if (!s_running) return;
    s_running = false;
    for (int i = 0; i < 100 && s_uart_task_handle != NULL; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_uart_task_handle = NULL;
    ESP_LOGI(TAG, "uart stopped");
}

void uart_handler_deinit(void)
{
    uart_handler_stop();
    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));
    ESP_LOGI(TAG, "uart driver deleted");
}

void uart_handler_get_stats(uint32_t* total_bytes, uint32_t* rx_chunks)
{
    portENTER_CRITICAL(&s_stats_mux);
    if (total_bytes) *total_bytes = s_total_bytes;
    if (rx_chunks) *rx_chunks = s_rx_chunks;
    portEXIT_CRITICAL(&s_stats_mux);
}
