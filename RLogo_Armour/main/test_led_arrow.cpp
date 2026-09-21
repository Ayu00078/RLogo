/**
 * @file test_led_arrow.cpp
 * @brief 灯板综合测试程序（用于排查灯板硬件故障）
 *
 * 使用方法：
 *   在 main/CMakeLists.txt 中将 "app_main.cpp" / "armour_app.cpp" 等注释掉，
 *   启用 "test_led_arrow.cpp"，然后编译烧录即可。
 *
 * 测试流程（自动循环）：
 *   1. 全板纯色测试  —— 依次全红/全绿/全蓝/全白，检查整板亮灭
 *   2. 子板分区测试  —— 依次独立点亮4块 16×16 子板，定位损坏分区
 *   3. 逐行扫描      —— 从上到下逐行点亮（红色），确认行走线是否连通
 *   4. 逐列扫描      —— 从左到右逐列点亮（蓝色），确认列走线是否连通
 *   5. 棋盘格测试    —— 间隔点亮像素，检查相邻LED干扰
 *   6. 臂部 LED 与灯带测试（颜色轮替）
 *
 * 所有测试结果同步通过串口日志 (ESP_LOGI) 输出，方便对照观察。
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_controller.h"
#include "esp_log.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

static const char* TAG = "led_test";

/**
 * 灯板数据线 = led_controller.h 里的 LED_BOARD_GPIO_PIN（当前为 13）。
 * ESP32-S3：GPIO0~31 共用 GPIO_OUT_REG / GPIO_ENABLE_REG / GPIO_IN_REG，单脚对应「寄存器里的 bit N」。
 * - GPIO_OUT_REG   (DR_REG_GPIO_BASE+0x04) 输出锁存
 * - GPIO_ENABLE_REG(DR_REG_GPIO_BASE+0x20) 输出使能
 * - GPIO_IN_REG    (DR_REG_GPIO_BASE+0x3C) 输入采样
 * IO_MUX：本板灯脚在 0~14 范围内时，地址为 REG_IO_MUX_BASE+0x04+pin*4（与 io_mux_reg.h 中 PERIPHS_IO_MUX_GPIOx_U 一致）
 */
static void log_led_board_gpio_registers(void)
{
    const int pin = LED_BOARD_GPIO_PIN;
    if (pin < 0 || pin > 31) {
        ESP_LOGW(TAG, "灯板 GPIO=%d 超出 OUT/IN 寄存器 bit 范围，跳过寄存器打印", pin);
        return;
    }

    const uint32_t out = REG_READ(GPIO_OUT_REG);
    const uint32_t oe = REG_READ(GPIO_ENABLE_REG);
    const uint32_t in = REG_READ(GPIO_IN_REG);

    uint32_t mux = 0;
    if (pin >= 0 && pin <= 14) {
        mux = REG_READ(REG_IO_MUX_BASE + 0x04 + (uint32_t)pin * 4);
    }

    ESP_LOGI(TAG,
             "[board GPIO%d] OUT_REG=0x%08" PRIx32 " bit%d=%" PRIu32 " | OE_REG=0x%08" PRIx32 " bit%d=%" PRIu32
             " | IN_REG=0x%08" PRIx32 " bit%d=%" PRIu32,
             pin, out, pin, (uint32_t)((out >> pin) & 1),
             oe, pin, (uint32_t)((oe >> pin) & 1),
             in, pin, (uint32_t)((in >> pin) & 1));
    if (pin <= 14) {
        const uint32_t iomux_addr = (uint32_t)(REG_IO_MUX_BASE + 0x04 + (uint32_t)pin * 4);
        ESP_LOGI(TAG, "[board GPIO%d] IO_MUX@0x%08" PRIx32 " = 0x%08" PRIx32 " (see io_mux_reg.h)",
                 pin, iomux_addr, mux);
    } else {
        ESP_LOGI(TAG, "[灯板数据线 GPIO%d] IO_MUX 请查 TRM / io_mux_reg.h 中该脚对应宏", pin);
    }
}

// ==================== 灯板工具函数 ====================

/** 用原始索引填充整个灯板（不经过坐标映射） */
static void board_fill_raw(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_BOARD_LED_COUNT; i++) {
        led_strip_set_pixel(led_board, i, r, g, b);
    }
    led_strip_refresh(led_board);
}

/** 填充整个臂部LED */
static void arm_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_ARM_WIDTH * LED_ARM_HEIGHT; i++) {
        led_strip_set_pixel(led_arm, i, r, g, b);
    }
    led_strip_refresh(led_arm);
}

/** 填充整条灯带 */
static void strip_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
        led_strip_set_pixel(led_strip, i, r, g, b);
    }
    led_strip_refresh(led_strip);
}

/** 清空所有LED */
static void all_clear()
{
    led_strip_clear(led_board);
    led_strip_clear(led_arm);
    led_strip_clear(led_strip);
    led_strip_refresh(led_board);
    led_strip_refresh(led_arm);
    led_strip_refresh(led_strip);
}

// ==================== 各项测试函数 ====================

/**
 * 测试1：全板纯色测试
 * 依次显示 红/绿/蓝/白，每色持续 hold_ms 毫秒。
 * 用于快速判断整块灯板是否正常点亮。
 */
static void test_solid_colors(int hold_ms)
{
    const struct { uint8_t r, g, b; const char* name; } colors[] = {
        {255,   0,   0, "红色"},
        {  0, 255,   0, "绿色"},
        {  0,   0, 255, "蓝色"},
        {255, 255, 255, "白色"},
    };

    for (auto& c : colors) {
        ESP_LOGI(TAG, "[纯色] 全板 %s", c.name);
        board_fill_raw(c.r, c.g, c.b);
        arm_fill(c.r, c.g, c.b);
        strip_fill(c.r, c.g, c.b);
        vTaskDelay(pdMS_TO_TICKS(hold_ms));
    }
}

/**
 * 测试2：子板分区测试
 * 灯板共分为4块 16×16 子板（索引0~3），每次只点亮一块。
 * 可以清楚看出哪块子板不亮/颜色异常。
 */
static void test_subboards(int hold_ms)
{
    const int leds_per_board = LED_BOARD_LED_COUNT / 4;  // 每块256个LED

    for (int b = 0; b < 4; b++) {
        ESP_LOGI(TAG, "[子板] 点亮子板 #%d (索引 %d~%d)", b,
                 b * leds_per_board, (b + 1) * leds_per_board - 1);
        led_strip_clear(led_board);
        int offset = b * leds_per_board;
        for (int i = 0; i < leds_per_board; i++) {
            led_strip_set_pixel(led_board, offset + i, 255, 255, 255);
        }
        led_strip_refresh(led_board);
        vTaskDelay(pdMS_TO_TICKS(hold_ms));
    }
    led_strip_clear(led_board);
    led_strip_refresh(led_board);
}

/**
 * 测试3：逐行扫描
 * 使用坐标映射逐行（Y轴）点亮，每行持续 row_ms 毫秒后点亮下一行（累积）。
 * 用于确认行方向的走线是否完整。
 */
static void test_scan_rows(uint8_t r, uint8_t g, uint8_t b, int row_ms)
{
    ESP_LOGI(TAG, "[逐行] 开始从上到下逐行扫描");
    led_strip_clear(led_board);
    led_strip_refresh(led_board);

    for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
        for (int x = 0; x < LED_BOARD_WIDTH; x++) {
            set_pixel(led_board, x, y, Vec3(r, g, b), false);
        }
        led_strip_refresh(led_board);
        ESP_LOGD(TAG, "[逐行] 第 %d 行", y);
        vTaskDelay(pdMS_TO_TICKS(row_ms));
    }
}

/**
 * 测试4：逐列扫描
 * 使用坐标映射逐列（X轴）点亮，用于确认列方向走线。
 */
static void test_scan_cols(uint8_t r, uint8_t g, uint8_t b, int col_ms)
{
    ESP_LOGI(TAG, "[逐列] 开始从左到右逐列扫描");
    led_strip_clear(led_board);
    led_strip_refresh(led_board);

    for (int x = 0; x < LED_BOARD_WIDTH; x++) {
        for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
            set_pixel(led_board, x, y, Vec3(r, g, b), false);
        }
        led_strip_refresh(led_board);
        ESP_LOGD(TAG, "[逐列] 第 %d 列", x);
        vTaskDelay(pdMS_TO_TICKS(col_ms));
    }
}

/**
 * 测试5：棋盘格测试
 * 点亮坐标 (x+y) 为偶数的像素，形成棋盘格。
 * 用于检查相邻 LED 是否存在串扰或漏光。
 */
static void test_checkerboard(int hold_ms)
{
    ESP_LOGI(TAG, "[棋盘] 棋盘格测试");
    led_strip_clear(led_board);

    for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
        for (int x = 0; x < LED_BOARD_WIDTH; x++) {
            if ((x + y) % 2 == 0) {
                set_pixel(led_board, x, y, Vec3(255, 255, 255), false);
            }
        }
    }
    led_strip_refresh(led_board);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));

    // 反相棋盘格
    led_strip_clear(led_board);
    for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
        for (int x = 0; x < LED_BOARD_WIDTH; x++) {
            if ((x + y) % 2 == 1) {
                set_pixel(led_board, x, y, Vec3(255, 255, 255), false);
            }
        }
    }
    led_strip_refresh(led_board);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));

    led_strip_clear(led_board);
    led_strip_refresh(led_board);
}

/**
 * 测试6：臂部与灯带走马灯
 * 依次逐个点亮，用于确认臂部LED和灯带的连通性。
 */
static void test_arm_strip_sweep(int delay_ms)
{
    ESP_LOGI(TAG, "[臂部] 逐个点亮臂部LED");
    led_strip_clear(led_arm);
    led_strip_refresh(led_arm);
    for (int i = 0; i < LED_ARM_WIDTH * LED_ARM_HEIGHT; i++) {
        led_strip_set_pixel(led_arm, i, 0, 255, 0);
        led_strip_refresh(led_arm);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    ESP_LOGI(TAG, "[灯带] 逐个点亮灯带");
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
        led_strip_set_pixel(led_strip, i, 0, 0, 255);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    led_strip_clear(led_arm);
    led_strip_clear(led_strip);
    led_strip_refresh(led_arm);
    led_strip_refresh(led_strip);
}

// ==================== 主入口 ====================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== 灯板硬件测试程序启动 ===");
    ESP_LOGI(TAG, "灯板: %d×%d=%d LED  GPIO%d",
             LED_BOARD_WIDTH, LED_BOARD_HEIGHT, LED_BOARD_LED_COUNT, LED_BOARD_GPIO_PIN);
    ESP_LOGI(TAG, "臂部: %d×%d=%d LED  GPIO%d",
             LED_ARM_WIDTH, LED_ARM_HEIGHT, LED_ARM_WIDTH * LED_ARM_HEIGHT, LED_ARM_GPIO_PIN);
    ESP_LOGI(TAG, "灯带: %d LED  GPIO%d", LED_STRIP_LED_COUNT, LED_STRIP_GPIO_PIN);

    init_led();
    vTaskDelay(pdMS_TO_TICKS(500));
    log_led_board_gpio_registers();

    while (1) {
        // --- 测试1：全板纯色（每色2秒）---
        ESP_LOGI(TAG, "====== 测试1：全板纯色 ======");
        test_solid_colors(2000);
        all_clear();
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- 测试2：子板分区（每块1.5秒）---
        ESP_LOGI(TAG, "====== 测试2：子板分区 ======");
        test_subboards(1500);
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- 测试3：逐行扫描（每行80ms）---
        ESP_LOGI(TAG, "====== 测试3：逐行扫描（红色）======");
        test_scan_rows(255, 0, 0, 80);
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- 测试4：逐列扫描（每列80ms）---
        ESP_LOGI(TAG, "====== 测试4：逐列扫描（蓝色）======");
        test_scan_cols(0, 0, 255, 80);
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- 测试5：棋盘格（每相1.5秒）---
        ESP_LOGI(TAG, "====== 测试5：棋盘格 ======");
        test_checkerboard(1500);
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- 测试6：臂部 & 灯带逐个点亮（每颗15ms）---
        ESP_LOGI(TAG, "====== 测试6：臂部/灯带扫描 ======");
        test_arm_strip_sweep(15);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "====== 一轮测试完成，重新开始 ======");
    }
}
