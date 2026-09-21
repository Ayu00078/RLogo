#include "led_controller.h"
#include "data.h"
#include "esp_log.h"
#include <algorithm> 
#include <cstring>

static const char* TAG = "led_ctrl";
// ==================== 全局LED控制器句柄定义 ====================
// 这些是系统中使用的三个主要LED控制器实例

// 装甲板臂部LED控制器
led_strip_handle_t led_arm;

// LED灯带控制器
led_strip_handle_t led_strip;

// LED显示面板控制器
led_strip_handle_t led_board;

// esp32板载LED控制器（状态指示用）
led_strip_handle_t led_esp32;

// 偏置值，用于改善圆形绘制效果
#define BIAS LED_BOARD_WIDTH * 0.015f

// ==================== LED配置函数 ====================

/**
 * @brief 配置LED控制器
 * @details 根据提供的配置参数创建和初始化LED控制器
 * @param config LED配置参数结构体
 * @return 配置好的LED控制器句柄
 */
led_strip_handle_t configure_led(led_config_t config)
{
    // LED灯带通用初始化配置
    led_strip_config_t strip_config = {
        .strip_gpio_num = config.strip_gpio_num,    // 连接到LED灯带数据线的GPIO引脚
        .max_leds = config.max_leds,                // 灯带中的LED数量
        .led_model = LED_MODEL_WS2812,              // LED灯带型号（WS2812）
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // 颜色顺序：GRB
        .flags = {
            .invert_out = false,                    // 不反转输出信号
        }
    };
    
    // LED灯带后端配置：使用RMT（遥控微控制器）
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,             // 使用默认时钟源
        .resolution_hz = LED_STRIP_RMT_RES_HZ,      // RMT计数器时钟频率
        .mem_block_symbols = config.mem_block_symbols, // RMT通道使用的内存块大小
        .flags = {
            .with_dma = config.with_dma,            // 使用DMA可以提高驱动更多LED时的性能
        }
    };

    // LED灯带对象句柄
    led_strip_handle_t led_handle;
    
    // 创建新的RMT设备并进行错误检查
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_handle));
    
    return led_handle;
}

// ==================== 系统初始化函数 ====================

/**
 * @brief 初始化LED系统
 * @details 初始化所有LED控制器，包括GPIO配���、内存分配等
 *          设置默认的LED状态和颜色
 */
void init_led(){
    // 配置各个LED控制器的参数
    
    // ESP32板载LED配置（用于状态指示）
    led_config_t esp32_config = led_config_t(48, 1, 0, 0); // gpio 48, 1个LED, 自动内存大小, 不使用DMA
    
    // 装甲板臂部LED配置
    led_config_t arm_config = led_config_t(LED_ARM_GPIO_PIN, LED_ARM_WIDTH * LED_ARM_HEIGHT, 0, 0);
    
    // 主LED灯带配置
    led_config_t strip_config = led_config_t(LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT, 0, 0);
    
    // LED显示面板配置
    led_config_t board_config = led_config_t(LED_BOARD_GPIO_PIN, LED_BOARD_WIDTH * LED_BOARD_HEIGHT, 0, 1);
    
    // 创建各个LED控制器实例
    led_esp32 = configure_led(esp32_config);
    led_arm = configure_led(arm_config);
    led_board = configure_led(board_config);
    led_strip = configure_led(strip_config);
}

// ==================== 像素控制函数 ====================

// 靶心图案像素缓存
static struct {
    uint8_t r[LED_BOARD_LED_COUNT];
    uint8_t g[LED_BOARD_LED_COUNT];
    uint8_t b[LED_BOARD_LED_COUNT];
    int cached_color;
    bool valid;
} s_target_cache = { {0}, {0}, {0}, -999, false };

// 控制是否录制缓存
static bool s_recording_cache = false;

/**
 * @brief 设置指定位置的LED像素颜色
 * @details 在指定的LED控制器上设置特定坐标位置的像素颜色
 * @param led LED控制器句柄
 * @param x X坐标
 * @param y Y坐标
 * @param color RGB颜色值
 * @param skip_zero 是否跳过黑色像素（true=跳过，false=设置）
 */
void set_pixel(led_strip_handle_t led, const int& x, const int& y, const Vec3& color, bool skip_zero) {
    // 边界检查
    if (x < 0 || x >= LED_BOARD_WIDTH || y < 0 || y >= LED_BOARD_HEIGHT) return;
    
    // 如果启用了跳过黑色像素且当前颜色为黑色，则直接返回
    if (skip_zero && color == Vec3(0,0,0)) return;
    
    // 计算在哪个板子上 (0-3)
    // 将大的LED面板分割成4个小板子进行管理
    int board_x = x / (LED_BOARD_WIDTH / 2);    // 计算水平板子编号
    int board_y = y / (LED_BOARD_HEIGHT / 2);   // 计算垂直板子编号
    int board_num = board_y * 2 + board_x;      // 计算总的板子编号
    
    // 计算板子内的相对坐标
    int local_x = x % (LED_BOARD_WIDTH / 2);    // 板子内X坐标
    int local_y = y % (LED_BOARD_HEIGHT / 2);   // 板子内Y坐标

    // 计算在板子内的LED索引
    int led_index;
    if (local_y % 2 == 0) { // 偶数行：从右到左排列
        led_index = local_y * (LED_BOARD_WIDTH / 2) + ((LED_BOARD_WIDTH / 2) - 1 - local_x);
    } else { // 奇数行：从左到右排列
        led_index = local_y * (LED_BOARD_WIDTH / 2) + local_x;
    }

    // 计算全局LED索引
    int board_offset = board_num * LED_BOARD_WIDTH * LED_BOARD_HEIGHT / 4;  // 板子偏移量
    const size_t index = board_offset + led_index;                          // 全局索引
    
    // 数组越界检测
    if(index >= LED_BOARD_LED_COUNT) return;
    
    // 设置LED像素颜色
    led_strip_set_pixel(led, index, color.y, color.x, color.z);

    if (s_recording_cache && led == led_board) {
        s_target_cache.r[index] = color.y;
        s_target_cache.g[index] = color.x;
        s_target_cache.b[index] = color.z;
    }
}

// ==================== 图形绘制函数 ====================

/**
 * @brief 绘制抗锯齿圆环
 */
void draw_ring_ssaa(float cx, float cy, float inner_radius, float outer_radius, 
                    const Vec3& color, int ssaa_factor) { 
    
    // SSAA采样参数设置
    float sample_step = 1.0f / ssaa_factor;         // 采样步长
    float sample_weight = 1.0f / (ssaa_factor * ssaa_factor); // 采样权重
    
    // 预计算半径的平方值，避免重复计算
    float inner_r_sq = inner_radius * inner_radius;
    float outer_r_sq = outer_radius * outer_radius;
    
    // 遍历LED矩阵的每个像素
    for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
        for (int x = 0; x < LED_BOARD_WIDTH; x++) {
            float r = 0;    // 红色分量累加器
            float g = 0;    // 绿色分量累加器
            float b = 0;    // 蓝色分量累加器
            
            // 对每个像素进行超采样
            for (int sy = 0; sy < ssaa_factor; sy++) {
                for (int sx = 0; sx < ssaa_factor; sx++) {
                    // 计算子像素坐标
                    float sub_x = x - 0.5 + (sx + 0.5) * sample_step;
                    float sub_y = y - 0.5 + (sy + 0.5) * sample_step;
                    
                    // 计算到圆心的距离
                    float dx = sub_x - cx;
                    float dy = sub_y - cy;
                    float distance = dx * dx + dy * dy;
                    
                    // 如果距离在内外半径之间，则累加颜色值
                    if (distance >= inner_r_sq && distance <= outer_r_sq){
                        r += color.x;
                        g += color.y;
                        b += color.z;
                    }
                }
            }
            
            // 应用采样权重
            r *= sample_weight;
            g *= sample_weight;
            b *= sample_weight;

            // 设置像素颜色（加上0.5进行四舍五入）
            set_pixel(led_board, x, y, Vec3(r + 0.5f, g + 0.5f, b + 0.5f));
        }
    }
}

/**
 * @brief 绘制扇形圆环
 */
void draw_arc_ring_ssaa(float cx, float cy, float inner_radius, float outer_radius,
                        float start_angle, float end_angle, const Vec3& color, int ssaa_factor) {
    
    // SSAA采样参数
    float sample_step = 1.0f / ssaa_factor;
    float sample_weight = 1.0f / (ssaa_factor * ssaa_factor);
    
    // 预计算半径平方
    float inner_radius_sq = inner_radius * inner_radius;
    float outer_radius_sq = outer_radius * outer_radius;
    
    // 计算圆弧的总跨度（弧度）
    float sweep_angle = end_angle - start_angle;
    
    // 处理角度跨越0度的情况（例如350°->10°）
    while (sweep_angle < 0) sweep_angle += 2 * M_PI;
    
    // 处理全圆情况
    if (sweep_angle == 0 && start_angle != end_angle) sweep_angle = 2 * M_PI;

    // 遍历所有像素
    for (int y = 0; y < LED_BOARD_HEIGHT; y++) {
        for (int x = 0; x < LED_BOARD_WIDTH; x++) {
            float r = 0, g = 0, b = 0;
            
            // 超采样处理
            for (int sy = 0; sy < ssaa_factor; sy++) {
                for (int sx = 0; sx < ssaa_factor; sx++) {
                    // 计算子像素坐标
                    float sub_x = x - 0.5f + (sx + 0.5f) * sample_step;
                    float sub_y = y - 0.5f + (sy + 0.5f) * sample_step;
                    
                    // 计算距离和角度
                    float dx = sub_x - cx;
                    float dy = sub_y - cy;
                    float distance_sq = dx * dx + dy * dy;
                    
                    // 检查是否在半径范围内
                    if (distance_sq >= inner_radius_sq && distance_sq <= outer_radius_sq) {
                        float angle = atan2f(dy, dx);  // 计算角度
                        
                        // 计算相对于起始角的相对角度
                        float relative_angle = angle - start_angle;
                        
                        // 将相对角度归一化到 [0, 2π)
                        while (relative_angle < 0) relative_angle += 2 * M_PI;
                        while (relative_angle >= 2 * M_PI) relative_angle -= 2 * M_PI;
                        
                        // 检查是否在扇形角度范围内
                        if (relative_angle <= sweep_angle + 1e-4f) { 
                            r += color.x;
                            g += color.y;
                            b += color.z;
                        }
                    }
                }
            }
            
            // 应用权重并设置像素
            r *= sample_weight;
            g *= sample_weight;
            b *= sample_weight;
            
            if (r > 0 || g > 0 || b > 0) {
                 set_pixel(led_board, x, y, Vec3(r + 0.5f, g + 0.5f, b + 0.5f)); 
            }
        }
    }
}

// ==================== 显示效果函数 ====================

/**
 * @brief 显示结果数字
 */
void show_result(int is_blue, int num) { 
    ESP_LOGI(TAG, "show_result: color=%s num=%d", is_blue ? "BLUE" : "RED", num);
    Vec3 color = Vec3(255, 0, 0);
    if(is_blue) color = Vec3(0, 0, 255);

    led_strip_clear(led_board);
    
    draw_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                   (10 - num) * LED_BOARD_WIDTH * 0.05f + BIAS, 
                   (11 - num) * LED_BOARD_WIDTH * 0.05f - BIAS, 
                   color, 4);
    
    led_strip_refresh(led_board);
}

/**
 * @brief 显示目标图案（带缓存优化）
 * @details 首次渲染时执行完整的SSAA计算并缓存结果，
 *          后续调用直接从缓存恢复像素数据，大幅降低延迟。
 */

void show_target(int is_blue) {
    ESP_LOGD(TAG, "show_target: color=%s cached=%d", is_blue ? "BLUE" : "RED", s_target_cache.valid);
    if (s_target_cache.valid && s_target_cache.cached_color == is_blue) {
        for (int i = 0; i < LED_BOARD_LED_COUNT; i++) {
            led_strip_set_pixel(led_board, i, s_target_cache.r[i], s_target_cache.g[i], s_target_cache.b[i]);
        }
        led_strip_refresh(led_board);
        return;
    }

    // 缓存无效或颜色改变，执行完整渲染
    Vec3 color = Vec3(255, 0, 0);
    if(is_blue) color = Vec3(0, 0, 255);

    led_strip_clear(led_board);

    // 在首次渲染时，将渲染结果通过 `set_pixel` 同步记录在缓存中
    memset(s_target_cache.r, 0, sizeof(s_target_cache.r));
    memset(s_target_cache.g, 0, sizeof(s_target_cache.g));
    memset(s_target_cache.b, 0, sizeof(s_target_cache.b));
    s_recording_cache = true;
    
    // 绘制多个同心圆环（靶心的主要组成部分）
    draw_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                   LED_BOARD_WIDTH * 0.05f + BIAS, LED_BOARD_WIDTH * 0.117f - BIAS, 
                   color, 4);
    draw_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                   LED_BOARD_WIDTH * 0.183f + BIAS, LED_BOARD_WIDTH * 0.25f - BIAS, 
                   color, 4);
    draw_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                   LED_BOARD_WIDTH * 0.383f + BIAS, LED_BOARD_WIDTH * 0.45f - BIAS, 
                   color, 4);
    
    // 绘制十字线（靶心的辅助线）
    draw_arc_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                       LED_BOARD_WIDTH * 0.333f / 2, LED_BOARD_WIDTH / 2, 
                       M_PI * 0.5f - 7 * M_PI / 180, M_PI * 0.5f + 7 * M_PI / 180, 
                       color, 4);
    draw_arc_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                       LED_BOARD_WIDTH * 0.333f / 2, LED_BOARD_WIDTH / 2, 
                       M_PI - 7 * M_PI / 180, M_PI + 7 * M_PI / 180, 
                       color, 4);
    draw_arc_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                       LED_BOARD_WIDTH * 0.333f / 2, LED_BOARD_WIDTH / 2, 
                       M_PI  * 1.5f - 7 * M_PI / 180, M_PI * 1.5f + 7 * M_PI / 180, 
                       color, 4);
    draw_arc_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                       LED_BOARD_WIDTH * 0.333f /2, LED_BOARD_WIDTH / 2, 
                       0.001f, 7 * M_PI / 180, 
                       color, 4);
    draw_arc_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                       LED_BOARD_WIDTH * 0.333f /2, LED_BOARD_WIDTH / 2, 
                       - 7 * M_PI / 180, 0, 
                       color, 4);
    
    // 录制结束，后续调用直接从缓存读取
    s_recording_cache = false;
    
    led_strip_refresh(led_board);
    
    s_target_cache.cached_color = is_blue;
    s_target_cache.valid = true;
}

/**
 * @brief 显示臂部效果
 */
void show_arm(int is_blue, int group) {
    ESP_LOGI(TAG, "show_arm: color=%s group=%d", is_blue ? "BLUE" : (is_blue == 0 ? "RED" : "OFF"), group);
    Vec3 color;

      if (is_blue == 1) {
        color = Vec3(0, 0, 255);  // 蓝队
    } else if (is_blue == 0) {
        color = Vec3(255, 0, 0);  // 红队
    } else {
        color = Vec3(0, 0, 0);    // 其他值(如 -1)表示全灭
    }

    // 根据组别确定点亮范围
    int begin = 0;  // 起始LED索引
    int end = 0;    // 结束LED索引
    
    switch (group)
    {
    case 1:  // 第1组
        begin = 31;
        end = 58;
        break;
    case 2:  // 第2组
        begin = 23;
        end = 66;
        break;
    case 3:  // 第3组
        begin = 15;
        end = 74;
        break;
    case 4:  // 第4组
        begin = 7;
        end = 82;
        break;
    case 5:  // 第5组
        begin = 0;
        end = 89;
        break;
    default:
        break;
    }

    // 先清空所有LED
    for(int i = 0; i < LED_ARM_WIDTH * LED_ARM_HEIGHT; i++){
        led_strip_set_pixel(led_arm, i, 0, 0, 0);
    }
    for(int i = 0; i < LED_STRIP_LED_COUNT; i++){
        led_strip_set_pixel(led_strip, i, 0, 0, 0);
    }

    // 按组点亮LED灯带
    for(int i = begin; i < end; i++){
        led_strip_set_pixel(led_strip, i, color.x, color.y, color.z);
    }
    
    // 点亮臂部LED（根据组别数量）
    for(int i = LED_ARM_WIDTH * LED_ARM_HEIGHT - 1; i >= 0; i--){
        if(LED_ARM_WIDTH * LED_ARM_HEIGHT - i - 1 < group * 35)
            led_strip_set_pixel(led_arm, i, color.x, color.y, color.z);
    }

    // 刷新显示
    led_strip_refresh(led_strip);
    led_strip_refresh(led_arm);
}

// ==================== 动画播放函数 ====================

/**
 * @brief Bad Apple动画播放任务
 */
// void badapple_task(void *pvParameters) {
//     // 使用vTaskDelayUntil保证精确的帧率控制
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(VIDEO_TARGET_MS);  // 目标帧率

//     int current_payload_offset = 0;  // 当前数据偏移量
//     xLastWakeTime = xTaskGetTickCount();  // 记录开始时间

//     // 播放所有视频帧
//     for (int f = 0; f < VIDEO_FRAME_COUNT; f++) {
//         int pixel_idx = 0;
//         // 解压当前帧的RLE数据（游程编码压缩）
//         for (int i = 0; i < frame_sizes[f]; i += 2) {
//             uint8_t count = video_payload[current_payload_offset + i];      // 连续像素数量
//             uint8_t brightness = video_payload[current_payload_offset + i + 1];  // 亮度值

//             brightness *= 0.1;  // 调整亮度
            
//             // 设置连续的像素
//             for (int p = 0; p < count; p++) {
//                 if (pixel_idx < 1024) { // 32*32像素限制
//                     int x = pixel_idx % 32;   // 计算X坐标
//                     int y = pixel_idx / 32;   // 计算Y坐标
                    
//                     // 设置像素为灰度值
//                     set_pixel(led_board, x, y, Vec3(brightness, brightness, brightness), false);
//                 }
//                 pixel_idx++;
//             }
//         }
        
//         // 更新数据偏移量
//         current_payload_offset += frame_sizes[f];
//         // 刷新显示
//         led_strip_refresh(led_board);

//         // 阻塞直到下一帧时间到达，自动减去上面解压和刷屏的耗时
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// /**
//  * @brief 启动Bad Apple动画播放
//  */
// void show_badapple() {
//     // 创建高优先级任务在核心1上运行
//     xTaskCreatePinnedToCore(badapple_task, "BadAppleTask", 4096, NULL, 5, NULL, 1);
// }

// ==================== 箭头指示函数 ====================

constexpr static bool single_arrow[] = {
    0, 0, 1, 0, 0,
    0, 1, 1, 1, 0,
    1, 1, 0, 1, 1,
    1, 0, 0, 0, 1,
    0, 0, 0, 0, 0,
};

/**
 * @brief 显示箭头指示 (去除死循环，单帧无阻塞版)
 * @details 显示滚动的箭头动画效果，用于方向指示，配合FreeRTOS任务进行周期刷新
 * @param is_blue 是否为蓝色队伍
 */
void show_arrow(bool is_blue) {
    static uint8_t i = 0;  // 动画帧计数器
    
    // 矩阵尺寸常量
    const int MATRIX_W = 5;
    const int MATRIX_H = 33;
    const int TOTAL = MATRIX_W * MATRIX_H; // 165个LED

    // 遍历所有LED
    for (uint16_t j = 0; j < TOTAL; j++) {
        // 从预定义的箭头图案数据中获取位值
        bool bit = single_arrow[(j + i * 5) % 25];
        int intensity = bit ? 255 : 0;  // 根据位值设置亮度

        // 根据队伍颜色设置相应的颜色通道
        if (!is_blue) {
            // 红色队伍：红色箭头
            led_strip_set_pixel(led_arm, j, intensity, 0, 0);
        } else {
            // 蓝色队伍：蓝色箭头
            led_strip_set_pixel(led_arm, j, 0, 0, intensity);
        }
    }

    // 刷新显示
    led_strip_refresh(led_arm);
    
    // 更新动画帧计数器供下一帧使用
    i = (i + 1) % 5;
}

// ==================== 图形绘制函数 ====================

/**
 * @brief 绘制抗锯齿三角形
 * @param x1,y1 顶点1坐标
 * @param x2,y2 顶点2坐标
 * @param x3,y3 顶点3坐标
 * @param color RGB颜色值
 * @param ssaa_factor 超采样倍数
 */
void draw_triangle_ssaa(float x1, float y1, float x2, float y2, float x3, float y3, 
                        const Vec3& color, int ssaa_factor) { 
    
    // SSAA采样参数设置
    float sample_step = 1.0f / ssaa_factor;                   // 采样步长
    float sample_weight = 1.0f / (ssaa_factor * ssaa_factor); // 采样权重
    
    // 性能优化：计算三角形的边界框(Bounding Box)
    // 找出包含该三角形的最小矩形，避免遍历整个LED面板
    int min_x = std::max(0, (int)std::floor(std::min({x1, x2, x3})));
    int max_x = std::min((int)LED_BOARD_WIDTH - 1, (int)std::ceil(std::max({x1, x2, x3})));
    int min_y = std::max(0, (int)std::floor(std::min({y1, y2, y3})));
    int max_y = std::min((int)LED_BOARD_HEIGHT - 1, (int)std::ceil(std::max({y1, y2, y3})));

    // 遍历边界框内的每个像素
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            float r = 0;    // 红色分量累加器
            float g = 0;    // 绿色分量累加器
            float b = 0;    // 蓝色分量累加器
            
            // 对每个像素进行超采样
            for (int sy = 0; sy < ssaa_factor; sy++) {
                for (int sx = 0; sx < ssaa_factor; sx++) {
                    // 计算子像素坐标
                    float sub_x = x - 0.5f + (sx + 0.5f) * sample_step;
                    float sub_y = y - 0.5f + (sy + 0.5f) * sample_step;
                    
                    // 使用叉乘法(2D Cross Product)判断点是否在三角形内部
                    // 分别计算点到三条边的向量叉乘
                    float cp1 = (x2 - x1) * (sub_y - y1) - (y2 - y1) * (sub_x - x1);
                    float cp2 = (x3 - x2) * (sub_y - y2) - (y3 - y2) * (sub_x - x2);
                    float cp3 = (x1 - x3) * (sub_y - y3) - (y1 - y3) * (sub_x - x3);
                    
                    // 检查叉乘结果是否同号。如果全部大于等于0，或全部小于等于0，说明点在三角形内
                    bool has_neg = (cp1 < 0) || (cp2 < 0) || (cp3 < 0);
                    bool has_pos = (cp1 > 0) || (cp2 > 0) || (cp3 > 0);
                    
                    if (!(has_neg && has_pos)) {
                        r += color.x;
                        g += color.y;
                        b += color.z;
                    }
                }
            }
            
            // 优化：如果当前像素的所有子像素都不在三角形内（即纯黑），直接跳过
            if (r == 0 && g == 0 && b == 0) continue;

            // 应用采样权重计算平均颜色
            r *= sample_weight;
            g *= sample_weight;
            b *= sample_weight;

            // 设置像素颜色（加上0.5进行四舍五入）
            // 注：此处假设 led_board 为全局句柄，与您提供的 draw_ring_ssaa 内部调用保持一致
            set_pixel(led_board, x, y, Vec3(r + 0.5f, g + 0.5f, b + 0.5f), false);
        }
    }
}

void show_success(bool is_blue) {
    led_strip_clear(led_board);
    led_strip_clear(led_strip);
    led_strip_clear(led_arm);

    Vec3 color = is_blue ? Vec3(0, 0, 255) : Vec3(255, 0, 0);

    draw_triangle_ssaa(0, 0, 0, 9, 2, 1, color);
    draw_triangle_ssaa(0, 0, 9, 0, 2, 1, color);

    draw_triangle_ssaa(31, 0, 31, 9, 29, 1, color);
    draw_triangle_ssaa(31, 0, 22, 0, 29, 1, color);

    draw_triangle_ssaa(0, 31, 0, 22, 2, 30, color);
    draw_triangle_ssaa(0, 31, 9, 31, 2, 30, color);

    draw_triangle_ssaa(31, 31, 31, 22, 29, 30, color);
    draw_triangle_ssaa(31, 31, 22, 31, 29, 30, color);

    draw_ring_ssaa(LED_BOARD_WIDTH * 0.5f - 0.5f, LED_BOARD_HEIGHT * 0.5f - 0.5f, 
                   2 * LED_BOARD_WIDTH * 0.05f + BIAS, 
                   3 * LED_BOARD_WIDTH * 0.05f - BIAS, 
                   color, 4);

    for(int i = 0; i < 124; i++){
        led_strip_set_pixel(led_strip, i, !is_blue * 255, 0, is_blue * 255);
    }
    for(int i=154;i<LED_STRIP_LED_COUNT;i++){
        led_strip_set_pixel(led_strip, i, !is_blue * 255, 0, is_blue * 255);
    }

    for(int i = LED_ARM_WIDTH * LED_ARM_HEIGHT - 1; i >= 0; i--){
        led_strip_set_pixel(led_arm, i, !is_blue * 255, 0, is_blue * 255);
    }

    led_strip_refresh(led_board);
    led_strip_refresh(led_strip);
    led_strip_refresh(led_arm);
}