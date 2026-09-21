/**
 * @file led_controller.h
 * @brief LED控制器头文件
 */
#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_STRIP_USE_DMA  1
#define LED_STRIP_LED_COUNT 200
#define LED_BOARD_LED_COUNT 1024
#define LED_STRIP_MEMORY_BLOCK_WORDS 0

#define LED_ARM_GPIO_PIN  12
#define LED_BOARD_GPIO_PIN  13 
#define LED_STRIP_GPIO_PIN  14
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define LED_ARM_WIDTH 5
#define LED_ARM_HEIGHT 33
#define LED_BOARD_WIDTH 32
#define LED_BOARD_HEIGHT 32

#define MATRIX_REFRESH_PERIOD 100

extern led_strip_handle_t led_strip;
extern led_strip_handle_t led_board;
extern led_strip_handle_t led_arm;
extern led_strip_handle_t led_esp32;

enum DEBUG_LED_MODE {
    DEBUG_LED_MODE_OFF = 0,
    DEBUG_LED_MODE_WHITE = 1,
    DEBUG_LED_MODE_RED = 2,
    DEBUG_LED_MODE_GREEN = 3,
    DEBUG_LED_MODE_BLUE = 4
};

class led_config_t{
public:
    led_config_t():
        strip_gpio_num(LED_STRIP_GPIO_PIN),
        max_leds(LED_STRIP_LED_COUNT),
        mem_block_symbols(LED_STRIP_MEMORY_BLOCK_WORDS),
        with_dma(LED_STRIP_USE_DMA) {}

    led_config_t(int gpio, int leds, int mem_words, bool dma):
        strip_gpio_num(gpio), 
        max_leds(leds), 
        mem_block_symbols(mem_words), 
        with_dma(dma) {};

    int strip_gpio_num;
    uint32_t max_leds;
    size_t mem_block_symbols;
    bool with_dma;
};

void init_led();

class Vec3 {
public:
    Vec3() : x(0), y(0), z(0) {}
    Vec3(const int x, const int y, const int z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return { x + b.x, y + b.y, z + b.z }; }
    Vec3 operator-(const Vec3& b) const { return { x - b.x, y - b.y, z - b.z }; }
    Vec3 operator*(const int b) const { return { x * b, y * b, z * b }; }
    Vec3 operator*(const float b) const { return { static_cast<int>(x * b), static_cast<int>(y * b), static_cast<int>(z * b) }; }
    Vec3 operator/(const int b) const { return { x / b, y / b, z / b }; }
    [[nodiscard]] Vec3 hadamard(const Vec3& b) const { return {x * b.x, y * b.y, z * b.z}; }
    int operator*(const Vec3& b) const { return x * b.x + y * b.y + z * b.z; }
    bool operator==(const Vec3& b) const { return x == b.x && y == b.y && z == b.z; }

    int x, y, z;
};

void set_pixel(led_strip_handle_t led, const int& x, const int& y, const Vec3& color, bool skip_zero = true);
void draw_ring(float cx, float cy, float inner_radius, float outer_radius, const Vec3& color);
void draw_ring_ssaa(float cx, float cy, float inner_radius, float outer_radius, const Vec3& color, int ssaa_factor = 2);
void draw_arc_ring_ssaa(float cx, float cy, float inner_radius, float outer_radius,
                                float start_angle, float end_angle, const Vec3& color, int ssaa_factor = 2);
void draw_triangle_ssaa(float x1, float y1, float x2, float y2, float x3, float y3, 
                        const Vec3& color, int ssaa_factor = 2);

void show_result(int is_blue, int num); 
void show_target(int is_blue);
void show_arm(int is_blue, int group);
void show_badapple();
void show_arrow(bool is_blue);
void show_success(bool is_blue);

#endif //LED_CONTROLLER_H