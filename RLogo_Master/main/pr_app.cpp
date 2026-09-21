#include "pr_app.h"

#include "esp_log.h"
#include "esp_event.h"

#include "firmware.h"
#include "tcp_protocol.h"

#include "device_registry.h"
#include "mech_engine.h"
#include "controller_gateway.h"
#include "comm_scheduler.h"
#include "hit_filter.h"

#include "LED.h"
#include "LED_Strip.h"
#include "pr_cmd_events.h"

static const char* TAG = "pr_app";

extern Config* config;
extern esp_event_loop_handle_t pr_events_loop_handle;
extern LED* led;

TcpProtocol* tcp_protocol = nullptr;
LED_Strip* led_strip = nullptr;

static void create_pr_event_loop(void)
{
    esp_event_loop_args_t loop_args = {
        .queue_size = 32,
        .task_name = "pr_events_loop",
        .task_priority = 6,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));
}


void pr_app_init(void)
{
    ESP_LOGI(TAG, "init");

    if (led == nullptr) {
        led = new LED(GPIO_NUM_2);
    }
    if (led_strip == nullptr) {
        led_strip = new LED_Strip(GPIO_NUM_10, 49);//根据实际情况来调整数据
    }

    create_pr_event_loop();

    static Firmware firmware;
    (void)firmware;
    hit_filter_init();

    // ===== TCP（可能阻塞在 establish_peer_list / reset_armour_id）=====
    // BLE 延迟到所有设备（5装甲板+电机）上线后，由 device_registry 启动
    tcp_protocol = new TcpProtocol();
    controller_gateway_start();

    // 注册事件...
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT,   TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT,     TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT,  TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT,   TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_NOTARGET_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_SET_SENSOR_PARAM_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_CALIBRATE_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_DECISION_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_THRESHOLD_CONFIG_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_EVENT,  TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_STOP_EVENT,   TcpProtocol::tx_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, nullptr));

    device_registry_init();
    mech_engine_init();

    ESP_LOGI(TAG, "init done");
}
void pr_app_start(void)
{
    ESP_LOGI(TAG, "start");
    led->set_mode(LED_MODE_FADE, 0);
    ESP_LOGI(TAG, "led_strip=%p, setting BLUE (0,0,50)", led_strip);
    led_strip->set_color(0, 0, 50);
    led_strip->refresh();
    ESP_LOGI(TAG, "led_strip set_color done");
    device_registry_start();
    mech_engine_start();
    comm_scheduler_start();

    ESP_LOGI(TAG, "start done");
}
