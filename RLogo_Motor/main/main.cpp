
#include "main.h"
TcpProtocol *tcp_protocol = nullptr;
/**
 * @note beacon timeout处理函数
 */
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    // 立刻停止电机
    ESP_LOGE(TAG, "beacon_timeout, stopping motor");
    esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY);
    return;
}

// define event base handler function
static void PRM_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // set handler_args to motor_3508
    Motor *motor_3508 = (Motor *)handler_args;
    esp_err_t err = ESP_OK;
    if (event_base == PRC)
    {
        if (event_id == OTA_BEGIN_EVENT)
        {
            ESP_LOGI(TAG, "OTA_BEGIN_EVENT, stopping motor");
            // 同 PRM_STOP_EVENT
            motor_3508->disable_motor(CONFIG_DEFAULT_MOTOR_ID);
            return;
        }
        else if (event_id == CONFIG_EVENT)
        {
            CONFIG_EVENT_DATA *config_data = (CONFIG_EVENT_DATA *)event_data;
            PowerRune_Motor_config_info_t *ci = &config_data->config_motor_info;
            ESP_LOGI(TAG, "CONFIG_EVENT, PID: Kp:%f Ki:%f Kd:%f Imax:%f Dmax:%f Outmax:%f", ci->kp, ci->ki, ci->kd, ci->i_max, ci->d_max, ci->out_max);
            if (ci->kp > 0.01f && ci->out_max > 1.0f) {
                motor_3508->reset(ci->kp, ci->ki, ci->kd, ci->i_max, ci->d_max, ci->out_max);
            } else {
                ESP_LOGW(TAG, "CONFIG_EVENT has invalid PID params, ignoring");
            }
            return;
        }
    }
    switch (event_id)
    {
    case PRM_UNLOCK_EVENT:
    {
        ESP_LOGI(TAG, "PRM_UNLOCK_EVENT");

        err = motor_3508->unlock_motor(CONFIG_DEFAULT_MOTOR_ID);
        // post PRM_UNLOCK_DONE_EVENT
        PRM_UNLOCK_DONE_EVENT_DATA unlock_done_data = {
            .status = err,
        };
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, &unlock_done_data, sizeof(PRM_UNLOCK_DONE_EVENT_DATA), portMAX_DELAY));
        break;
    }
    case PRM_START_EVENT:
    {
        struct PRM_START_EVENT_DATA *start_data = (struct PRM_START_EVENT_DATA *)event_data;
        ESP_LOGI(TAG, "Starting motor with mode: %d, clockwise: %d", start_data->mode, start_data->clockwise);

        if (motor_3508->get_motor_status(CONFIG_DEFAULT_MOTOR_ID) == MOTOR_DISABLED_LOCKED)
        {
            // Unlock is an explicit control-plane command. Never turn a
            // reordered or malformed START into an implicit motor enable.
            ESP_LOGW(TAG, "Motor is locked; rejecting START until UNLOCK arrives");
            err = ESP_ERR_INVALID_STATE;
            PRM_START_DONE_EVENT_DATA start_done_data = {
                .status = err,
                .mode = start_data->mode,
            };
            ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM,
                                               PRM_START_DONE_EVENT,
                                               &start_done_data,
                                               sizeof(start_done_data),
                                               portMAX_DELAY));
            break;
        }

        if (start_data->mode == PRA_RUNE_SMALL_MODE)
        {
            err = motor_3508->set_speed(CONFIG_DEFAULT_MOTOR_ID, start_data->clockwise ? 1140 : -1140);
        }
        else if (start_data->mode == PRA_RUNE_BIG_MODE)
        {
            ESP_LOGI(TAG, "Amplitude: %f, omega: %f, offset: %f", start_data->amplitude, start_data->omega, start_data->offset);
            if (start_data->clockwise == PRM_DIRECTION_CLOCKWISE)
            {
                start_data->amplitude = -start_data->amplitude;
                start_data->offset = -start_data->offset;
            }
            err = motor_3508->set_speed(CONFIG_DEFAULT_MOTOR_ID, start_data->amplitude, start_data->omega, start_data->offset);
        }
        else
        {
            err = ESP_ERR_INVALID_ARG;
        };
        PRM_START_DONE_EVENT_DATA start_done_data = {
            .status = err,
            .mode = start_data->mode,
        };
        // post PRM_START_DONE_EVENT
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_DONE_EVENT, &start_done_data, sizeof(PRM_START_DONE_EVENT_DATA), portMAX_DELAY));
        break;
    }
    case PRM_STOP_EVENT:
        ESP_LOGI(TAG, "PRM_STOP_EVENT");
        motor_3508->disable_motor(CONFIG_DEFAULT_MOTOR_ID);
        break;
    default:
        break;
    };
};

extern "C" void app_main(void)
{
    // LED and LED Strip init
    led = new LED(GPIO_NUM_2, 1, LED_MODE_ON, 1);

    // 启动事件循环
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "pr_events_loop",
        .task_priority = 5,
        .task_stack_size = 8192,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // Firmware init
    Firmware firmware;

    // TCP init
    tcp_protocol = new TcpProtocol(beacon_timeout);

    ESP_LOGI(TAG, "TCP Start");

    uint8_t motor_counts = 1;
    uint8_t id[1] = {1};

    const auto *mcfg = config->get_config_info_pt();
    float kp      = (mcfg->kp > 0.01f)      ? mcfg->kp      : 4.0f;
    float ki      = (mcfg->ki > 0.001f)     ? mcfg->ki      : 0.2f;
    float kd      = (mcfg->kd > 0.01f)      ? mcfg->kd      : 0.4f;
    float i_max   = (mcfg->i_max > 1.0f)    ? mcfg->i_max   : 2000.0f;
    float d_max   = (mcfg->d_max > 1.0f)    ? mcfg->d_max   : 2000.0f;
    float out_max = (mcfg->out_max > 1.0f)  ? mcfg->out_max : 6000.0f;

    if (mcfg->kp < 0.01f || mcfg->out_max < 1.0f) {
        ESP_LOGW(TAG, "NVS motor config invalid (kp=%.2f out_max=%.1f), using defaults", mcfg->kp, mcfg->out_max);
    }

    Motor motor_3508(id, motor_counts, GPIO_NUM_4, GPIO_NUM_5,
                     kp, ki, kd, 2000.0f, i_max, d_max, out_max);

    // 注册大符通讯协议事件
    // 发送事件
#if CONFIG_POWERRUNE_TYPE == 1 // RLogo
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_DISCONNECT_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_SPEED_STABLE_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_DONE_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE != 1 // 除了主控外的设备
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, TcpProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, TcpProtocol::tx_event_handler, NULL));
#endif

    // register event PRM handler, transfer motor_3508 to handler_args
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508));
    // LED闪烁
    led->set_mode(LED_MODE_FADE, 1);


    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

