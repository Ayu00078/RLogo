#include "firmware.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "PowerRune_Events.h"
#include <string.h>

static const char *TAG = "firmware";

ESP_EVENT_DEFINE_BASE(PRC);
ESP_EVENT_DEFINE_BASE(PRA);
ESP_EVENT_DEFINE_BASE(PRM);

Config *config = nullptr;
esp_event_loop_handle_t pr_events_loop_handle = nullptr;

#if CONFIG_POWERRUNE_TYPE == 0
PowerRune_Armour_config_info_t Config::config_info = {};
#elif CONFIG_POWERRUNE_TYPE == 1
PowerRune_Rlogo_config_info_t Config::config_info = {};
PowerRune_Armour_config_info_t Config::config_armour_info[5] = {};
PowerRune_Motor_config_info_t Config::config_motor_info = {};
#elif CONFIG_POWERRUNE_TYPE == 2
PowerRune_Motor_config_info_t Config::config_info = {};
#endif
PowerRune_Common_config_info_t Config::config_common_info = {};
const char *Config::PowerRune_description = "PowerRune v1.2";

Config::Config() { read(); }

#if CONFIG_POWERRUNE_TYPE == 0
const PowerRune_Armour_config_info_t *Config::get_config_info_pt() { return &config_info; }
#elif CONFIG_POWERRUNE_TYPE == 1
PowerRune_Rlogo_config_info_t *Config::get_config_info_pt() { return &config_info; }
PowerRune_Armour_config_info_t *Config::get_config_armour_info_pt(uint8_t index) {
    return (index < 5) ? &config_armour_info[index] : nullptr;
}
PowerRune_Motor_config_info_t *Config::get_config_motor_info_pt() { return &config_motor_info; }
PowerRune_Common_config_info_t *Config::get_config_common_info_pt() { return &config_common_info; }
#elif CONFIG_POWERRUNE_TYPE == 2
const PowerRune_Motor_config_info_t *Config::get_config_info_pt() { return &config_info; }
#endif

#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2
const PowerRune_Common_config_info_t *Config::get_config_common_info_pt() { return &config_common_info; }
#endif

esp_err_t Config::read() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS read failed (%s), using defaults", esp_err_to_name(err));
#if CONFIG_POWERRUNE_TYPE == 0
        config_info.brightness = 128;
        config_info.armour_id = 1;
        config_info.brightness_proportion_matrix = 100;
        config_info.brightness_proportion_edge = 100;
#elif CONFIG_POWERRUNE_TYPE == 1
        config_info.brightness = 128;
        config_motor_info = {10.0f, 0.5f, 5.0f, 1000.0f, 500.0f, 10000.0f, 1, 0};
#elif CONFIG_POWERRUNE_TYPE == 2
        config_info = {10.0f, 0.5f, 5.0f, 1000.0f, 500.0f, 10000.0f, 1, 0};
#endif
        strncpy(config_common_info.SSID, CONFIG_DEFAULT_UPDATE_SSID, sizeof(config_common_info.SSID) - 1);
        strncpy(config_common_info.SSID_pwd, CONFIG_DEFAULT_UPDATE_PWD, sizeof(config_common_info.SSID_pwd) - 1);
        return err;
    }
    size_t len;
#if CONFIG_POWERRUNE_TYPE == 0
    len = sizeof(config_info);
    nvs_get_blob(nvs, "cfg", &config_info, &len);
#elif CONFIG_POWERRUNE_TYPE == 1
    len = sizeof(config_info);
    nvs_get_blob(nvs, "cfg", &config_info, &len);
    len = sizeof(config_motor_info);
    nvs_get_blob(nvs, "cfg_motor", &config_motor_info, &len);
    for (int i = 0; i < 5; i++) {
        char key[16];
        snprintf(key, sizeof(key), "cfg_a%d", i);
        len = sizeof(config_armour_info[i]);
        nvs_get_blob(nvs, key, &config_armour_info[i], &len);
    }
#elif CONFIG_POWERRUNE_TYPE == 2
    len = sizeof(config_info);
    nvs_get_blob(nvs, "cfg", &config_info, &len);
#endif
    len = sizeof(config_common_info);
    nvs_get_blob(nvs, "cfg_common", &config_common_info, &len);
    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t Config::save() {
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "NVS open");
#if CONFIG_POWERRUNE_TYPE == 0
    nvs_set_blob(nvs, "cfg", &config_info, sizeof(config_info));
#elif CONFIG_POWERRUNE_TYPE == 1
    nvs_set_blob(nvs, "cfg", &config_info, sizeof(config_info));
    nvs_set_blob(nvs, "cfg_motor", &config_motor_info, sizeof(config_motor_info));
    for (int i = 0; i < 5; i++) {
        char key[16];
        snprintf(key, sizeof(key), "cfg_a%d", i);
        nvs_set_blob(nvs, key, &config_armour_info[i], sizeof(config_armour_info[i]));
    }
#elif CONFIG_POWERRUNE_TYPE == 2
    nvs_set_blob(nvs, "cfg", &config_info, sizeof(config_info));
#endif
    nvs_set_blob(nvs, "cfg_common", &config_common_info, sizeof(config_common_info));
    nvs_commit(nvs);
    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t Config::reset() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_PR_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    memset(&config_info, 0, sizeof(config_info));
    memset(&config_common_info, 0, sizeof(config_common_info));
    return read();
}

void Config::global_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data) {
    if (id == CONFIG_EVENT && event_data) {
        auto *d = (CONFIG_EVENT_DATA *)event_data;
#if CONFIG_POWERRUNE_TYPE == 0
        config_info = d->config_armour_info;
#elif CONFIG_POWERRUNE_TYPE == 1
        config_info = d->config_rlogo_info;
        config_motor_info = d->config_motor_info;
#elif CONFIG_POWERRUNE_TYPE == 2
        config_info = d->config_motor_info;
#endif
        config_common_info = d->config_common_info;
        save();
        CONFIG_COMPLETE_EVENT_DATA c = {};
        c.status = ESP_OK;
        esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, &c, sizeof(c), portMAX_DELAY);
    }
}

// ===================== Firmware =====================

esp_app_desc_t Firmware::app_desc = {};
esp_netif_t *Firmware::netif = nullptr;
EventGroupHandle_t Firmware::ota_event_group = nullptr;
QueueHandle_t Firmware::ota_complete_queue = nullptr;

#if CONFIG_POWERRUNE_TYPE != 1
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *evt = (ip_event_got_ip_t *)__builtin_frame_address(0);
        (void)evt;
        ESP_LOGI(TAG, "Got IP from AP");
    }
}
#endif

Firmware::Firmware() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS...");
        nvs_flash_erase();
        nvs_flash_init();
    }

    static Config cfg;
    config = &cfg;

    const esp_app_desc_t *desc = esp_app_get_description();
    memcpy(&app_desc, desc, sizeof(app_desc));
    ESP_LOGI(TAG, "Project: %s  Version: %s", app_desc.project_name, app_desc.version);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_POWERRUNE_TYPE == 1
    netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_cfg = {};
    strcpy((char *)ap_cfg.ap.ssid, "PowerRune");
    ap_cfg.ap.ssid_len = 9;
    ap_cfg.ap.max_connection = 10;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    ap_cfg.ap.channel = CONFIG_TCP_WIFI_CHANNEL;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi AP started: SSID=PowerRune  CH=%d", CONFIG_TCP_WIFI_CHANNEL);
#else
    netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t sta_cfg = {};
    strcpy((char *)sta_cfg.sta.ssid, "PowerRune");
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi STA started, connecting to PowerRune AP...");
#endif

    ota_event_group = xEventGroupCreate();
    ota_complete_queue = xQueueCreate(1, sizeof(esp_err_t));
}

esp_err_t Firmware::wifi_ota_init() { return ESP_OK; }
esp_err_t Firmware::wifi_connect(const PowerRune_Common_config_info_t *, uint8_t) { return ESP_OK; }
void Firmware::wifi_disconnect() {}
void Firmware::http_cleanup(esp_http_client_handle_t client) {
    if (client) esp_http_client_cleanup(client);
}

void Firmware::task_OTA(void *args) {
    ESP_LOGW(TAG, "OTA stub – not implemented in TCP mode");
    vTaskDelete(NULL);
}

void Firmware::global_system_event_handler(void *, esp_event_base_t, int32_t, void *) {}

void Firmware::global_pr_event_handler(void *, esp_event_base_t base, int32_t id, void *) {
    if (id == OTA_BEGIN_EVENT) {
        ESP_LOGI(TAG, "OTA begin event received");
        xTaskCreate(task_OTA, "ota_task", 8192, (void *)1, 5, NULL);
    }
}
