#include "pr_protocol_legacy.h"
#include "pr_types.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "nvs_flash.h"

static const char *TAG = "controller";

static constexpr EventBits_t WIFI_READY_BIT = BIT0;
static constexpr uint32_t TCP_RETRY_MS = 1000;
static constexpr uint32_t TCP_WARNING_INTERVAL_MS = 5000;

static EventGroupHandle_t s_wifi_events = nullptr;
static SemaphoreHandle_t s_socket_mutex = nullptr;
static int s_socket = -1;
static bool s_debug = false;
static TickType_t s_last_master_warning = 0;
static bool s_master_warning_sent = false;

static pr_controller_run_config_t s_config = {
    PR_CONTROLLER_ADDRESS,
    0, // red
    0, // big
    0, // no loop
    PR_RUN_DIR_CW,
};

static const char *level_name(uint8_t level)
{
    switch (level) {
    case PR_CONTROLLER_LOG_INFO: return "INFO";
    case PR_CONTROLLER_LOG_WARN: return "WARN";
    case PR_CONTROLLER_LOG_ERROR: return "ERROR";
    case PR_CONTROLLER_LOG_DEBUG: return "DEBUG";
    default: return "LOG";
    }
}

static void print_line(const char *prefix, const char *text)
{
    printf("%s%s\r\n", prefix, text ? text : "");
    fflush(stdout);
}

static bool is_master_connection_log(const char *text)
{
    static constexpr char prefix[] = "TCP connected to Master; controller address=";
    return text != nullptr && strncmp(text, prefix, sizeof(prefix) - 1U) == 0;
}

static void print_master_connection_info(void)
{
    printf("[INFO] TCP connected to Master; controller address=0x%02X\r\n",
           PR_CONTROLLER_ADDRESS);
    fflush(stdout);
    s_master_warning_sent = false;
}

static void print_master_disconnected_warning(void)
{
    const TickType_t now = xTaskGetTickCount();
    if (!s_master_warning_sent ||
        (TickType_t)(now - s_last_master_warning) >= pdMS_TO_TICKS(TCP_WARNING_INTERVAL_MS)) {
        print_line("[WARN] ", "TCP not connected to Master; retrying");
        s_last_master_warning = now;
        s_master_warning_sent = true;
    }
}

static void wifi_event_handler(void *, esp_event_base_t event_base,
                               int32_t event_id, void *)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_READY_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_events, WIFI_READY_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected to %s", PR_WIFI_SSID);
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                wifi_event_handler, nullptr));

    wifi_config_t config = {};
    strlcpy((char *)config.sta.ssid, PR_WIFI_SSID, sizeof(config.sta.ssid));
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static bool send_all(int sock, const uint8_t *data, size_t length)
{
    size_t sent = 0;
    while (sent < length) {
        const int result = send(sock, data + sent, length - sent, 0);
        if (result <= 0) return false;
        sent += (size_t)result;
    }
    return true;
}

static bool send_packet(uint8_t event_id, const void *payload, uint16_t payload_len)
{
    if (payload == nullptr || payload_len == 0 || payload_len > PR_TCP_DATA_LEN) {
        return false;
    }

    pr_packet_t packet;
    pr_packet_init(&packet, PR_BASE_COMMON, event_id,
                   ((const uint8_t *)payload)[0]);
    if (!pr_packet_set_payload(&packet, payload, payload_len)) return false;

    uint8_t encoded[PR_TCP_FRAME_SIZE];
    size_t encoded_size = 0;
    if (!pr_packet_encode(&packet, encoded, sizeof(encoded), &encoded_size)) {
        return false;
    }

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return false;
    }

    const int sock = s_socket;
    const bool ok = sock >= 0 && send_all(sock, encoded, encoded_size);
    if (!ok && sock >= 0 && s_socket == sock) {
        s_socket = -1;
    }
    xSemaphoreGive(s_socket_mutex);
    return ok;
}

static bool send_simple(uint8_t event_id)
{
    const uint8_t address = PR_CONTROLLER_ADDRESS;
    return send_packet(event_id, &address, sizeof(address));
}

static bool recv_all(int sock, uint8_t *data, size_t length)
{
    size_t received = 0;
    while (received < length) {
        const int result = recv(sock, data + received, length - received, 0);
        if (result <= 0) return false;
        received += (size_t)result;
    }
    return true;
}

static void handle_packet(const uint8_t *data)
{
    pr_packet_t packet;
    if (!pr_packet_decode(data, PR_TCP_FRAME_SIZE, &packet)) {
        print_line("[ERROR] ", "invalid TCP frame from Master");
        return;
    }
    if (!pr_packet_is(&packet, PR_BASE_COMMON, PRC_EVT_CONTROLLER_LOG) ||
        packet.event_data_len < 2) {
        return;
    }

    const uint8_t level = packet.event_data[1];
    if (level == PR_CONTROLLER_LOG_DEBUG && !s_debug) return;

    char text[PR_CONTROLLER_LOG_MAX_TEXT + 1U] = {};
    const size_t text_len = packet.event_data_len - 2U;
    const size_t copy_len = text_len < sizeof(text) - 1U ? text_len : sizeof(text) - 1U;
    memcpy(text, &packet.event_data[2], copy_len);
    text[copy_len] = '\0';
    // Master sends the same connection information after receiving HELLO.
    // The Controller prints the state transition locally, so suppress this
    // duplicate packet on the virtual serial port.
    if (level == PR_CONTROLLER_LOG_INFO && is_master_connection_log(text)) return;
    printf("[%s] %s\r\n", level_name(level), text);
    fflush(stdout);
}

static bool connect_to_master(int *connected_socket)
{
    *connected_socket = -1;
    if ((xEventGroupGetBits(s_wifi_events) & WIFI_READY_BIT) == 0) return false;

    const int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) return false;

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(PR_TCP_PORT);
    address.sin_addr.s_addr = inet_addr(PR_MASTER_IP);
    const int result = connect(sock, (sockaddr *)&address, sizeof(address));
    if (result != 0) {
        close(sock);
        return false;
    }

    int no_delay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        close(sock);
        return false;
    }
    s_socket = sock;
    xSemaphoreGive(s_socket_mutex);
    *connected_socket = sock;

    const pr_controller_hello_t hello = {
        PR_CONTROLLER_ADDRESS,
        PR_PROTOCOL_VERSION,
    };
    if (!send_packet(PRC_EVT_CONTROLLER_HELLO, &hello, sizeof(hello))) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
        *connected_socket = -1;
        return false;
    }
    print_master_connection_info();
    return true;
}

static void tcp_task(void *)
{
    uint8_t frame[PR_TCP_FRAME_SIZE] = {};

    while (true) {
        int sock = -1;
        if (!connect_to_master(&sock)) {
            print_master_disconnected_warning();
            vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_MS));
            continue;
        }

        while (true) {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(sock, &read_set);
            timeval timeout = {1, 0};
            const int selected = select(sock + 1, &read_set, nullptr, nullptr, &timeout);
            if (selected < 0 || (selected > 0 && !recv_all(sock, frame, sizeof(frame)))) {
                break;
            }
            if (selected == 0) {
                send_simple(PRC_EVT_CONTROLLER_PING);
            } else {
                handle_packet(frame);
            }

            if (s_socket != sock) break;
        }

        if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(250)) == pdTRUE) {
            if (s_socket == sock) s_socket = -1;
            xSemaphoreGive(s_socket_mutex);
        }
        shutdown(sock, SHUT_RDWR);
        close(sock);
        print_master_disconnected_warning();
        vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_MS));
    }
}

static bool parse_bool_value(const char *value, uint8_t *out)
{
    if (strcasecmp(value, "1") == 0 || strcasecmp(value, "on") == 0 ||
        strcasecmp(value, "yes") == 0 || strcasecmp(value, "true") == 0) {
        *out = 1;
        return true;
    }
    if (strcasecmp(value, "0") == 0 || strcasecmp(value, "off") == 0 ||
        strcasecmp(value, "no") == 0 || strcasecmp(value, "false") == 0) {
        *out = 0;
        return true;
    }
    return false;
}

static bool parse_armour_id(const char *value, uint8_t *out)
{
    if (!value || !out) return false;
    char *end = nullptr;
    errno = 0;
    const unsigned long parsed = strtoul(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < 1 ||
        parsed > PR_ARMOUR_COUNT) {
        return false;
    }
    *out = (uint8_t)parsed;
    return true;
}

static bool parse_threshold_x100(const char *value, uint16_t *out)
{
    if (!value || !out) return false;
    char *end = nullptr;
    errno = 0;
    const float parsed = strtof(value, &end);
    if (errno != 0 || end == value || *end != '\0' || !isfinite(parsed) ||
        parsed < 0.0f || parsed > 255.0f) {
        return false;
    }
    const long scaled = lroundf(parsed * 100.0f);
    if (scaled < 0 || scaled > 25500) return false;
    *out = (uint16_t)scaled;
    return true;
}

static bool parse_config_token(char *token, pr_controller_run_config_t *config)
{
    char *separator = strchr(token, '=');
    if (separator == nullptr) return false;
    *separator = '\0';
    const char *key = token;
    const char *value = separator + 1;
    uint8_t parsed = 0;

    if (strcasecmp(key, "color") == 0) {
        if (strcasecmp(value, "red") == 0 || strcmp(value, "0") == 0) parsed = 0;
        else if (strcasecmp(value, "blue") == 0 || strcmp(value, "1") == 0) parsed = 1;
        else return false;
        config->color = parsed;
    } else if (strcasecmp(key, "mode") == 0) {
        if (strcasecmp(value, "big") == 0 || strcmp(value, "0") == 0) parsed = 0;
        else if (strcasecmp(value, "small") == 0 || strcmp(value, "1") == 0) parsed = 1;
        else return false;
        config->mode = parsed;
    } else if (strcasecmp(key, "loop") == 0) {
        if (!parse_bool_value(value, &parsed)) return false;
        config->loop = parsed;
    } else if (strcasecmp(key, "dir") == 0 || strcasecmp(key, "direction") == 0) {
        if (strcasecmp(value, "cw") == 0 || strcasecmp(value, "clockwise") == 0 ||
            strcmp(value, "0") == 0) parsed = PR_RUN_DIR_CW;
        else if (strcasecmp(value, "ccw") == 0 ||
                 strcasecmp(value, "anticlockwise") == 0 || strcmp(value, "1") == 0) {
            parsed = PR_RUN_DIR_CCW;
        }
        else if (strcasecmp(value, "cs") == 0 || strcmp(value, "2") == 0) {
            parsed = PR_RUN_DIR_CS;
        }
        else if (!parse_bool_value(value, &parsed)) return false;
        config->dir = parsed;
    } else {
        return false;
    }
    return true;
}

static bool parse_config_tokens(char *rest, pr_controller_run_config_t *config, bool *changed)
{
    *changed = false;
    char *save = nullptr;
    for (char *token = strtok_r(rest, " \t", &save); token != nullptr;
         token = strtok_r(nullptr, " \t", &save)) {
        if (!parse_config_token(token, config)) return false;
        *changed = true;
    }
    return true;
}

static void print_config(void)
{
    const char *direction = s_config.dir == PR_RUN_DIR_CW ? "cw" :
                            s_config.dir == PR_RUN_DIR_CCW ? "ccw" : "cs";
    printf("[INFO] CONFIG color=%s mode=%s loop=%s dir=%s\r\n",
           s_config.color ? "blue" : "red",
           s_config.mode ? "small" : "big",
           s_config.loop ? "on" : "off", direction);
}

static void print_help(void)
{
    print_line("[INFO] ", "Commands:");
    print_line("        ", "CONFIG color=red|blue mode=big|small loop=on|off dir=cw|ccw|cs");
    print_line("        ", "START (or RUN), STOP, UNLOCK, OTA, STATUS");
    print_line("        ", "REDLED ON, REDLED OFF");
    print_line("        ", "DEBUG ON, DEBUG OFF");
    print_line("        ", "THRESHOLD SET <armour 1-5> <peak 0.00-255.00>");
    print_line("        ", "THRESHOLD OFF <armour 1-5>, THRESHOLD SHOW [armour]");
    print_line("        ", "HELP");
}

static void cli_task(void *)
{
    char line[256] = {};
    print_line("[INFO] ", "Controller ready; type HELP for commands");
    print_config();

    while (true) {
        if (fgets(line, sizeof(line), stdin) == nullptr) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        line[strcspn(line, "\r\n")] = '\0';
        char *save = nullptr;
        char *command = strtok_r(line, " \t", &save);
        if (command == nullptr) continue;

        if (strcasecmp(command, "HELP") == 0) {
            print_help();
        } else if (strcasecmp(command, "STATUS") == 0) {
            if (!send_simple(PRC_EVT_CONTROLLER_STATUS_REQUEST)) {
                print_line("[WARN] ", "STATUS not sent: TCP is not connected");
            }
        } else if (strcasecmp(command, "REDLED") == 0) {
            char *value = strtok_r(nullptr, " \t", &save);
            uint8_t event_id = 0;
            const char *state = nullptr;
            if (value != nullptr && strcasecmp(value, "ON") == 0) {
                event_id = PRC_EVT_CONTROLLER_REDLED_ON;
                state = "ON";
            } else if (value != nullptr && strcasecmp(value, "OFF") == 0) {
                event_id = PRC_EVT_CONTROLLER_REDLED_OFF;
                state = "OFF";
            } else {
                print_line("[WARN] ", "Usage: REDLED ON|OFF");
                continue;
            }
            if (send_simple(event_id)) {
                printf("[INFO] REDLED %s sent\r\n", state);
            } else {
                printf("[WARN] REDLED %s not sent: TCP is not connected\r\n", state);
            }
        } else if (strcasecmp(command, "DEBUG") == 0) {
            char *value = strtok_r(nullptr, " \t", &save);
            uint8_t enabled = 0;
            if (value == nullptr || !parse_bool_value(value, &enabled)) {
                print_line("[WARN] ", "Usage: DEBUG ON|OFF");
                continue;
            }
            s_debug = enabled != 0;
            pr_controller_debug_t debug = {PR_CONTROLLER_ADDRESS, enabled};
            if (send_packet(PRC_EVT_CONTROLLER_DEBUG, &debug, sizeof(debug))) {
                printf("[INFO] DEBUG %s; detailed logs %s\r\n",
                       enabled ? "ON" : "OFF", enabled ? "visible" : "hidden");
            } else {
                print_line("[WARN] ", "DEBUG changed locally; TCP is not connected");
            }
        } else if (strcasecmp(command, "THRESHOLD") == 0) {
            char *subcommand = strtok_r(nullptr, " \t", &save);
            if (subcommand == nullptr) {
                print_line("[WARN] ", "Usage: THRESHOLD SET|OFF|SHOW ...");
                continue;
            }

            if (strcasecmp(subcommand, "SET") == 0) {
                char *id_text = strtok_r(nullptr, " \t", &save);
                char *value_text = strtok_r(nullptr, " \t", &save);
                uint8_t armour_id = 0;
                uint16_t threshold_x100 = 0;
                if (!parse_armour_id(id_text, &armour_id) ||
                    !parse_threshold_x100(value_text, &threshold_x100)) {
                    print_line("[WARN] ", "Usage: THRESHOLD SET <armour 1-5> <peak 0.00-255.00>");
                    continue;
                }
                const pr_controller_threshold_set_t request = {
                    PR_CONTROLLER_ADDRESS, armour_id, 1, threshold_x100};
                if (send_packet(PRC_EVT_CONTROLLER_THRESHOLD_SET,
                                &request, sizeof(request))) {
                    print_line("[INFO] ", "THRESHOLD SET sent");
                } else {
                    print_line("[WARN] ", "THRESHOLD SET not sent: TCP is not connected");
                }
            } else if (strcasecmp(subcommand, "OFF") == 0) {
                char *id_text = strtok_r(nullptr, " \t", &save);
                uint8_t armour_id = 0;
                if (!parse_armour_id(id_text, &armour_id)) {
                    print_line("[WARN] ", "Usage: THRESHOLD OFF <armour 1-5>");
                    continue;
                }
                const pr_controller_threshold_set_t request = {
                    PR_CONTROLLER_ADDRESS, armour_id, 0, 0};
                if (send_packet(PRC_EVT_CONTROLLER_THRESHOLD_OFF,
                                &request, sizeof(request))) {
                    print_line("[INFO] ", "THRESHOLD OFF sent");
                } else {
                    print_line("[WARN] ", "THRESHOLD OFF not sent: TCP is not connected");
                }
            } else if (strcasecmp(subcommand, "SHOW") == 0) {
                char *id_text = strtok_r(nullptr, " \t", &save);
                uint8_t armour_id = 0;
                if (id_text != nullptr && !parse_armour_id(id_text, &armour_id)) {
                    print_line("[WARN] ", "Usage: THRESHOLD SHOW [armour 1-5]");
                    continue;
                }
                const pr_controller_threshold_query_t request = {
                    PR_CONTROLLER_ADDRESS, armour_id};
                if (!send_packet(PRC_EVT_CONTROLLER_THRESHOLD_SHOW,
                                 &request, sizeof(request))) {
                    print_line("[WARN] ", "THRESHOLD SHOW not sent: TCP is not connected");
                }
            } else {
                print_line("[WARN] ", "Usage: THRESHOLD SET|OFF|SHOW ...");
            }
        } else if (strcasecmp(command, "CONFIG") == 0 ||
                   strcasecmp(command, "RUN") == 0) {
            pr_controller_run_config_t next = s_config;
            bool changed = false;
            if (!parse_config_tokens(save ? save : (char *)"", &next, &changed)) {
                print_line("[WARN] ", "Bad CONFIG; use key=value tokens (see HELP)");
                continue;
            }
            if (changed) s_config = next;
            if (strcasecmp(command, "CONFIG") == 0) {
                if (send_packet(PRC_EVT_CONTROLLER_CONFIG, &s_config, sizeof(s_config))) {
                    print_line("[INFO] ", "CONFIG sent");
                    print_config();
                } else {
                    print_line("[WARN] ", "CONFIG not sent: TCP is not connected");
                }
            } else {
                if (changed && !send_packet(PRC_EVT_CONTROLLER_CONFIG,
                                             &s_config, sizeof(s_config))) {
                    print_line("[WARN] ", "RUN config not sent: TCP is not connected");
                    continue;
                }
                if (send_simple(PRC_EVT_CONTROLLER_START)) {
                    print_line("[INFO] ", "START sent");
                } else {
                    print_line("[WARN] ", "START not sent: TCP is not connected");
                }
            }
        } else if (strcasecmp(command, "START") == 0) {
            if (send_simple(PRC_EVT_CONTROLLER_START)) {
                print_line("[INFO] ", "START sent");
            } else {
                print_line("[WARN] ", "START not sent: TCP is not connected");
            }
        } else if (strcasecmp(command, "STOP") == 0) {
            if (send_simple(PRC_EVT_CONTROLLER_STOP)) {
                print_line("[INFO] ", "STOP sent");
            } else {
                print_line("[WARN] ", "STOP not sent: TCP is not connected");
            }
        } else if (strcasecmp(command, "UNLOCK") == 0 ||
                   strcasecmp(command, "UNLK") == 0) {
            if (send_simple(PRC_EVT_CONTROLLER_UNLOCK)) {
                print_line("[INFO] ", "UNLOCK sent");
            } else {
                print_line("[WARN] ", "UNLOCK not sent: TCP is not connected");
            }
        } else if (strcasecmp(command, "OTA") == 0) {
            if (send_simple(PRC_EVT_CONTROLLER_OTA)) {
                print_line("[INFO] ", "OTA sent");
            } else {
                print_line("[WARN] ", "OTA not sent: TCP is not connected");
            }
        } else {
            print_line("[WARN] ", "Unknown command; type HELP");
        }
    }
}

extern "C" void controller_start(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);

    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);
    s_socket_mutex = xSemaphoreCreateMutex();
    assert(s_socket_mutex != nullptr);

    wifi_init();
    xTaskCreate(tcp_task, "controller_tcp", 6144, nullptr, 6, nullptr);
    xTaskCreate(cli_task, "controller_cli", 4096, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "started; Master=%s:%d", PR_MASTER_IP, PR_TCP_PORT);
}
