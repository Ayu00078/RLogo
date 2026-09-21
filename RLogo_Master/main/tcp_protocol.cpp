#include "tcp_protocol.h"
#include "esp_log.h"
#include <cstdarg>
#include <cstdio>
#include <string.h>

static const char *TAG = "TCP_PROTO";

EventGroupHandle_t TcpProtocol::send_state = NULL;
QueueHandle_t TcpProtocol::tcp_rx_queue = NULL;
SemaphoreHandle_t TcpProtocol::tx_semaphore = NULL;

#if CONFIG_POWERRUNE_TYPE == 1
int TcpProtocol::server_socket = -1;
int TcpProtocol::client_sockets[6] = {-1, -1, -1, -1, -1, -1};
int TcpProtocol::controller_socket = -1;
uint8_t TcpProtocol::mac_addr[6][TCP_ADDR_LEN] = {0};

uint8_t TcpProtocol::mac_to_address(uint8_t *mac) {
    return 0;
}

esp_err_t TcpProtocol::reset_armour_id() {
    xEventGroupSetBits(send_state, 1);
    return ESP_OK;
}

void TcpProtocol::reset_id_PRA_HIT_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {}
#else
int TcpProtocol::client_socket = -1;
TaskHandle_t TcpProtocol::beacon_task_handle = NULL;
esp_event_handler_t TcpProtocol::beacon_timeout_handler = NULL;
uint8_t TcpProtocol::mac_addr[TCP_ADDR_LEN] = {0};
#endif

TcpProtocol::TcpProtocol(esp_event_handler_t timeout_handler) {
    if (send_state == NULL) {
        send_state = xEventGroupCreate();
        tcp_rx_queue = xQueueCreate(TCP_QUEUE_SIZE, sizeof(tcp_DATA_pack_t));
        tx_semaphore = xSemaphoreCreateMutex();

#if CONFIG_POWERRUNE_TYPE == 1
        xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
#else
        TcpProtocol::beacon_timeout_handler = timeout_handler;
        xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 6, NULL);
        xTaskCreate(beacon_task, "tcp_beacon", 2048, NULL, 4, &beacon_task_handle);
#endif
        xTaskCreate(parse_data_task, "tcp_parse", 4096, NULL, 4, NULL);
    }
}

static inline void serialize_and_send(int sock, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len) {
    if (sock < 0) {
        ESP_LOGW(TAG, "TX SKIP: sock<0 for base=%s id=%d", event_base, (int)event_id);
        return;
    }
    uint16_t actual_len = data_len;
    if (data && data_len > 1U) {
        const uint8_t declared_len = ((const uint8_t *)data)[1];
        if (declared_len > 0U && declared_len <= TCP_DATA_LEN) actual_len = declared_len;
    }
    pr_packet_t packet;
    pr_packet_init(&packet, event_base, (uint8_t)event_id,
                   data ? ((const uint8_t *)data)[0] : 0xFFU);
    if (!pr_packet_set_payload(&packet, data, actual_len)) return;
    uint8_t encoded[PR_TCP_FRAME_SIZE];
    size_t encoded_size = 0;
    if (!pr_packet_encode(&packet, encoded, sizeof(encoded), &encoded_size)) return;
    size_t sent = 0;
    while (sent < encoded_size) {
        const int ret = send(sock, &encoded[sent], encoded_size - sent, 0);
        if (ret <= 0) {
            ESP_LOGE(TAG, "TX FAIL: sock=%d base=%s id=%d errno=%d", sock, event_base, (int)event_id, errno);
            return;
        }
        sent += (size_t)ret;
    }
}

static bool serialize_payload_and_send(int sock, const char *event_base, int32_t event_id,
                                       const void *payload, uint16_t payload_len) {
    if (sock < 0 || payload == nullptr || payload_len > TCP_DATA_LEN) {
        return false;
    }

    pr_packet_t packet;
    const uint8_t address = ((const uint8_t *)payload)[0];
    pr_packet_init(&packet, event_base, (uint8_t)event_id, address);
    if (!pr_packet_set_payload(&packet, payload, payload_len)) {
        return false;
    }

    uint8_t encoded[PR_TCP_FRAME_SIZE];
    size_t encoded_size = 0;
    if (!pr_packet_encode(&packet, encoded, sizeof(encoded), &encoded_size)) {
        return false;
    }

    size_t sent = 0;
    while (sent < encoded_size) {
        const int ret = send(sock, &encoded[sent], encoded_size - sent, 0);
        if (ret <= 0) {
            ESP_LOGE(TAG, "TX FAIL: sock=%d base=%s id=%d errno=%d",
                     sock, event_base, (int)event_id, errno);
            return false;
        }
        sent += (size_t)ret;
    }
    return true;
}

void TcpProtocol::tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_data == NULL) return;
#if CONFIG_POWERRUNE_TYPE == 1
    uint8_t address = ((uint8_t *)event_data)[0];

    xSemaphoreTake(tx_semaphore, portMAX_DELAY);
    if (address == 0xFF) {
        ESP_LOGD(TAG, "TX BROADCAST: base=%s id=%d", event_base, (int)event_id);
        for (int i = 0; i < 6; i++) {
            if (client_sockets[i] >= 0) {
                serialize_and_send(client_sockets[i], event_base, event_id, event_data, 200);
            }
        }
    } else if (address < 6) {
        ESP_LOGD(TAG, "TX UNICAST: addr=%d sock=%d base=%s id=%d", address, client_sockets[address], event_base, (int)event_id);
        if (client_sockets[address] < 0) {
            ESP_LOGW(TAG, "TX DROPPED: addr=%d not connected! sockets=[%d,%d,%d,%d,%d,%d]",
                     address, client_sockets[0], client_sockets[1], client_sockets[2],
                     client_sockets[3], client_sockets[4], client_sockets[5]);
        }
        serialize_and_send(client_sockets[address], event_base, event_id, event_data, 200);
    } else {
        ESP_LOGW(TAG, "TX INVALID addr=%d base=%s id=%d", address, event_base, (int)event_id);
    }
    xSemaphoreGive(tx_semaphore);

#else
    xSemaphoreTake(tx_semaphore, portMAX_DELAY);
    serialize_and_send(client_socket, event_base, event_id, event_data, 200);
    xSemaphoreGive(tx_semaphore);
#endif
}

esp_err_t TcpProtocol::send_data(uint8_t *dest_mac, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len, uint8_t wait_ack, uint8_t mutex, uint8_t id_plus) {
    return ESP_OK;
}

bool TcpProtocol::controller_connected() {
#if CONFIG_POWERRUNE_TYPE == 1
    return controller_socket >= 0;
#else
    return false;
#endif
}

bool TcpProtocol::controller_send_log(uint8_t level, const char *format, ...) {
#if CONFIG_POWERRUNE_TYPE != 1
    (void)level;
    (void)format;
    return false;
#else
    if (format == nullptr || level < PR_CONTROLLER_LOG_INFO ||
        level > PR_CONTROLLER_LOG_DEBUG || tx_semaphore == nullptr) {
        return false;
    }

    uint8_t payload[PR_TCP_DATA_LEN] = {};
    char text[PR_CONTROLLER_LOG_MAX_TEXT + 1U] = {};
    payload[0] = PR_CONTROLLER_ADDRESS;
    payload[1] = level;

    va_list args;
    va_start(args, format);
    const int written = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (written < 0) {
        return false;
    }

    const uint16_t text_len = (uint16_t)((written > (int)PR_CONTROLLER_LOG_MAX_TEXT)
                                             ? PR_CONTROLLER_LOG_MAX_TEXT
                                             : written);
    memcpy(&payload[2], text, text_len);
    const uint16_t payload_len = (uint16_t)(2U + text_len);

    if (xSemaphoreTake(tx_semaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    const int sock = controller_socket;
    const bool ok = serialize_payload_and_send(sock, PR_BASE_COMMON,
                                               PRC_EVT_CONTROLLER_LOG,
                                               payload, payload_len);
    if (!ok && controller_socket == sock) {
        controller_socket = -1;
    }
    xSemaphoreGive(tx_semaphore);
    return ok;
#endif
}

#if CONFIG_POWERRUNE_TYPE == 1
void TcpProtocol::tcp_server_task(void *pvParameter) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(POWERRUNE_TCP_PORT);

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 10);

    ESP_LOGI(TAG, "Server listening on port %d", POWERRUNE_TCP_PORT);
    server_socket = listen_sock;

    while (1) {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock >= 0) {
            ESP_LOGI(TAG, "New client connected! fd=%d", sock);
            int no_delay = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
            xTaskCreate([](void* arg){
                int fd = (int)arg;
                tcp_DATA_pack_t pack;
                while(recv(fd, &pack, sizeof(pack), MSG_WAITALL) > 0) {
                    if (pack.header == POWERRUNE_DATA_HEADER) {
                        if (strcmp(pack.event_base, "PRA") == 0 || strcmp(pack.event_base, "PRM") == 0 || strcmp(pack.event_base, "PRC") == 0) {
                            if (pack.event_data_len > 0) {
                                uint8_t address = pack.event_data[0];
                                const bool is_controller =
                                    strcmp(pack.event_base, PR_BASE_COMMON) == 0 &&
                                    address == PR_CONTROLLER_ADDRESS;
                                if (is_controller) {
                                    TcpProtocol::controller_socket = fd;
                                    ESP_LOGI(TAG, "Controller connected: fd=%d", fd);
                                    if (pack.event_id == PRC_EVT_CONTROLLER_HELLO &&
                                        pack.event_data_len >= sizeof(pr_controller_hello_t)) {
                                        ESP_LOGI(TAG, "Controller protocol version=%u",
                                                 pack.event_data[1]);
                                    }
                                    TcpProtocol::controller_send_log(
                                        PR_CONTROLLER_LOG_INFO,
                                        "TCP connected to Master; controller address=0x%02X",
                                        PR_CONTROLLER_ADDRESS);
                                } else if (address < 6) {
                                    if (client_sockets[address] >= 0 && client_sockets[address] != fd) {
                                        ESP_LOGE(TAG, "REJECT duplicate fixed address=%d: fd=%d already owns it, new fd=%d",
                                                 address, client_sockets[address], fd);
                                        break;
                                    }
                                    client_sockets[address] = fd;
                                    ESP_LOGD(TAG, "REGISTER fixed address=%d -> fd=%d", address, fd);
                                } else if (strcmp(pack.event_base, "PRA") == 0 ||
                                           strcmp(pack.event_base, "PRM") == 0) {
                                    ESP_LOGE(TAG, "REJECT unconfigured address=%d from fd=%d", address, fd);
                                    break;
                                }
                            }
                            ESP_LOGD(TAG, "RX: base=%s id=%d len=%d from fd=%d", pack.event_base, (int)pack.event_id, pack.event_data_len, fd);
                            xQueueSend(tcp_rx_queue, &pack, 0);
                        }
                    }
                }
                ESP_LOGW(TAG, "Client disconnected! fd=%d", fd);
                close(fd);
                if (TcpProtocol::controller_socket == fd) {
                    TcpProtocol::controller_socket = -1;
                    ESP_LOGW(TAG, "Controller disconnected");
                }
                for(int i=0; i<6; i++) {
                    if (client_sockets[i] == fd) {
                        ESP_LOGW(TAG, "UNREGISTER: addr=%d (fd=%d)", i, fd);
                        client_sockets[i] = -1;
                    }
                }
                vTaskDelete(NULL);
            }, "client_rx", 4096, (void*)sock, 5, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#else
void TcpProtocol::tcp_client_task(void *pvParameter) {
    while(1) {
        if (client_socket < 0) {
            int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr("192.168.4.1");
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(POWERRUNE_TCP_PORT);

            ESP_LOGI(TAG, "Connecting to RLogo...");
            int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err == 0) {
                ESP_LOGI(TAG, "Successfully connected to Server!");
                client_socket = sock;

                tcp_DATA_pack_t pack;
                while(recv(sock, &pack, sizeof(pack), MSG_WAITALL) > 0) {
                    if (pack.header == POWERRUNE_DATA_HEADER) {
                        xQueueSend(tcp_rx_queue, &pack, 0);
                    }
                }
                ESP_LOGE(TAG, "Socket closed or error");
                close(sock);
                client_socket = -1;
            } else {
                ESP_LOGW(TAG, "Connect fail, retrying...");
                close(sock);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TcpProtocol::beacon_task(void *pvParameter) {
    while(1) {
        if (client_socket >= 0) {
            esp_event_base_t base = (CONFIG_POWERRUNE_TYPE == 0) ? PRA : PRM;
            int32_t id = (CONFIG_POWERRUNE_TYPE == 0) ? PRA_PING_EVENT : PRM_PING_EVENT;

            extern Config* config;
            if (config && config->get_config_info_pt()) {
                #if CONFIG_POWERRUNE_TYPE == 0
                PRA_PING_EVENT_DATA pdata = {};
                pdata.address = config->get_config_info_pt()->armour_id;
                if(pdata.address > 4) pdata.address = 0; // fallback if unset
                pdata.data_len = sizeof(pdata);
                pdata.config_info = *config->get_config_info_pt();
                serialize_and_send(client_socket, base, id, &pdata, sizeof(pdata));
                #elif CONFIG_POWERRUNE_TYPE == 2
                PRM_PING_EVENT_DATA mdata = {};
                mdata.address = 5;
                mdata.data_len = sizeof(mdata);
                mdata.config_info = *config->get_config_info_pt();
                serialize_and_send(client_socket, base, id, &mdata, sizeof(mdata));
                #endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

void TcpProtocol::parse_data_task(void *pvParameter) {
    tcp_DATA_pack_t pack;
    while(1) {
        if (xQueueReceive(tcp_rx_queue, &pack, portMAX_DELAY) == pdTRUE) {
            esp_event_base_t e_base = NULL;
            if (strcmp(pack.event_base, "PRA") == 0) e_base = PRA;
            else if (strcmp(pack.event_base, "PRM") == 0) e_base = PRM;
            else if (strcmp(pack.event_base, "PRC") == 0) e_base = PRC;

            if (e_base) {
                uint8_t addr = (pack.event_data_len > 0) ? pack.event_data[0] : 0xFF;
                ESP_LOGI(TAG, "PARSE: base=%s id=%d addr=%d len=%d", pack.event_base, (int)pack.event_id, addr, pack.event_data_len);
                TcpProtocol::controller_send_log(
                    PR_CONTROLLER_LOG_DEBUG,
                    "RX base=%s id=%d addr=%u len=%u",
                    pack.event_base, (int)pack.event_id, addr, pack.event_data_len);
                if (pack.event_data_len > 0) {
                    esp_event_post_to(pr_events_loop_handle, e_base, pack.event_id, pack.event_data, pack.event_data_len, portMAX_DELAY);
                } else {
                    esp_event_post_to(pr_events_loop_handle, e_base, pack.event_id, NULL, 0, portMAX_DELAY);
                }
            } else {
                ESP_LOGW(TAG, "PARSE: unknown base='%s'", pack.event_base);
            }
        }
    }
}
