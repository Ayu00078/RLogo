#include "tcp_protocol.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TCP_PROTO";

/* Formal address is fixed at flash time: physical ID 1..5 maps to 0..4. */
static volatile uint8_t s_runtime_armour_address = (uint8_t)(CONFIG_ARMOUR_ID - 1);

EventGroupHandle_t TcpProtocol::send_state = NULL;
QueueHandle_t TcpProtocol::tcp_rx_queue = NULL;
SemaphoreHandle_t TcpProtocol::tx_semaphore = NULL;

#if CONFIG_POWERRUNE_TYPE == 1
int TcpProtocol::server_socket = -1;
int TcpProtocol::client_sockets[6] = {-1, -1, -1, -1, -1, -1};
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
        ESP_LOGW(TAG, "TX SKIP: sock<0 base=%s id=%d", event_base, (int)event_id);
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

void TcpProtocol::tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_data == NULL) return;
    // 所有事件数据结构约定：[0]=address，[1]=data_len，通过第二字节获取实际长度，避免越界读取
    uint8_t actual_len = ((uint8_t *)event_data)[1];
    if (actual_len == 0 || actual_len > TCP_DATA_LEN) {
        ESP_LOGW(TAG, "TX: invalid data_len=%d, clamping", actual_len);
        actual_len = (actual_len > TCP_DATA_LEN) ? TCP_DATA_LEN : 1;
    }
#if CONFIG_POWERRUNE_TYPE == 1
    uint8_t address = ((uint8_t *)event_data)[0];

    xSemaphoreTake(tx_semaphore, portMAX_DELAY);
    if (address == 0xFF) {
        for (int i = 0; i < 6; i++) {
            if (client_sockets[i] >= 0) {
                serialize_and_send(client_sockets[i], event_base, event_id, event_data, actual_len);
            }
        }
    } else if (address < 6) {
        serialize_and_send(client_sockets[address], event_base, event_id, event_data, actual_len);
    }
    xSemaphoreGive(tx_semaphore);

#else
    ESP_LOGD(TAG, "TX: base=%s id=%d sock=%d len=%d", event_base, (int)event_id, client_socket, actual_len);
    xSemaphoreTake(tx_semaphore, portMAX_DELAY);
    serialize_and_send(client_socket, event_base, event_id, event_data, actual_len);
    xSemaphoreGive(tx_semaphore);
#endif
}

uint8_t TcpProtocol::runtime_armour_address() {
#if CONFIG_POWERRUNE_TYPE == 0
    return s_runtime_armour_address;
#else
    return 5;
#endif
}

void TcpProtocol::set_runtime_armour_address(uint8_t address) {
#if CONFIG_POWERRUNE_TYPE == 0
    const uint8_t fixed_address = (uint8_t)(CONFIG_ARMOUR_ID - 1);
    if (address != fixed_address) {
        ESP_LOGE(TAG, "Reject runtime address %u; fixed address is %u", address, fixed_address);
        return;
    }
    s_runtime_armour_address = fixed_address;
#else
    (void)address;
#endif
}

esp_err_t TcpProtocol::send_data(uint8_t *dest_mac, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len, uint8_t wait_ack, uint8_t mutex, uint8_t id_plus) {
    return ESP_OK;
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
            // 启用 TCP Keepalive
            int keepalive = 1, keepidle = 5, keepintvl = 2, keepcnt = 3;
            setsockopt(sock, SOL_SOCKET,  SO_KEEPALIVE,  &keepalive,  sizeof(keepalive));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,   sizeof(keepidle));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl,  sizeof(keepintvl));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,    sizeof(keepcnt));
            xTaskCreate([](void* arg){
                int fd = (int)arg;
                tcp_DATA_pack_t pack;
                while(recv(fd, &pack, sizeof(pack), MSG_WAITALL) > 0) {
                    if (pack.header == POWERRUNE_DATA_HEADER) {
                        if (strcmp(pack.event_base, "PRA") == 0 || strcmp(pack.event_base, "PRM") == 0 || strcmp(pack.event_base, "PRC") == 0) {
                            if (pack.event_data_len > 0) {
                                uint8_t address = pack.event_data[0];
                                if (address < 6) {
                                    client_sockets[address] = fd;
                                }
                            }
                            xQueueSend(tcp_rx_queue, &pack, 0);
                        }
                    }
                }
                ESP_LOGI(TAG, "Client disconnected! fd=%d", fd);
                close(fd);
                for(int i=0; i<6; i++) {
                    if (client_sockets[i] == fd) client_sockets[i] = -1;
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

            // 启用 TCP Keepalive，防止网络静默断开时 recv() 永久阻塞
            int keepalive = 1;
            int keepidle  = 5;   // 5 秒无数据后开始探测
            int keepintvl = 2;   // 每次探测间隔 2 秒
            int keepcnt   = 3;   // 最多探测 3 次，仍无响应则判定断连
            setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));

            ESP_LOGI(TAG, "Connecting to RLogo (192.168.4.1:%d)...", POWERRUNE_TCP_PORT);
            int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err == 0) {
                ESP_LOGI(TAG, "Connected to RLogo! sock=%d", sock);
                int no_delay = 1;
                setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
                client_socket = sock;

                tcp_DATA_pack_t pack;
                while(recv(sock, &pack, sizeof(pack), MSG_WAITALL) > 0) {
                    if (pack.header == POWERRUNE_DATA_HEADER) {
                        ESP_LOGI(TAG, "RX from RLogo: base=%s id=%d len=%d",
                                 pack.event_base, (int)pack.event_id, pack.event_data_len);
                        xQueueSend(tcp_rx_queue, &pack, 0);
                    }
                }
                ESP_LOGE(TAG, "Connection to RLogo lost! errno=%d", errno);
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
                pdata.address = s_runtime_armour_address;
                pdata.data_len = sizeof(pdata);
                pdata.config_info = *config->get_config_info_pt();
                pdata.config_info.armour_id = (s_runtime_armour_address < 5) ? (s_runtime_armour_address + 1) : 0;
                ESP_LOGD(TAG, "BEACON: runtime_addr=%d", pdata.address);
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
            uint8_t addr = (pack.event_data_len > 0) ? pack.event_data[0] : 0xFF;
            ESP_LOGI(TAG, "PARSE: base=%s id=%d addr=%d len=%d", pack.event_base, (int)pack.event_id, addr, pack.event_data_len);

            esp_event_base_t e_base = NULL;
            if (strcmp(pack.event_base, "PRA") == 0) e_base = PRA;
            else if (strcmp(pack.event_base, "PRM") == 0) e_base = PRM;
            else if (strcmp(pack.event_base, "PRC") == 0) e_base = PRC;

            if (e_base) {
#if CONFIG_POWERRUNE_TYPE != 1
                // Armour/Motor 侧：过滤掉只应该由本地产生、上报给 RLogo 的事件。
                // 若这些事件从网络接收后被重新 post，会再次触发 tx_event_handler，
                // 将报文反射回 RLogo 造成消息循环。
                if (e_base == PRA && (pack.event_id == PRA_HIT_EVENT ||
                                      pack.event_id == PRA_SENSOR_PARAM_ACK_EVENT ||
                                      pack.event_id == PRA_HIT_THRESHOLD_ACK_EVENT ||
                                      pack.event_id == PRA_PING_EVENT)) {
                    ESP_LOGW(TAG, "PARSE: discard upward event id=%d received from network", pack.event_id);
                    continue;
                }
#endif
                if (pack.event_data_len > 0) {
                    esp_event_post_to(pr_events_loop_handle, e_base, pack.event_id, pack.event_data, pack.event_data_len, portMAX_DELAY);
                } else {
                    esp_event_post_to(pr_events_loop_handle, e_base, pack.event_id, NULL, 0, portMAX_DELAY);
                }
            }
        }
    }
}
