#pragma once
#ifndef TCP_PROTOCOL_H_
#define TCP_PROTOCOL_H_

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_crc.h"
#include "firmware.h"
#include "LED.h"
#include "PowerRune_Events.h"
#include "pr_protocol_legacy.h"

#include "lwip/sockets.h"

#define POWERRUNE_TCP_PORT 8080
#define POWERRUNE_DATA_HEADER PR_TCP_HEADER
#define TCP_DATA_LEN PR_TCP_DATA_LEN
#define TCP_ADDR_LEN PR_TCP_ADDR_LEN

#if CONFIG_POWERRUNE_TYPE == 1
#define TCP_QUEUE_SIZE 6
#else
#define TCP_QUEUE_SIZE 1
#endif

typedef pr_wire_packet_t tcp_DATA_pack_t;

typedef struct
{
    uint16_t header;
    uint16_t pack_id = 0;
    uint8_t dest_mac[TCP_ADDR_LEN];
} ACK_OK_pack_t;

typedef struct
{
    uint16_t header;
    uint16_t pack_id = 0;
    uint8_t dest_mac[TCP_ADDR_LEN];
} ACK_FAIL_pack_t;

typedef struct {
    EventGroupHandle_t event_group;
    PowerRune_Armour_config_info_t *armour_config;
    uint8_t mac_addr_new[5][TCP_ADDR_LEN];
    uint16_t tx_id_new[5];
    uint16_t rx_id_new[5];
} reset_armour_id_t;
class TcpProtocol
{
public:
    static EventGroupHandle_t send_state;
    static QueueHandle_t tcp_rx_queue;
    static SemaphoreHandle_t tx_semaphore;

    TcpProtocol(esp_event_handler_t beacon_timeout_handler = NULL);

    static esp_err_t send_data(uint8_t *dest_mac, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len, uint8_t wait_ack = 1, uint8_t mutex = 1, uint8_t id_plus = 1);
    static bool controller_send_log(uint8_t level, const char *format, ...);
    static bool controller_connected();
    static void tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void parse_data_task(void *pvParameter);

#if CONFIG_POWERRUNE_TYPE == 1
    static int server_socket;
    static int client_sockets[6];
    static int controller_socket;
    static uint8_t mac_addr[6][TCP_ADDR_LEN]; // Dummy for mock
    static void tcp_server_task(void *pvParameter);
    static uint8_t mac_to_address(uint8_t *mac);
    static esp_err_t reset_armour_id();
    static void reset_id_PRA_HIT_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void update_mac_to_address_map() {}
#else
    static int client_socket;
    static TaskHandle_t beacon_task_handle;
    static esp_event_handler_t beacon_timeout_handler;
    static void beacon_task(void *pvParameter);
    static void tcp_client_task(void *pvParameter);
    static uint8_t mac_addr[TCP_ADDR_LEN]; // Dummy
#endif
};

#endif
