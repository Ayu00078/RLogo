#include "pr_protocol_legacy.h"

#include <string.h>

static bool valid_base(const char *base)
{
    return base != NULL &&
           (strncmp(base, PR_BASE_COMMON, 3) == 0 ||
            strncmp(base, PR_BASE_ARMOUR, 3) == 0 ||
            strncmp(base, PR_BASE_MOTOR, 3) == 0);
}

void pr_packet_init(pr_packet_t *packet, const char *event_base, uint8_t event_id,
                    uint8_t address)
{
    memset(packet, 0, sizeof(*packet));
    packet->pack_id = 0;
    memcpy(packet->event_base, event_base, 3);
    packet->event_base[3] = '\0';
    packet->event_id = event_id;
    packet->event_data[0] = address;
    packet->event_data_len = 1;
}

bool pr_packet_set_payload(pr_packet_t *packet, const void *payload, size_t length)
{
    if (packet == NULL || length > PR_TCP_DATA_LEN ||
        (length > 0U && payload == NULL)) {
        return false;
    }
    packet->event_data_len = (uint16_t)length;
    memset(packet->event_data, 0, sizeof(packet->event_data));
    if (length > 0U) {
        memcpy(packet->event_data, payload, length);
    }
    return true;
}

bool pr_packet_validate(const pr_packet_t *packet)
{
    return packet != NULL && valid_base(packet->event_base) &&
           packet->event_data_len <= PR_TCP_DATA_LEN;
}

bool pr_packet_encode(const pr_packet_t *packet, uint8_t *buffer, size_t capacity,
                      size_t *encoded_size)
{
    if (!pr_packet_validate(packet) || buffer == NULL || capacity < PR_TCP_FRAME_SIZE) {
        return false;
    }
    pr_wire_packet_t wire = {0};
    wire.header = PR_TCP_HEADER;
    wire.pack_id = packet->pack_id;
    wire.crc16 = 0;
    memcpy(wire.event_base, packet->event_base, sizeof(wire.event_base));
    wire.event_data_len = packet->event_data_len;
    wire.event_id = packet->event_id;
    memcpy(wire.event_data, packet->event_data, sizeof(wire.event_data));
    memcpy(wire.dest_mac, packet->dest_mac, sizeof(wire.dest_mac));
    memcpy(buffer, &wire, sizeof(wire));
    if (encoded_size != NULL) {
        *encoded_size = sizeof(wire);
    }
    return true;
}

bool pr_packet_decode(const uint8_t *buffer, size_t length, pr_packet_t *packet)
{
    if (buffer == NULL || packet == NULL || length != PR_TCP_FRAME_SIZE) {
        return false;
    }
    pr_wire_packet_t wire;
    memcpy(&wire, buffer, sizeof(wire));
    if (wire.header != PR_TCP_HEADER || !valid_base(wire.event_base) ||
        wire.event_data_len > PR_TCP_DATA_LEN) {
        return false;
    }
    memset(packet, 0, sizeof(*packet));
    packet->pack_id = wire.pack_id;
    packet->event_data_len = wire.event_data_len;
    packet->event_id = wire.event_id;
    memcpy(packet->event_base, wire.event_base, sizeof(packet->event_base));
    packet->event_base[3] = '\0';
    memcpy(packet->event_data, wire.event_data, sizeof(packet->event_data));
    memcpy(packet->dest_mac, wire.dest_mac, sizeof(packet->dest_mac));
    return true;
}

bool pr_packet_is(const pr_packet_t *packet, const char *event_base, uint8_t event_id)
{
    return packet != NULL && packet->event_id == event_id &&
           event_base != NULL && strncmp(packet->event_base, event_base, 3) == 0;
}

const char *pr_event_name(const char *event_base, uint8_t event_id)
{
    if (strncmp(event_base, PR_BASE_ARMOUR, 3) == 0) {
        switch (event_id) {
        case PR_EVT_STOP: return "PRA_STOP";
        case PR_EVT_START: return "PRA_START";
        case PR_EVT_HIT: return "PRA_HIT";
        case PR_EVT_COMPLETE: return "PRA_COMPLETE";
        case PR_EVT_PING: return "PRA_PING";
        case PR_EVT_NOTARGET: return "PRA_NOTARGET";
        case PR_EVT_SET_SENSOR_PARAM: return "PRA_SET_SENSOR_PARAM";
        case PR_EVT_SENSOR_PARAM_ACK: return "PRA_SENSOR_PARAM_ACK";
        case PR_EVT_CALIBRATE: return "PRA_CALIBRATE";
        case PR_EVT_ASSIGN_ID: return "PRA_ASSIGN_ID";
        case PR_EVT_HIT_DECISION: return "PRA_HIT_DECISION";
        case PR_EVT_HIT_DECISION_ACK: return "PRA_HIT_DECISION_ACK";
        case PR_EVT_HIT_TIMEOUT: return "PRA_HIT_TIMEOUT";
        case PR_EVT_HIT_THRESHOLD_CONFIG: return "PRA_HIT_THRESHOLD_CONFIG";
        case PR_EVT_HIT_THRESHOLD_ACK: return "PRA_HIT_THRESHOLD_ACK";
        default: break;
        }
    } else if (strncmp(event_base, PR_BASE_MOTOR, 3) == 0) {
        switch (event_id) {
        case PRM_EVT_UNLOCK: return "PRM_UNLOCK";
        case PRM_EVT_UNLOCK_DONE: return "PRM_UNLOCK_DONE";
        case PRM_EVT_START: return "PRM_START";
        case PRM_EVT_START_DONE: return "PRM_START_DONE";
        case PRM_EVT_SPEED_STABLE: return "PRM_SPEED_STABLE";
        case PRM_EVT_STOP: return "PRM_STOP";
        case PRM_EVT_DISCONNECT: return "PRM_DISCONNECT";
        case PRM_EVT_PING: return "PRM_PING";
        default: break;
        }
    } else if (strncmp(event_base, PR_BASE_COMMON, 3) == 0) {
        switch (event_id) {
        case PRC_EVT_OTA_BEGIN: return "PRC_OTA_BEGIN";
        case PRC_EVT_OTA_COMPLETE: return "PRC_OTA_COMPLETE";
        case PRC_EVT_CONFIG: return "PRC_CONFIG";
        case PRC_EVT_CONFIG_COMPLETE: return "PRC_CONFIG_COMPLETE";
        case PRC_EVT_RESPONSE: return "PRC_RESPONSE";
        case PRC_EVT_BEACON_TIMEOUT: return "PRC_BEACON_TIMEOUT";
        case PRC_EVT_CONTROLLER_HELLO: return "PRC_CONTROLLER_HELLO";
        case PRC_EVT_CONTROLLER_PING: return "PRC_CONTROLLER_PING";
        case PRC_EVT_CONTROLLER_CONFIG: return "PRC_CONTROLLER_CONFIG";
        case PRC_EVT_CONTROLLER_START: return "PRC_CONTROLLER_START";
        case PRC_EVT_CONTROLLER_STOP: return "PRC_CONTROLLER_STOP";
        case PRC_EVT_CONTROLLER_UNLOCK: return "PRC_CONTROLLER_UNLOCK";
        case PRC_EVT_CONTROLLER_OTA: return "PRC_CONTROLLER_OTA";
        case PRC_EVT_CONTROLLER_STATUS_REQUEST: return "PRC_CONTROLLER_STATUS_REQUEST";
        case PRC_EVT_CONTROLLER_DEBUG: return "PRC_CONTROLLER_DEBUG";
        case PRC_EVT_CONTROLLER_LOG: return "PRC_CONTROLLER_LOG";
        case PRC_EVT_CONTROLLER_THRESHOLD_SET: return "PRC_CONTROLLER_THRESHOLD_SET";
        case PRC_EVT_CONTROLLER_THRESHOLD_OFF: return "PRC_CONTROLLER_THRESHOLD_OFF";
        case PRC_EVT_CONTROLLER_THRESHOLD_SHOW: return "PRC_CONTROLLER_THRESHOLD_SHOW";
        default: break;
        }
    }
    return "UNKNOWN";
}
