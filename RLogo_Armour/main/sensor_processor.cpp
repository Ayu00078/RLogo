/**
 * Armour sensor detector.
 *
 * The UART receiver has already verified the 18-byte sensor frame.  The
 * detector below follows the standalone Learn_test firmware: a 20-frame
 * moving baseline, a fixed +20 trigger, a 50-frame peak window, and the
 * documented ADC-to-ring mapping.  Every local candidate is sent to Master;
 * Master later returns the authoritative game decision.
 */
#include "sensor_processor.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "PowerRune_Events.h"
#include "pr_types.h"
#include "tcp_protocol.h"

#include <inttypes.h>
#include <string.h>

static const char *TAG = "sensor_proc";

static constexpr size_t kChannelCount = PR_SENSOR_ADC_COUNT;
static constexpr size_t kBaselineSamples = PR_SENSOR_BASELINE_SAMPLES;
static constexpr int32_t kHitThreshold = PR_SENSOR_HIT_THRESHOLD;
static constexpr size_t kHitWindowSamples = PR_SENSOR_HIT_WINDOW_SAMPLES;

static const uint8_t kAdcToRing[kChannelCount] = {
    6, 7, 8, 9, 10, 1, 2, 3, 4, 5,
};

enum detector_state_t {
    DETECTOR_COLLECTING_BASELINE,
    DETECTOR_MONITORING,
    DETECTOR_HIT_WINDOW,
};

struct detector_t {
    detector_state_t state;
    uint8_t history[kChannelCount][kBaselineSamples];
    uint32_t sums[kChannelCount];
    size_t baseline_samples;
    size_t history_index;
    int32_t max_excess_scaled[kChannelCount];
    size_t hit_samples;
};

static detector_t s_detector = {};
static bool s_inited = false;
static bool s_raw_log_enabled = false;
static uint8_t s_mode = PRA_RUNE_SMALL_MODE;
static uint32_t s_frame_count = 0;
static uint32_t s_hits_count = 0;
static uint32_t s_session_id = 0;
static uint32_t s_next_sequence = 0;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

struct pending_hit_t {
    bool active = false;
    TickType_t sent_tick = 0;
    PRA_HIT_EVENT_DATA candidate = {};
};

static constexpr size_t kPendingHitCount = 8;
static pending_hit_t s_pending_hits[kPendingHitCount] = {};
static portMUX_TYPE s_pending_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_ack_watchdog_task = nullptr;

static void detector_reset(void)
{
    memset(&s_detector, 0, sizeof(s_detector));
    s_detector.state = DETECTOR_COLLECTING_BASELINE;
}

static bool pending_add(const PRA_HIT_EVENT_DATA *candidate)
{
    bool added = false;
    portENTER_CRITICAL(&s_pending_mux);
    for (auto &pending : s_pending_hits) {
        if (!pending.active) {
            pending.active = true;
            pending.sent_tick = xTaskGetTickCount();
            pending.candidate = *candidate;
            added = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pending_mux);
    return added;
}

static void send_hit_timeout(const PRA_HIT_EVENT_DATA &candidate, uint32_t elapsed_ms)
{
    PRA_HIT_TIMEOUT_EVENT_DATA timeout = {};
    timeout.address = candidate.address;
    timeout.data_len = sizeof(timeout);
    timeout.session_id = candidate.session_id;
    timeout.sequence = candidate.sequence;
    timeout.elapsed_ms = elapsed_ms;

    ESP_LOGW(TAG, "HIT_ACK_TIMEOUT: seq=%" PRIu32 " elapsed=%" PRIu32 "ms",
             candidate.sequence, elapsed_ms);
    if (pr_events_loop_handle) {
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_HIT_TIMEOUT_EVENT,
                          &timeout, sizeof(timeout), 0);
    }
}

static void hit_ack_watchdog_task(void *)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10));
        const TickType_t now = xTaskGetTickCount();
        pending_hit_t expired[kPendingHitCount] = {};
        size_t expired_count = 0;

        portENTER_CRITICAL(&s_pending_mux);
        for (auto &pending : s_pending_hits) {
            if (!pending.active) continue;
            const TickType_t elapsed = now - pending.sent_tick;
            if (elapsed >= pdMS_TO_TICKS(PR_HIT_ACK_TIMEOUT_MS)) {
                if (expired_count < kPendingHitCount) {
                    expired[expired_count++] = pending;
                }
                pending.active = false;
            }
        }
        portEXIT_CRITICAL(&s_pending_mux);

        for (size_t i = 0; i < expired_count; ++i) {
            send_hit_timeout(expired[i].candidate,
                             (uint32_t)pdTICKS_TO_MS(now - expired[i].sent_tick));
        }
    }
}

static void send_hit_event(uint8_t ring, uint16_t peak_x100)
{
    if (!pr_events_loop_handle) {
        return;
    }

    const uint8_t address = TcpProtocol::runtime_armour_address();
    if (address >= 5) {
        ESP_LOGW(TAG, "drop hit before fixed address is ready (address=%u)", address);
        return;
    }

    PRA_HIT_EVENT_DATA event = {};
    event.address = address;
    event.data_len = sizeof(event);
    event.session_id = s_session_id;
    event.sequence = s_next_sequence++;
    event.ring = ring;
    event.peak_x100 = peak_x100;
    event.detected_ms = esp_log_timestamp();

    if (!pending_add(&event)) {
        ESP_LOGE(TAG, "pending hit table full; candidate seq=%" PRIu32 " still sent",
                 event.sequence);
    }

    const esp_err_t err = esp_event_post_to(pr_events_loop_handle, PRA,
                                            PRA_HIT_EVENT, &event,
                                            sizeof(event), portMAX_DELAY);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_stats_mux);
        ++s_hits_count;
        portEXIT_CRITICAL(&s_stats_mux);
        ESP_LOGI(TAG, "HIT CANDIDATE: ADC->ring=%u address=%u peak=%u.%02u seq=%" PRIu32,
                 ring, address, peak_x100 / 100U, peak_x100 % 100U,
                 event.sequence);
    } else {
        ESP_LOGE(TAG, "failed to post HIT: %s", esp_err_to_name(err));
    }
}

static void finish_hit_window(void)
{
    size_t best_adc = 0;
    bool peak_tied = false;

    for (size_t adc = 1; adc < kChannelCount; ++adc) {
        if (s_detector.max_excess_scaled[adc] >
            s_detector.max_excess_scaled[best_adc]) {
            best_adc = adc;
            peak_tied = false;
        } else if (s_detector.max_excess_scaled[adc] ==
                   s_detector.max_excess_scaled[best_adc]) {
            peak_tied = true;
        }
    }

    const int32_t peak_scaled = s_detector.max_excess_scaled[best_adc];
    const uint32_t peak_x100_u32 = peak_scaled > 0
        ? ((uint32_t)peak_scaled * 100U + (kBaselineSamples / 2U)) / kBaselineSamples
        : 0U;
    const uint16_t peak_x100 = (uint16_t)((peak_x100_u32 > UINT16_MAX)
        ? UINT16_MAX : peak_x100_u32);
    const uint8_t ring = peak_tied ? 0 : kAdcToRing[best_adc];
    if (peak_tied) {
        ESP_LOGW(TAG, "ambiguous hit: equal peak increments, ring=0");
    } else {
        ESP_LOGI(TAG, "hit window complete: ADC%u -> ring %u peak=%ld.%02ld",
                 (unsigned)best_adc, (unsigned)ring,
                 (long)(s_detector.max_excess_scaled[best_adc] /
                        (int32_t)kBaselineSamples),
                 (long)((s_detector.max_excess_scaled[best_adc] %
                         (int32_t)kBaselineSamples) * 5));
    }

    send_hit_event(ring, peak_x100);
    detector_reset();
}

static void detector_process(const uint8_t *adc)
{
    if (s_detector.state == DETECTOR_COLLECTING_BASELINE) {
        for (size_t channel = 0; channel < kChannelCount; ++channel) {
            s_detector.history[channel][s_detector.baseline_samples] = adc[channel];
            s_detector.sums[channel] += adc[channel];
        }

        ++s_detector.baseline_samples;
        if (s_detector.baseline_samples == kBaselineSamples) {
            s_detector.history_index = 0;
            s_detector.state = DETECTOR_MONITORING;
            ESP_LOGI(TAG, "baseline ready after %u valid frames",
                     (unsigned)kBaselineSamples);
        }
        return;
    }

    int32_t excess_scaled[kChannelCount];
    bool hit_started = false;
    const int32_t threshold_scaled = kHitThreshold * (int32_t)kBaselineSamples;
    for (size_t channel = 0; channel < kChannelCount; ++channel) {
        excess_scaled[channel] =
            (int32_t)adc[channel] * (int32_t)kBaselineSamples -
            (int32_t)s_detector.sums[channel];
        if (excess_scaled[channel] > threshold_scaled) {
            hit_started = true;
        }
    }

    if (s_detector.state == DETECTOR_MONITORING) {
        if (hit_started) {
            memcpy(s_detector.max_excess_scaled, excess_scaled,
                   sizeof(s_detector.max_excess_scaled));
            s_detector.hit_samples = 1;
            s_detector.state = DETECTOR_HIT_WINDOW;
            ESP_LOGI(TAG, "hit window started");
            return;
        }

        for (size_t channel = 0; channel < kChannelCount; ++channel) {
            s_detector.sums[channel] -=
                s_detector.history[channel][s_detector.history_index];
            s_detector.history[channel][s_detector.history_index] = adc[channel];
            s_detector.sums[channel] += adc[channel];
        }
        s_detector.history_index =
            (s_detector.history_index + 1) % kBaselineSamples;
        return;
    }

    for (size_t channel = 0; channel < kChannelCount; ++channel) {
        if (excess_scaled[channel] > s_detector.max_excess_scaled[channel]) {
            s_detector.max_excess_scaled[channel] = excess_scaled[channel];
        }
    }

    ++s_detector.hit_samples;
    if (s_detector.hit_samples == kHitWindowSamples) {
        finish_hit_window();
    }
}

void sensor_processor_init(void)
{
    if (s_inited) {
        return;
    }
    detector_reset();
    s_frame_count = 0;
    s_session_id = esp_random();
    if (s_session_id == 0) s_session_id = 1;
    s_next_sequence = 1;
    memset(s_pending_hits, 0, sizeof(s_pending_hits));
    portENTER_CRITICAL(&s_stats_mux);
    s_hits_count = 0;
    portEXIT_CRITICAL(&s_stats_mux);
    s_inited = true;
    xTaskCreate(hit_ack_watchdog_task, "hit_ack_watch", 3072, nullptr, 3,
                &s_ack_watchdog_task);
    ESP_LOGI(TAG, "initialized: channels=%u baseline=%u threshold=%d window=%u",
             (unsigned)kChannelCount, (unsigned)kBaselineSamples,
             (int)kHitThreshold, (unsigned)kHitWindowSamples);
}

void sensor_processor_set_params(size_t channel_count,
                                 float ema_alpha_big,
                                 float ema_alpha_small,
                                 float hit_threshold_big,
                                 float hit_threshold_small,
                                 uint32_t refractory_ms)
{
    // Keep the old SET_SENSOR_PARAM packet for compatibility, but do not let
    // legacy floating-point tuning change the validated physical algorithm.
    (void)ema_alpha_big;
    (void)ema_alpha_small;
    (void)hit_threshold_big;
    (void)hit_threshold_small;
    (void)refractory_ms;
    ESP_LOGI(TAG, "sensor parameter request received: channels=%u (fixed algorithm retained)",
             (unsigned)channel_count);
    sensor_processor_reset();
}

void sensor_processor_set_mode(uint8_t mode)
{
    s_mode = mode;
    ESP_LOGI(TAG, "sensor mode=%u", (unsigned)mode);
}

void sensor_processor_start_calibration(void)
{
    sensor_processor_init();
    detector_reset();
    ESP_LOGI(TAG, "calibration requested; collecting %u-frame baseline",
             (unsigned)kBaselineSamples);
}

void sensor_processor_reset(void)
{
    sensor_processor_init();
    detector_reset();
    ESP_LOGI(TAG, "detector reset");
}

void sensor_processor_set_raw_log(bool enable)
{
    s_raw_log_enabled = enable;
    ESP_LOGI(TAG, "raw ADC log %s", enable ? "enabled" : "disabled");
}

void sensor_processor_get_stats(uint32_t *hits_count)
{
    portENTER_CRITICAL(&s_stats_mux);
    if (hits_count) {
        *hits_count = s_hits_count;
    }
    portEXIT_CRITICAL(&s_stats_mux);
}

bool sensor_processor_mark_hit_decision(
    const PRA_HIT_DECISION_EVENT_DATA *decision,
    uint32_t *elapsed_ms)
{
    if (elapsed_ms) *elapsed_ms = 0;
    if (!decision) return false;

    const TickType_t now = xTaskGetTickCount();
    bool on_time = false;
    portENTER_CRITICAL(&s_pending_mux);
    for (auto &pending : s_pending_hits) {
        if (!pending.active ||
            pending.candidate.session_id != decision->session_id ||
            pending.candidate.sequence != decision->sequence) {
            continue;
        }
        const uint32_t elapsed = (uint32_t)pdTICKS_TO_MS(now - pending.sent_tick);
        if (elapsed_ms) *elapsed_ms = elapsed;
        on_time = elapsed <= PR_HIT_ACK_TIMEOUT_MS;
        pending.active = false;
        break;
    }
    portEXIT_CRITICAL(&s_pending_mux);
    return on_time;
}

void sensor_processor_input_cb(const uint8_t *adc, size_t adc_len, void *user_ctx)
{
    (void)user_ctx;
    if (!adc || adc_len < kChannelCount) {
        return;
    }
    sensor_processor_init();
    ++s_frame_count;
    if (s_raw_log_enabled && (s_frame_count % 300U) == 0U) {
        ESP_LOGI(TAG, "raw frame=%" PRIu32 " adc=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                 s_frame_count, adc[0], adc[1], adc[2], adc[3], adc[4],
                 adc[5], adc[6], adc[7], adc[8], adc[9]);
    }
    detector_process(adc);
}
