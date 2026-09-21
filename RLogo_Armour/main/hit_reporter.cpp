// #include "hit_reporter.h"
// #include "sensor_processor.h"
// #include "PowerRune_Events.h"
// #include "esp_event.h"

// extern esp_event_loop_handle_t pr_events_loop_handle;

// static void on_hit(uint8_t ring, void* user_ctx)
// {
//     (void)user_ctx;
//     PRA_HIT_EVENT_DATA hit = {};
//     hit.address = 0xFF;
//     hit.data_len = sizeof(PRA_HIT_EVENT_DATA);
//     hit.score = ring;

//     esp_event_post_to(pr_events_loop_handle, PRA, PRA_HIT_EVENT, &hit, sizeof(hit), 0);
// }

// void hit_reporter_init(void)
// {
//     sensor_processor_set_hit_callback(on_hit, nullptr);
// }