#include "esp_log.h"

extern "C" void controller_start(void);

extern "C" void app_main(void)
{
    controller_start();
}
