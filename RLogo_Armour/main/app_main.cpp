
#include "armour_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern "C" void app_main(void)
{
    armour_app_init();
    armour_app_start();
    vTaskSuspend(nullptr);
}