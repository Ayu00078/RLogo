#include "pr_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern "C" void app_main(void)
{
    pr_app_init();
    pr_app_start();
    vTaskSuspend(nullptr);
}