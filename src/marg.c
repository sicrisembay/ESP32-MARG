#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "../include/marg.h"
#include "../sensor/marg_sensor.h"

static const char *TAG = "marg";
static TaskHandle_t marg_task_handle = NULL;

static dataXYZf_t accel;
static dataXYZf_t gyro;
static dataXYZf_t mag;

static void _marg_task(void *pArg)
{
    if(ESP_OK != marg_sensor_reg_notification(marg_task_handle)) {
        ESP_LOGE(TAG, "Unable to register to sensor");
    }
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(ESP_OK == marg_sensor_get_data(&accel, &gyro, &mag)) {

        }
    }
}

marg_ret_t marg_init(void) {
    accel.x = 0.0f;
    accel.y = 0.0f;
    accel.z = 0.0f;
    gyro.x = 0.0f;
    gyro.y = 0.0f;
    gyro.z = 0.0f;
    mag.x = 0.0f;
    mag.y = 0.0f;
    mag.z = 0.0f;

    if(!marg_sensor_init()) {
        return MARG_ERROR_INIT;
    }
#if defined(CONFIG_MARG_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
            _marg_task,
            "marg",
            CONFIG_MARG_STACK_SIZE,
            NULL,
            CONFIG_MARG_TASK_PRIORITY,
            &marg_task_handle,
            PRO_CPU_NUM)) {
#elif defined(CONFIG_MARG_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
            _marg_task,
            "marg",
            CONFIG_MARG_STACK_SIZE,
            NULL,
            CONFIG_MARG_TASK_PRIORITY,
            &marg_task_handle,
            APP_CPU_NUM)) {
#else
#endif
        return MARG_ERROR_INIT;
    }
    return MARG_NOERROR;
}
