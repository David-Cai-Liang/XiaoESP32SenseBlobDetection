#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>

// Direct pin definitions matching schematic
#define PIN_SCL GPIO_NUM_6
#define PIN_SDA GPIO_NUM_5
#define PIN_INT GPIO_NUM_8
#define MPU_ADDR MPU6050_I2C_ADDRESS_LOW // 0x68

static const char *TAG = "mpu6050_test";
static TaskHandle_t mpu_task_handle = NULL;

// Interrupt Service Routine executed when MPU6050 fires the INT pin
static void IRAM_ATTR mpu_gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void mpu6050_test(void *pvParameters)
{
    // Save current task handle for ISR notifications
    mpu_task_handle = xTaskGetCurrentTaskHandle();

    mpu6050_dev_t dev = { 0 };

    // Initialize descriptor[cite: 1]
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, MPU_ADDR, 0, PIN_SDA, PIN_SCL));

    // Enable internal pull-ups[cite: 1]
    dev.i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev.i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;

    ESP_LOGI(TAG, "Probing MPU6050 on SDA=GPIO%d, SCL=GPIO%d, Address=0x%02x...", PIN_SDA, PIN_SCL, MPU_ADDR);

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "SUCCESS: Found MPU60x0 device!");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found (error 0x%x)", res);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    // --- Configure MPU6050 Interrupt Output ---
    // Enables the Data Ready interrupt on MPU6050
    ESP_ERROR_CHECK(mpu6050_set_int_enabled(&dev, true));

    // --- Configure ESP32 GPIO Interrupt Pin ---
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_INT),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,     // MPU6050 INT pin defaults to Active-High
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Install GPIO ISR service and attach handler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_INT, mpu_gpio_isr_handler, NULL));

    ESP_LOGI(TAG, "Interrupt driver configured on GPIO%d. Waiting for sensor triggers...", PIN_INT);

    while (1)
    {
        // Block task until unblocked by ISR (or 1 second timeout)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        if (notified > 0)
        {
            float temp;
            mpu6050_acceleration_t accel = {0};
            mpu6050_rotation_t rotation = {0};

            ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
            ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

            ESP_LOGI(TAG, "Accel: x=%.2f y=%.2f z=%.2f Gyro: x=%.2f y=%.2f z=%.2f | Temp: %.1f°C",
                     accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, temp);
        }
        else
        {
            ESP_LOGW(TAG, "Interrupt timeout: No data received in 1 second.");
        }
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}
