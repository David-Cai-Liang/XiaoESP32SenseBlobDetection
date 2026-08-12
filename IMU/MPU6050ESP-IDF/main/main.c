#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>

// Direct pin definitions matching Max Imagination's schematic
#define PIN_SCL GPIO_NUM_6
#define PIN_SDA GPIO_NUM_5
#define MPU_ADDR MPU6050_I2C_ADDRESS_LOW // 0x68 (AD0 connected to GND)

static const char *TAG = "mpu6050_test";

void mpu6050_test(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    // Initialize descriptor
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, MPU_ADDR, 0, PIN_SDA, PIN_SCL));

    // Enable internal pull-ups
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

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = {0};
        mpu6050_rotation_t rotation = {0};

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        ESP_LOGI(TAG, "Accel: x=%.2f y=%.2f z=%.2f Gyro: x=%.2f y=%.2f z=%.2f | Temp: %.1f°C", accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, temp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}
