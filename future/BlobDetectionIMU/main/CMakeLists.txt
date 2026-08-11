idf_component_register(
    SRCS "BlobDetectionIMU.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera esp_jpeg vfs mpu6050 i2cdev
)
