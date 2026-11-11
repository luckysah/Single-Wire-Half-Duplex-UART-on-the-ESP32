idf_component_register(SRCS "hw_test_main.c"
                       PRIV_REQUIRES dxl mpu6050 esp_driver_gpio esp_driver_i2c esp_timer spi_flash esp_hw_support)
