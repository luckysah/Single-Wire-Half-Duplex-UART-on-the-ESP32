idf_component_register(SRCS "dxl.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart
                       PRIV_REQUIRES esp_driver_gpio esp_timer log)
