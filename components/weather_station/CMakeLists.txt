idf_component_register(SRCS "weather_station.c"
                    INCLUDE_DIRS "."
                    REQUIRES main driver freertos esp_timer nvs_flash fluctus
                    PRIV_REQUIRES sht4x bmp280 as5600 i2cdev esp_idf_lib_helpers)