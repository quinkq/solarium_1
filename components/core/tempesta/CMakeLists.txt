idf_component_register(SRCS "tempesta_sensors.c" "tempesta_processing.c" "tempesta_accumulator.c" "tempesta_wind.c" "tempesta.c"
                    INCLUDE_DIRS "."
                    REQUIRES main driver freertos esp_timer nvs_flash fluctus interval_config
                    PRIV_REQUIRES sht4x bmp280 as5600 i2cdev esp_idf_lib_helpers)