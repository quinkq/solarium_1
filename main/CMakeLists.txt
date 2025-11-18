idf_component_register(SRCS "main.c" "main.h"
                    INCLUDE_DIRS "."
                    REQUIRES spi_flash nvs_flash i2cdev ads1115_helper impluvium tempesta fluctus stellaria telemetry hmi esp_wifi esp_event interval_config)
