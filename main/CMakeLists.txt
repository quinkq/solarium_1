idf_component_register(SRCS "main.c" "main.h"
                    INCLUDE_DIRS "."
                    REQUIRES spi_flash nvs_flash bmp280 sht4x ads111x i2cdev as5600 ina219 irrigation esp_wifi esp_event)
