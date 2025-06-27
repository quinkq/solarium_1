idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 sht4x ads111x i2cdev as5600 ina219)
