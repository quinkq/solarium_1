idf_component_register(
    SRCS "hmi.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_lcd
        freertos
        fluctus
        impluvium
        tempesta
        stellaria
)
