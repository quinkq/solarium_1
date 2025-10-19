idf_component_register(
    SRCS "hmi.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_lcd
        freertos
        fluctus
        irrigation
        weather_station
        stellaria
)
