idf_component_register(
    SRCS "hmi_render_system.c" "hmi_render_impluvium.c" "hmi_render_tempesta.c" "hmi_render_fluctus.c" "hmi_navigation.c" "hmi_display.c" "hmi.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_lcd
        freertos
        telemetry
        fluctus
        impluvium
        tempesta
        stellaria
        interval_config
        wifi_helper
        mcp23008_helper
)
