# Base source files (always compiled)
set(TELEMETRY_SRCS
    "telemetry.c"
    "telemetry_cache.c"
    "telemetry_mqtt_buffer.c"
    "telemetry_msgpack.c"
)

# Base requirements (always needed)
set(TELEMETRY_REQS
    esp_timer
    nvs_flash
    littlefs
    msgpack
    stellaria
    fluctus
    tempesta
    impluvium
    wifi_helper
    solar_calc
    interval_config
)

# Conditionally add MQTT sources and requirements
if(CONFIG_TELEMETRY_MQTT_ENABLE)
    list(APPEND TELEMETRY_SRCS
        "telemetry_mqtt_client.c"
        "telemetry_commands.c"
    )
    list(APPEND TELEMETRY_REQS mqtt)
endif()

idf_component_register(
    SRCS ${TELEMETRY_SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${TELEMETRY_REQS}
)
