# components/fluctus/CMakeLists.txt
idf_component_register(SRCS "fluctus_accumulator.c" "fluctus_thermal.c" "fluctus_solar_tracking.c" "fluctus_power_monitor.c" "fluctus_power_bus.c" "fluctus.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer esp_common freertos driver ina219 ads1115_helper stellaria impluvium tempesta telemetry ds18x20 interval_config wifi_helper solar_calc mcp23008_helper main)