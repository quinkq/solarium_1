# components/fluctus/CMakeLists.txt
idf_component_register(SRCS "fluctus_accumulator.c" "fluctus_thermal.c" "fluctus_solar_tracking.c" "fluctus_power_monitor.c" "fluctus_power_bus.c" "fluctus.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer driver ads111x ina219 i2cdev ads1115_helper stellaria impluvium tempesta esp_common freertos ds18x20 telemetry interval_config)