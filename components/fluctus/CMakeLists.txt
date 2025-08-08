# components/fluctus/CMakeLists.txt
idf_component_register(SRCS "fluctus.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer driver ads111x ina219 i2cdev ads1115_helper stellaria irrigation weather_station esp_common freertos)