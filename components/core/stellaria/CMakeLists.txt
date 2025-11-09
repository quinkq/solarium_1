# components/stellaria/CMakeLists.txt
idf_component_register(SRCS "stellaria.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_common fluctus telemetry)