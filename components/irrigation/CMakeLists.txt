# components/irrigation/CMakeLists.txt
idf_component_register(SRCS "irrigation.c"
                    INCLUDE_DIRS "."
                    REQUIRES ads1115_helper ads111x abp i2cdev weather_station fluctus)