# components/as5600/CMakeLists.txt
idf_component_register(SRCS "as5600.c"
                        INCLUDE_DIRS "." # Header is in the same directory
                        REQUIRES i2cdev esp_idf_lib_helpers)