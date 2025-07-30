idf_component_register(SRCS "as5600.c"
                    INCLUDE_DIRS "."
                    REQUIRES i2cdev log esp_idf_lib_helpers) 