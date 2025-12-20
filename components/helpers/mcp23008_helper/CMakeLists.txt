idf_component_register(SRCS "mcp23008_helper.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos esp_common mcp23008 i2cdev main)
