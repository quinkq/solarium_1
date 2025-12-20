idf_component_register(SRCS "ads1115_helper.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos esp_common ads111x i2cdev fluctus main)