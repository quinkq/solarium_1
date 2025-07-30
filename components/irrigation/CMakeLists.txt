# components/irrigation/CMakeLists.txt
idf_component_register(SRCS "irrigation.c"
                    INCLUDE_DIRS "."
                    REQUIRES main ads111x abp i2cdev)