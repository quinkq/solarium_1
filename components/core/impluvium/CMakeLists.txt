# components/core/impluvium/CMakeLists.txt
idf_component_register(
    SRCS
        "impluvium.c"
        "impluvium_sensors.c"
        "impluvium_actuators.c"
        "impluvium_learning.c"
        "impluvium_state_machine.c"
        "impluvium_safety.c"
        "impluvium_storage.c"
    INCLUDE_DIRS "."
    REQUIRES
        ads1115_helper
        interval_config
        abp
        i2cdev
        tempesta
        fluctus
        stellaria
        telemetry
)