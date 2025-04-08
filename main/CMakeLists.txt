idf_component_register(
    SRCS "main.c"
         "motorcontrol.c"
         "ultrasonic.c"
         "mpu.c"
         "encoder.c"
         "wifi_logger.c"
         "surroundings.c"
    INCLUDE_DIRS "."
)
