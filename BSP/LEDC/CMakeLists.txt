idf_component_register(
    SRCS "ledc.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc
)
