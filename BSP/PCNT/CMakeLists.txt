idf_component_register(
    SRCS "pcnt_encoder.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_pcnt esp_timer
)
