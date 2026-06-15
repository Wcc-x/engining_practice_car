idf_component_register(
    SRCS "ax_hal.c"
         "ax_robot.c"
         "ax_kinematics.c"
         "ax_speed.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
             esp_driver_ledc
             esp_driver_pcnt
             esp_timer
             freertos
             LEDC
             PCNT
)
