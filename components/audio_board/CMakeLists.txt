# Custom audio_board component for ESP-ADF
# This replaces the default ADF audio_board with custom pin definitions

idf_component_register(
    SRCS
        "my_board_v1_0/board.c"
        "my_board_v1_0/board_pins_config.c"
    INCLUDE_DIRS
        "include"
        "my_board_v1_0"
    REQUIRES
        driver
        esp_peripherals
        ads1115
    PRIV_REQUIRES
        audio_sal
        audio_hal
        bd37033
        esp_dispatcher
        display_service
)
