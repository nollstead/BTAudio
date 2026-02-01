idf_component_register(
    SRCS "board.c" "board_pins_config.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES audio_hal driver
)