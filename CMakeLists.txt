cmake_minimum_required(VERSION 3.5)

# ES8388 I2C address override: CE/AD0 tied to 3.3V = 0x22 (8-bit address)
#add_compile_definitions(ES8388_ADDR=0x22)

# Read version from version.txt
file(READ "${CMAKE_CURRENT_LIST_DIR}/version.txt" PROJECT_VERSION)
string(STRIP "${PROJECT_VERSION}" PROJECT_VERSION)
string(REPLACE "." "_" PROJECT_VERSION_UNDERSCORE "${PROJECT_VERSION}")

# Pass version to code as a compile definition
add_compile_definitions(BTAUDIO_VERSION="${PROJECT_VERSION}")

# Add ADF components to the build
set(EXTRA_COMPONENT_DIRS
    $ENV{ADF_PATH}/components
    ${CMAKE_CURRENT_LIST_DIR}/components
)

# Exclude speech recognition - we don't need it and it pulls in buggy esp-dsp
# set(EXCLUDE_COMPONENTS esp-sr)

# Suppress deprecated warnings from ADF/IDF components
add_compile_options(-Wno-deprecated-declarations -Wno-cpp)

# Pull in ADF version info
include($ENV{ADF_PATH}/CMakeLists.txt)

# Pull in ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(BTAudio VERSION ${PROJECT_VERSION})

# Create merged binary after build
set(MERGED_BIN_NAME "${CMAKE_PROJECT_NAME}_${PROJECT_VERSION_UNDERSCORE}.bin")
add_custom_command(
    TARGET app
    POST_BUILD
    COMMAND ${ESPTOOLPY} --chip esp32 merge_bin
        -o "${CMAKE_BINARY_DIR}/${MERGED_BIN_NAME}"
        --flash_mode dio
        --flash_size 8MB
        --flash_freq 40m
        0x1000 "${CMAKE_BINARY_DIR}/bootloader/bootloader.bin"
        0x8000 "${CMAKE_BINARY_DIR}/partition_table/partition-table.bin"
        0x20000 "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMENT "Creating merged binary: ${MERGED_BIN_NAME}"
)