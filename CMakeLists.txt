cmake_minimum_required(VERSION 3.27)

# Initialize the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(mlem)
pico_sdk_init()
add_executable(mlem
    src/wrappers.c
)

if (STDIO_USB)
    pico_enable_stdio_uart(mlem 0)
    pico_enable_stdio_usb(mlem 1)
endif()

if (STDIO_UART)
    pico_enable_stdio_uart(mlem 1)
    pico_enable_stdio_usb(mlem 0)   
endif()

# Adjust libraries as needed
target_link_libraries(mlem
    pico_stdlib
    pico_cyw43_arch_none
    ${CMAKE_SOURCE_DIR}/zig-out/mlem.o
    # wrappers
    -Wl,--wrap=gpio_set_dir
    -Wl,--wrap=gpio_put
    # -Wl,--wrap=gpio_get
)

# Generate binary
pico_add_extra_outputs(mlem)
