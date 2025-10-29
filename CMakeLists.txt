cmake_minimum_required(VERSION 3.27)

# Initialize the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(app)
pico_sdk_init()
add_executable(app
    src/wrappers.c
)

if (STDIO_USB)
    pico_enable_stdio_uart(app 0)
    pico_enable_stdio_usb(app 1)
endif()

if (STDIO_UART)
    pico_enable_stdio_uart(app 1)
    pico_enable_stdio_usb(app 0)
endif()

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/config     # Where you place lwipopts.h
    # ${PICO_SDK_PATH}/lib/lwip
    # Other pico-sdk includes...
)

# Adjust libraries as needed
target_link_libraries(app
    pico_stdlib
    hardware_adc
    pico_cyw43_arch_none
    pico_cyw43_arch_lwip_threadsafe_background
    ${CMAKE_SOURCE_DIR}/zig-out/app.o
    # wrappers
    -Wl,--wrap=gpioc_bit_out_put
    -Wl,--wrap=gpioc_bit_oe_put
)

# Generate binary
pico_add_extra_outputs(app)
