// reference: https://zig.news/anders/call-c-from-zig-on-the-pico-47p

#include "hardware/gpio.h"

void __wrap_gpioc_bit_out_put(uint pin, bool val) {
    gpioc_bit_out_put(pin, val);
}

void __wrap_gpioc_bit_oe_put(uint pin, bool val) { gpioc_bit_oe_put(pin, val); }
