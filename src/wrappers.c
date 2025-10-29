// reference: https://zig.news/anders/call-c-from-zig-on-the-pico-47p

#include "hardware/adc.h"
#include "hardware/gpio.h"

void __wrap_gpio_set_dir(uint gpio, bool out) { gpio_set_dir(gpio, out); }

void __wrap_gpio_put(uint gpio, bool value) { gpio_put(gpio, value); }

uint16_t __wrap_adc_read() { return adc_read(); }
