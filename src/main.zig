const std = @import("std");
const pico = @import("pico.zig");
const stdio = pico.stdio;
const cyw = pico.cyw;
const gpio = pico.gpio;
const printf = pico.c.printf;

export fn main() c_int {
    stdio.init();
    cyw.init();

    const led_pin = 15;
    gpio.init(led_pin, .output);

    var i: u32 = 0;
    while (true) {
        cyw.ledPut(true);
        gpio.put(led_pin, false);
        stdio.sleep(250);

        cyw.ledPut(false);
        gpio.put(led_pin, true);
        stdio.sleep(100);

        _ = printf("Hello world %d\n", i);
        i +%= 1;
    }
}
