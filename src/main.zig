const std = @import("std");
const pico = @import("pico.zig");
const stdio = pico.stdio;
const cyw = pico.cyw;
const gpio = pico.gpio;
const printf = pico.c.printf;
const secrets = @import("secrets.zig");

export fn main() c_int {
    stdio.init();
    cyw.init();

    const led_pin = 15;
    gpio.init(led_pin, .output);

    const res = cyw.connect(secrets.ssid, secrets.pwd, 30000);
    _ = printf("wifi connect to ssid: '%s' result: %d\n", secrets.ssid, res);

    var i: u32 = 0;
    while (true) {
        cyw.ledPut(true);
        gpio.put(led_pin, false);
        stdio.sleep(250);

        cyw.ledPut(false);
        gpio.put(led_pin, true);
        stdio.sleep(200);

        _ = printf("Hello world %d\n", i);
        i +%= 1;

        cyw.send();
    }
}
