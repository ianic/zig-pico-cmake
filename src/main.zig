const std = @import("std");
const pico = @import("pico.zig");
const stdio = pico.stdio;
const cyw = pico.cyw;
const gpio = pico.gpio;
const printf = pico.c.printf;
const conf = @import("config.zig");

export fn main() c_int {
    _main() catch |err| {
        const str = @errorName(err);
        _ = printf("fatal error %s\n", &str[0]);
        return -1;
    };
    return 0;
}

fn _main() !void {
    stdio.init();
    cyw.init();

    const led_pin = 15;
    gpio.init(led_pin, .output);

    try cyw.connect(conf.ssid, conf.pwd, 30000);
    _ = printf("wifi connected to ssid: '%s'\n", conf.ssid);

    var udp: cyw.Udp = .{};
    try udp.init(conf.target, conf.port);

    var i: u32 = 0;
    while (true) {
        cyw.ledPut(true);
        gpio.put(led_pin, false);
        stdio.sleep(250);

        cyw.ledPut(false);
        gpio.put(led_pin, true);
        stdio.sleep(1000);

        _ = printf("Hello world %d\n", i);
        i +%= 1;

        try udp.send("iso medo u ducan, nije reko dobar dan, ajde medo van nisi reko dobar dan\n");
    }
}
