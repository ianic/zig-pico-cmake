const std = @import("std");
const pico = @import("pico.zig");
const stdio = pico.stdio;
const cyw = pico.cyw;
const gpio = pico.gpio;
const printf = pico.c.printf;
const conf = @import("config.zig");
const log = std.log.scoped(.io);

pub const std_options: std.Options = .{
    .log_level = .debug,
    .logFn = pico.logFn,
};

export fn main() c_int {
    _main() catch |err| {
        log.err("fatal {}", .{err});
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
    log.debug("wifi connected to ssid: '{s}'", .{conf.ssid});

    var udp: cyw.Udp = .{};
    try udp.init(conf.target, conf.port);

    var timer: pico.Timer = .{};
    try timer.init(500, onTimer);

    var i: u32 = 0;
    while (true) {
        cyw.ledPut(true);
        gpio.put(led_pin, false);
        stdio.sleep(250);

        cyw.ledPut(false);
        gpio.put(led_pin, true);
        stdio.sleep(1000);

        log.debug("loop run {}", .{i});
        //_ = printf("Hello world %d\n", i);
        i +%= 1;

        try udp.send("iso medo u ducan, nije reko dobar dan, ajde medo van nisi reko dobar dan\n");
    }
}

fn onTimer(_: ?*pico.Timer.T) callconv(.c) bool {
    log.debug("onTimer", .{});
    return true;
}
