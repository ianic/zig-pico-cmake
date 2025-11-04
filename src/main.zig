const std = @import("std");
const hal = @import("hal.zig");
const stdio = hal.stdio;
const cyw = hal.cyw;
const gpio = hal.gpio;
const adc = hal.adc;

const conf = @import("config.zig");
const log = std.log.scoped(.io);

pub const std_options: std.Options = .{
    .log_level = .debug,
    .logFn = hal.logFn,
};

export fn main() c_int {
    _main() catch |err| {
        log.err("fatal {}", .{err});
        return -1;
    };
    return 0;
}

fn _main() !void {
    try stdio.init();
    adc.initOnboardTemp();
    try cyw.init();

    const led_pin = 15;
    gpio.init(led_pin, .output);

    try cyw.connect(conf.ssid, conf.pwd, 30000);
    log.debug("wifi connected to ssid: '{s}'", .{conf.ssid});

    var udp: cyw.Udp = .{};
    try udp.init(conf.target, conf.port);

    var timer: hal.Timer = .{};
    try timer.init(5000, onTimer);

    var timer2: hal.Timer = .{};
    try timer2.init(10000, onTimer2);

    //const ow: hal.OneWire = .init(22);
    const ts: hal.TempSensor = .init(22);

    var i: u32 = 0;
    while (true) {
        ts.convert() catch |err| {
            log.err("temperature sensor convert {}", .{err});
        };

        cyw.ledPut(true);
        gpio.put(led_pin, false);
        stdio.sleep(250);

        cyw.ledPut(false);
        gpio.put(led_pin, true);
        stdio.sleep(1000);

        const tmp = adc.readOnboardTemperature();
        log.debug("loop run: {} tmp: {}", .{ i, tmp });
        i +%= 1;

        try udp.send("iso medo u ducan, nije reko dobar dan, ajde medo van nisi reko dobar dan\n");

        if (ts.read()) |temp| {
            log.info("temp: {}", .{temp});
        } else |err| {
            log.err("temperature sensor read {}", .{err});
        }
    }
}

fn onTimer(_: ?*hal.Timer.T) callconv(.c) bool {
    log.debug("onTimer", .{});
    return true;
}

fn onTimer2(_: ?*hal.Timer.T) callconv(.c) bool {
    log.debug("onTimer2", .{});
    return true;
}
