const std = @import("std");
const log = std.log.scoped(.pico);
const c = @import("sdk.zig");

pub fn logFn(
    comptime level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    // reference: https://github.com/ZigEmbeddedGroup/microzig/blob/2d929d92607076131a11d79fcaba327cbb5eaa61/port/raspberrypi/rp2xxx/src/hal/uart.zig#L508
    const at: u64 = c.get_absolute_time();
    const seconds = at / std.time.us_per_s;
    const microseconds = at % std.time.us_per_s;

    const level_prefix = comptime "[{}.{:0>6}] " ++ level.asText();
    const prefix = comptime level_prefix ++ switch (scope) {
        .default => ": ",
        else => " (" ++ @tagName(scope) ++ "): ",
    };

    var buffer: [128]u8 = undefined;
    const buf = std.fmt.bufPrint(&buffer, prefix ++ format ++ "\n", .{ seconds, microseconds } ++ args) catch return;
    _ = c.printf("%.*s", buf.len, &buf[0]);
}

pub const stdio = struct {
    pub fn init() !void {
        try check(c.stdio_init_all());
    }
    pub const printf = c.printf;
    pub fn sleep(ms: u32) void {
        c.sleep_ms(ms);
    }
};

pub const Timer = struct {
    pub const T = c.repeating_timer_t;
    const Callback = *const fn (?*T) callconv(.c) bool;

    t: T = undefined,

    pub fn init(self: *Timer, delay: i32, callback: Callback) !void {
        try check(c.add_repeating_timer_ms(delay, callback, c.NULL, &self.t));
    }
};

// cyw43 chip
pub const cyw = struct {
    pub fn init() !void {
        try check(c.cyw43_arch_init());
    }

    pub fn ledPut(value: bool) void {
        c.cyw43_arch_gpio_put(c.CYW43_WL_GPIO_LED_PIN, value);
    }

    pub fn connect(ssid: [*c]const u8, pwd: [*c]const u8, timeout: u32) !void {
        c.cyw43_arch_enable_sta_mode();
        try check(c.cyw43_arch_wifi_connect_timeout_ms(ssid, pwd, c.CYW43_AUTH_WPA2_AES_PSK, timeout));
    }

    pub fn deinit() void {
        c.cyw43_arch_deinit();
    }

    pub const Udp = struct {
        pcb: *c.udp_pcb = undefined,
        addr: c.ip_addr_t = undefined,
        port: u16 = 0,

        pub fn init(self: *Udp, target: [*c]const u8, port: u16) !void {
            self.port = port;
            self.pcb = c.udp_new();
            try check(c.ipaddr_aton(target, &self.addr));
        }

        pub fn send(self: *Udp, data: []const u8) !void {
            const p: *c.struct_pbuf = c.pbuf_alloc(c.PBUF_TRANSPORT, @intCast(data.len), c.PBUF_RAM);
            defer _ = c.pbuf_free(p);
            const ptr: [*c]u8 = @ptrCast(p.payload.?);
            @memcpy(ptr, data);
            try check(c.udp_sendto(self.pcb, p, &self.addr, self.port));
        }
    };
};

pub const gpio = struct {
    pub const Direction = enum {
        input,
        output,
    };

    pub fn init(pin: u32, dir: Direction) void {
        c.gpio_init(pin);
        c.gpio_set_dir(pin, dir == .output);
    }

    pub fn put(pin: u32, value: bool) void {
        c.gpio_put(pin, value);
    }

    pub const Pin = struct {
        pin: u32 = 0,

        pub fn init(pin: u32) Pin {
            c.gpio_init(pin);
            return .{ .pin = pin };
        }

        pub fn setDir(self: Pin, dir: Direction) void {
            c.gpio_set_dir(self.pin, dir == .output);
        }

        pub fn put(self: Pin, value: bool) void {
            c.gpio_put(self.pin, value);
        }

        pub fn get(self: Pin) bool {
            return c.gpio_get(self.pin);
        }
    };
};

pub const adc = struct {
    pub fn init(input: u32) void {
        c.adc_init();
        c.adc_select_input(input);
    }

    pub fn initOnboardTemp() void {
        c.adc_init();
        c.adc_select_input(4);
        c.adc_set_temp_sensor_enabled(true);
    }

    pub fn read() u16 {
        return c.adc_read();
    }

    pub fn readOnboardTemperature() f32 {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const conversionFactor: f32 = @as(f32, 3.3) / (1 << 12);
        //log.debug("readOnboardTemperature1", .{});
        const sample = c.adc_read();
        //log.debug("readOnboardTemperature2", .{});
        const voltage: f32 = @as(f32, @floatFromInt(sample)) * conversionFactor;
        const tempC = 27.0 - (voltage - 0.706) / 0.001721;

        return tempC;
    }
};

fn check(res: anytype) !void {
    switch (@TypeOf(res)) {
        c.err_t, c_int => {
            if (res >= 0) return;
            if (res != c.ERR_OK) {
                _ = c.printf("something failed with %d\n", res);
            }
            switch (res) {
                c.ERR_OK => {},
                c.ERR_MEM => return error.Mem,
                c.ERR_BUF => return error.Buf,
                c.ERR_TIMEOUT => return error.Timeout,
                c.ERR_RTE => return error.Rte,
                c.ERR_INPROGRESS => return error.Inprogress,
                c.ERR_VAL => return error.Val,
                c.ERR_WOULDBLOCK => return error.WouldBlock,
                c.ERR_USE => return error.Use,
                c.ERR_ALREADY => return error.Already,
                c.ERR_ISCONN => return error.IsConn,
                c.ERR_CONN => return error.Conn,
                c.ERR_IF => return error.If,
                c.ERR_ABRT => return error.Abrt,
                c.ERR_RST => return error.Rst,
                c.ERR_CLSD => return error.Clsd,
                c.ERR_ARG => return error.Arg,
                else => return error.Unkonown,
            }
        },
        bool => {
            if (!res) {
                return error.Fail;
            }
        },
        else => @compileError("unknown type"),
    }
}

pub const OneWire = struct {
    pin: gpio.Pin = .{},

    pub fn init(pin: u32) OneWire {
        return .{ .pin = .init(pin) };
    }

    // Raises error if no devices are present on the data bus
    pub fn reset(self: OneWire) !void {
        self.pin.setDir(.output);

        // bring low for 480us
        self.pin.put(false);
        c.sleep_us(480);

        self.pin.setDir(.input); // let the data line float high
        c.sleep_us(70);
        const presence = !self.pin.get(); // see if any devices are pulling the data line low
        c.sleep_us(410);

        if (!presence) return error.NoDevices;
    }

    fn putBit(self: OneWire, bit: bool) void {
        self.pin.setDir(.output);
        self.pin.put(false);
        c.sleep_us(3);
        if (bit) {
            self.pin.put(true);
            c.sleep_us(55);
        } else {
            c.sleep_us(60);
            self.pin.put(true);
            c.sleep_us(5);
        }
    }

    fn getBit(self: OneWire) bool {
        self.pin.setDir(.output);
        self.pin.put(false);
        c.sleep_us(3);

        self.pin.setDir(.input);
        c.sleep_us(3);
        const result = self.pin.get();
        c.sleep_us(45);

        return result;
    }

    fn putByte(self: OneWire, b: u8) void {
        var byte = b;
        for (0..8) |_| {
            self.putBit(byte & 0x01 > 0);
            byte = byte >> 1;
        }
    }

    fn getByte(self: OneWire) u8 {
        var byte: u8 = 0;
        for (0..8) |_| {
            byte = byte >> 1;
            if (self.getBit()) {
                byte = byte | 0x80;
            }
        }
        return byte;
    }
};

// DS18B20
pub const TempSensor = struct {
    ow: OneWire,

    pub fn init(pin: u32) TempSensor {
        return .{ .ow = .init(pin) };
    }

    // Initiates a single temperature conversion
    // Leave 750ms before reading
    pub fn convert(self: TempSensor) !void {
        try self.ow.reset();
        self.ow.putByte(0xcc);
        self.ow.putByte(0x44);
    }

    pub fn read(self: TempSensor) !f32 {
        // read the contents of the scratchpad
        try self.ow.reset();
        self.ow.putByte(0xcc);
        self.ow.putByte(0xbe);
        var res: [9]u8 = @splat(0);
        for (&res) |*r| {
            const b = self.ow.getByte();
            r.* = b;
        }
        // check crc
        if (crc(res[0..8]) != res[8]) return error.CrcFail;
        // temperature bytes
        const lsb = res[0];
        const msb = res[1];
        const temp: u16 = (@as(u16, @intCast(msb)) << 8 | lsb);
        return @as(f32, @floatFromInt(temp)) / 16;
    }

    fn crc(bytes: []const u8) u8 {
        var res: u8 = 0;
        for (bytes) |byte| {
            var b = byte;
            for (0..8) |_| {
                const tmp = ((res ^ b) & 0x01) > 0;
                res >>= 1;
                if (tmp)
                    res ^= 0x8C;
                b >>= 1;
            }
        }
        return res;
    }

    test crc {
        const data: [8]u8 = .{ 112, 1, 0, 0, 127, 225, 60, 170 };
        try std.testing.expectEqual(103, crc(&data));
    }
};

test {
    _ = TempSensor;
}
