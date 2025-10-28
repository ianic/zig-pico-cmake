const std = @import("std");

pub const c = @cImport({
    @cInclude("pico.h");
    @cInclude("stdio.h");
    @cInclude("pico/stdlib.h");
    // PICO W specific header
    @cInclude("pico/cyw43_arch.h");
});

extern fn gpio_init(gpio: u32) void;
extern fn gpio_set_dir(gpio: u32, out: bool) void;
extern fn gpio_put(gpio: u32, value: bool) void;

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
    pub fn init() void {
        if (!c.stdio_init_all()) {
            @panic("stdio init failed");
        }
    }
    pub const printf = c.printf;
    pub fn sleep(ms: u32) void {
        c.sleep_ms(ms);
    }

    pub fn print(buf: []const u8) void {
        _ = c.printf("%.*s", buf.len, &buf[0]);
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
    pub fn init() void {
        if (c.cyw43_arch_init() != 0) {
            @panic("cyw43 init failed");
        }
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
        gpio_set_dir(pin, dir == .output);
    }

    pub fn put(pin: u32, value: bool) void {
        gpio_put(pin, value);
    }
};

pub fn check(res: anytype) !void {
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
