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

    pub fn connect(ssid: [*c]const u8, pwd: [*c]const u8, timeout: u32) isize {
        c.cyw43_arch_enable_sta_mode();
        return c.cyw43_arch_wifi_connect_timeout_ms(ssid, pwd, c.CYW43_AUTH_WPA2_AES_PSK, timeout);
    }

    pub fn send() void {
        const target = "192.168.190.235";
        const port = 4242;
        const pcb: *c.udp_pcb = c.udp_new();

        var addr: c.ip_addr_t = undefined;
        var r = c.ipaddr_aton(target, &addr);
        _ = c.printf("aton res %d\n", r);

        var i: usize = 0;
        while (true) : (i +%= 1) {
            const fmt = "udp send {d}\n";
            const cnt: u16 = @intCast(std.fmt.count(fmt, .{i}));
            const p: *c.struct_pbuf = c.pbuf_alloc(c.PBUF_TRANSPORT, cnt + 1, c.PBUF_RAM);
            var buf: []u8 = undefined;
            buf.ptr = @ptrCast(p.payload.?);
            buf.len = cnt + 1;
            _ = std.fmt.bufPrintZ(buf, fmt, .{i}) catch {
                _ = c.printf("bufPrintZ no space \n");
            };

            const er = c.udp_sendto(pcb, p, &addr, port);
            _ = c.printf("send %d\n", er);
            r = c.pbuf_free(p);
            _ = c.printf("free res %d\n", r);
            stdio.sleep(1000);
        }
    }

    pub fn deinit() void {
        c.cyw43_arch_deinit();
    }
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
