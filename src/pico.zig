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
