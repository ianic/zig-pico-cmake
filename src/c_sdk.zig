pub const __builtin = @import("std").zig.c_translation.builtins;
pub const __helpers = @import("std").zig.c_translation.helpers;

pub const wint_t = c_int;
pub extern fn __assert([*c]const u8, c_int, [*c]const u8) noreturn;
pub extern fn __assert_func([*c]const u8, c_int, [*c]const u8, [*c]const u8) noreturn;
pub const intmax_t = c_longlong;
pub const uintmax_t = c_ulonglong;
pub const int_least64_t = c_longlong;
pub const int_least32_t = c_int;
pub const int_least16_t = c_short;
pub const int_least8_t = i8;
pub const uint_least64_t = c_ulonglong;
pub const uint_least32_t = c_uint;
pub const uint_least16_t = c_ushort;
pub const uint_least8_t = u8;
pub const int_fast64_t = c_longlong;
pub const int_fast32_t = c_int;
pub const int_fast16_t = c_short;
pub const int_fast8_t = i8;
pub const uint_fast64_t = c_ulonglong;
pub const uint_fast32_t = c_uint;
pub const uint_fast16_t = c_ushort;
pub const uint_fast8_t = u8;
pub const ptrdiff_t = c_int;
pub const wchar_t = c_uint;
pub const max_align_t = extern struct {
    __aro_max_align_ll: c_longlong = 0,
    __aro_max_align_ld: c_longdouble = 0,
};
pub const uint = c_uint;
pub const absolute_time_t = u64;
pub fn to_us_since_boot(arg_t: absolute_time_t) callconv(.c) u64 {
    var t = arg_t;
    _ = &t;
    return t;
}
pub fn update_us_since_boot(arg_t: [*c]absolute_time_t, arg_us_since_boot: u64) callconv(.c) void {
    var t = arg_t;
    _ = &t;
    var us_since_boot = arg_us_since_boot;
    _ = &us_since_boot;
    t.* = us_since_boot;
}
pub fn from_us_since_boot(arg_us_since_boot: u64) callconv(.c) absolute_time_t {
    var us_since_boot = arg_us_since_boot;
    _ = &us_since_boot;
    var t: absolute_time_t = undefined;
    _ = &t;
    update_us_since_boot(&t, us_since_boot);
    return t;
}
pub const __int8_t = i8;
pub const __uint8_t = u8;
pub const __int16_t = c_short;
pub const __uint16_t = c_ushort;
pub const __int32_t = c_int;
pub const __uint32_t = c_uint;
pub const __int64_t = c_longlong;
pub const __uint64_t = c_ulonglong;
pub const __int_least8_t = i8;
pub const __uint_least8_t = u8;
pub const __int_least16_t = c_short;
pub const __uint_least16_t = c_ushort;
pub const __int_least32_t = c_int;
pub const __uint_least32_t = c_uint;
pub const __int_least64_t = c_longlong;
pub const __uint_least64_t = c_ulonglong;
pub const __intmax_t = c_longlong;
pub const __uintmax_t = c_ulonglong;
pub const __intptr_t = c_long;
pub const __uintptr_t = c_ulong;
pub inline fn __compiler_memory_barrier() void {}
pub extern fn panic_unsupported() noreturn;
pub extern fn panic(fmt: [*c]const u8, ...) noreturn;
pub fn running_on_fpga() callconv(.c) bool {
    return @as(c_int, 0) != 0;
}
pub fn running_in_sim() callconv(.c) bool {
    return @as(c_int, 0) != 0;
}
pub fn busy_wait_at_least_cycles(arg_minimum_cycles: u32) callconv(.c) void {
    var minimum_cycles = arg_minimum_cycles;
    _ = &minimum_cycles;
}
pub inline fn __breakpoint() void {}
pub inline fn get_core_num() uint {
    return @as([*c]u32, @ptrFromInt(@as(c_uint, 3489660928) +% @as(c_uint, 0))).*;
}
pub inline fn __get_current_exception() uint {
    var exception: uint = undefined;
    _ = &exception;
    return exception;
}
pub inline fn pico_processor_state_is_nonsecure() bool {
    var tt: u32 = undefined;
    _ = &tt;
    return !((tt & (@as(c_uint, 1) << @intCast(22))) != 0);
}
pub extern fn rp2350_chip_version() u8;
pub fn rp2040_chip_version() callconv(.c) u8 {
    return 2;
}
pub fn rp2040_rom_version() callconv(.c) u8 {
    return @as([*c]u8, @ptrFromInt(@as(usize, @intCast(@as(c_int, 19))))).*;
}
pub inline fn __mul_instruction(arg_a: i32, arg_b: i32) i32 {
    var a = arg_a;
    _ = &a;
    var b = arg_b;
    _ = &b;
    return a;
}
pub const PICO_OK: c_int = 0;
pub const PICO_ERROR_NONE: c_int = 0;
pub const PICO_ERROR_GENERIC: c_int = -1;
pub const PICO_ERROR_TIMEOUT: c_int = -2;
pub const PICO_ERROR_NO_DATA: c_int = -3;
pub const PICO_ERROR_NOT_PERMITTED: c_int = -4;
pub const PICO_ERROR_INVALID_ARG: c_int = -5;
pub const PICO_ERROR_IO: c_int = -6;
pub const PICO_ERROR_BADAUTH: c_int = -7;
pub const PICO_ERROR_CONNECT_FAILED: c_int = -8;
pub const PICO_ERROR_INSUFFICIENT_RESOURCES: c_int = -9;
pub const PICO_ERROR_INVALID_ADDRESS: c_int = -10;
pub const PICO_ERROR_BAD_ALIGNMENT: c_int = -11;
pub const PICO_ERROR_INVALID_STATE: c_int = -12;
pub const PICO_ERROR_BUFFER_TOO_SMALL: c_int = -13;
pub const PICO_ERROR_PRECONDITION_NOT_MET: c_int = -14;
pub const PICO_ERROR_MODIFIED_DATA: c_int = -15;
pub const PICO_ERROR_INVALID_DATA: c_int = -16;
pub const PICO_ERROR_NOT_FOUND: c_int = -17;
pub const PICO_ERROR_UNSUPPORTED_MODIFICATION: c_int = -18;
pub const PICO_ERROR_LOCK_REQUIRED: c_int = -19;
pub const PICO_ERROR_VERSION_MISMATCH: c_int = -20;
pub const PICO_ERROR_RESOURCE_IN_USE: c_int = -21;
pub const enum_pico_error_codes = c_int;
pub const io_rw_64 = u64;
pub const io_ro_64 = u64;
pub const io_wo_64 = u64;
pub const io_rw_32 = u32;
pub const io_ro_32 = u32;
pub const io_wo_32 = u32;
pub const io_rw_16 = u16;
pub const io_ro_16 = u16;
pub const io_wo_16 = u16;
pub const io_rw_8 = u8;
pub const io_ro_8 = u8;
pub const io_wo_8 = u8;
pub const ioptr = [*c]volatile u8;
pub const const_ioptr = ioptr;
pub inline fn hw_set_bits(arg_addr: [*c]volatile io_rw_32, arg_mask: u32) void {
    var addr = arg_addr;
    _ = &addr;
    var mask = arg_mask;
    _ = &mask;
    @as([*c]volatile io_rw_32, @ptrCast(@alignCast(@as(?*anyopaque, @ptrFromInt(@as(usize, @as(c_uint, 2) << @intCast(12)) +% @as(usize, @intCast(@intFromPtr(@as(?*volatile anyopaque, @ptrCast(@alignCast(addr))))))))))).* = mask;
}
pub inline fn hw_clear_bits(arg_addr: [*c]volatile io_rw_32, arg_mask: u32) void {
    var addr = arg_addr;
    _ = &addr;
    var mask = arg_mask;
    _ = &mask;
    @as([*c]volatile io_rw_32, @ptrCast(@alignCast(@as(?*anyopaque, @ptrFromInt(@as(usize, @as(c_uint, 3) << @intCast(12)) +% @as(usize, @intCast(@intFromPtr(@as(?*volatile anyopaque, @ptrCast(@alignCast(addr))))))))))).* = mask;
}
pub inline fn hw_xor_bits(arg_addr: [*c]volatile io_rw_32, arg_mask: u32) void {
    var addr = arg_addr;
    _ = &addr;
    var mask = arg_mask;
    _ = &mask;
    @as([*c]volatile io_rw_32, @ptrCast(@alignCast(@as(?*anyopaque, @ptrFromInt(@as(usize, @as(c_uint, 1) << @intCast(12)) +% @as(usize, @intCast(@intFromPtr(@as(?*volatile anyopaque, @ptrCast(@alignCast(addr))))))))))).* = mask;
}
pub inline fn hw_write_masked(arg_addr: [*c]volatile io_rw_32, arg_values: u32, arg_write_mask: u32) void {
    var addr = arg_addr;
    _ = &addr;
    var values = arg_values;
    _ = &values;
    var write_mask = arg_write_mask;
    _ = &write_mask;
    hw_xor_bits(addr, (addr.* ^ values) & write_mask);
}
pub const accessctrl_hw_t = extern struct {
    lock: io_rw_32 = 0,
    force_core_ns: io_rw_32 = 0,
    cfgreset: io_wo_32 = 0,
    gpio_nsmask: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    rom: io_rw_32 = 0,
    xip_main: io_rw_32 = 0,
    sram: [10]io_rw_32 = @import("std").mem.zeroes([10]io_rw_32),
    dma: io_rw_32 = 0,
    usbctrl: io_rw_32 = 0,
    pio: [3]io_rw_32 = @import("std").mem.zeroes([3]io_rw_32),
    coresight_trace: io_rw_32 = 0,
    coresight_periph: io_rw_32 = 0,
    sysinfo: io_rw_32 = 0,
    resets: io_rw_32 = 0,
    io_bank: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    pads_bank0: io_rw_32 = 0,
    pads_qspi: io_rw_32 = 0,
    busctrl: io_rw_32 = 0,
    adc0: io_rw_32 = 0,
    hstx: io_rw_32 = 0,
    i2c: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    pwm: io_rw_32 = 0,
    spi: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    timer: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    uart: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    otp: io_rw_32 = 0,
    tbman: io_rw_32 = 0,
    powman: io_rw_32 = 0,
    trng: io_rw_32 = 0,
    sha256: io_rw_32 = 0,
    syscfg: io_rw_32 = 0,
    clocks: io_rw_32 = 0,
    xosc: io_rw_32 = 0,
    rosc: io_rw_32 = 0,
    pll_sys: io_rw_32 = 0,
    pll_usb: io_rw_32 = 0,
    ticks: io_rw_32 = 0,
    watchdog: io_rw_32 = 0,
    rsm: io_rw_32 = 0,
    xip_ctrl: io_rw_32 = 0,
    xip_qmi: io_rw_32 = 0,
    xip_aux: io_rw_32 = 0,
};
comptime {
    if (!(@sizeOf(accessctrl_hw_t) == @as(c_uint, 236))) @compileError("static assertion failed \"\"");
}
pub const adc_hw_t = extern struct {
    cs: io_rw_32 = 0,
    result: io_ro_32 = 0,
    fcs: io_rw_32 = 0,
    fifo: io_ro_32 = 0,
    div: io_rw_32 = 0,
    intr: io_ro_32 = 0,
    inte: io_rw_32 = 0,
    intf: io_rw_32 = 0,
    ints: io_ro_32 = 0,
};
comptime {
    if (!(@sizeOf(adc_hw_t) == @as(c_uint, 36))) @compileError("static assertion failed \"\"");
}
pub const interp_hw_t = extern struct {
    accum: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    base: [3]io_rw_32 = @import("std").mem.zeroes([3]io_rw_32),
    pop: [3]io_ro_32 = @import("std").mem.zeroes([3]io_ro_32),
    peek: [3]io_ro_32 = @import("std").mem.zeroes([3]io_ro_32),
    ctrl: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    add_raw: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    base01: io_wo_32 = 0,
};
comptime {
    if (!(@sizeOf(interp_hw_t) == @as(c_uint, 64))) @compileError("static assertion failed \"\"");
}
pub const sio_hw_t = extern struct {
    cpuid: io_ro_32 = 0,
    gpio_in: io_ro_32 = 0,
    gpio_hi_in: io_ro_32 = 0,
    _pad0: u32 = 0,
    gpio_out: io_rw_32 = 0,
    gpio_hi_out: io_rw_32 = 0,
    gpio_set: io_wo_32 = 0,
    gpio_hi_set: io_wo_32 = 0,
    gpio_clr: io_wo_32 = 0,
    gpio_hi_clr: io_wo_32 = 0,
    gpio_togl: io_wo_32 = 0,
    gpio_hi_togl: io_wo_32 = 0,
    gpio_oe: io_rw_32 = 0,
    gpio_hi_oe: io_rw_32 = 0,
    gpio_oe_set: io_wo_32 = 0,
    gpio_hi_oe_set: io_wo_32 = 0,
    gpio_oe_clr: io_wo_32 = 0,
    gpio_hi_oe_clr: io_wo_32 = 0,
    gpio_oe_togl: io_wo_32 = 0,
    gpio_hi_oe_togl: io_wo_32 = 0,
    fifo_st: io_rw_32 = 0,
    fifo_wr: io_wo_32 = 0,
    fifo_rd: io_ro_32 = 0,
    spinlock_st: io_ro_32 = 0,
    _pad1: [8]u32 = @import("std").mem.zeroes([8]u32),
    interp: [2]interp_hw_t = @import("std").mem.zeroes([2]interp_hw_t),
    spinlock: [32]io_rw_32 = @import("std").mem.zeroes([32]io_rw_32),
    doorbell_out_set: io_rw_32 = 0,
    doorbell_out_clr: io_rw_32 = 0,
    doorbell_in_set: io_rw_32 = 0,
    doorbell_in_clr: io_rw_32 = 0,
    peri_nonsec: io_rw_32 = 0,
    _pad2: [3]u32 = @import("std").mem.zeroes([3]u32),
    riscv_softirq: io_rw_32 = 0,
    mtime_ctrl: io_rw_32 = 0,
    _pad3: [2]u32 = @import("std").mem.zeroes([2]u32),
    mtime: io_rw_32 = 0,
    mtimeh: io_rw_32 = 0,
    mtimecmp: io_rw_32 = 0,
    mtimecmph: io_rw_32 = 0,
    tmds_ctrl: io_rw_32 = 0,
    tmds_wdata: io_wo_32 = 0,
    tmds_peek_single: io_ro_32 = 0,
    tmds_pop_single: io_ro_32 = 0,
    tmds_peek_double_l0: io_ro_32 = 0,
    tmds_pop_double_l0: io_ro_32 = 0,
    tmds_peek_double_l1: io_ro_32 = 0,
    tmds_pop_double_l1: io_ro_32 = 0,
    tmds_peek_double_l2: io_ro_32 = 0,
    tmds_pop_double_l2: io_ro_32 = 0,
};
comptime {
    if (!(@sizeOf(sio_hw_t) == @as(c_uint, 488))) @compileError("static assertion failed \"\"");
}
pub const pads_bank0_hw_t = extern struct {
    voltage_select: io_rw_32 = 0,
    io: [48]io_rw_32 = @import("std").mem.zeroes([48]io_rw_32),
};
comptime {
    if (!(@sizeOf(pads_bank0_hw_t) == @as(c_uint, 196))) @compileError("static assertion failed \"\"");
}
pub const GPIO_FUNC_HSTX: c_int = 0;
pub const GPIO_FUNC_SPI: c_int = 1;
pub const GPIO_FUNC_UART: c_int = 2;
pub const GPIO_FUNC_I2C: c_int = 3;
pub const GPIO_FUNC_PWM: c_int = 4;
pub const GPIO_FUNC_SIO: c_int = 5;
pub const GPIO_FUNC_PIO0: c_int = 6;
pub const GPIO_FUNC_PIO1: c_int = 7;
pub const GPIO_FUNC_PIO2: c_int = 8;
pub const GPIO_FUNC_GPCK: c_int = 9;
pub const GPIO_FUNC_XIP_CS1: c_int = 9;
pub const GPIO_FUNC_CORESIGHT_TRACE: c_int = 9;
pub const GPIO_FUNC_USB: c_int = 10;
pub const GPIO_FUNC_UART_AUX: c_int = 11;
pub const GPIO_FUNC_NULL: c_int = 31;
pub const enum_gpio_function_rp2350 = c_uint;
pub const gpio_function_t = enum_gpio_function_rp2350;
pub const io_bank0_status_ctrl_hw_t = extern struct {
    status: io_ro_32 = 0,
    ctrl: io_rw_32 = 0,
};
pub const io_bank0_irq_ctrl_hw_t = extern struct {
    inte: [6]io_rw_32 = @import("std").mem.zeroes([6]io_rw_32),
    intf: [6]io_rw_32 = @import("std").mem.zeroes([6]io_rw_32),
    ints: [6]io_ro_32 = @import("std").mem.zeroes([6]io_ro_32),
};
const struct_unnamed_2 = extern struct {
    proc0_irq_ctrl: io_bank0_irq_ctrl_hw_t = @import("std").mem.zeroes(io_bank0_irq_ctrl_hw_t),
    proc1_irq_ctrl: io_bank0_irq_ctrl_hw_t = @import("std").mem.zeroes(io_bank0_irq_ctrl_hw_t),
    dormant_wake_irq_ctrl: io_bank0_irq_ctrl_hw_t = @import("std").mem.zeroes(io_bank0_irq_ctrl_hw_t),
};
const union_unnamed_1 = extern union {
    unnamed_0: struct_unnamed_2,
    irq_ctrl: [3]io_bank0_irq_ctrl_hw_t,
};
pub const io_bank0_hw_t = extern struct {
    io: [48]io_bank0_status_ctrl_hw_t = @import("std").mem.zeroes([48]io_bank0_status_ctrl_hw_t),
    _pad0: [32]u32 = @import("std").mem.zeroes([32]u32),
    irqsummary_proc0_secure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    irqsummary_proc0_nonsecure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    irqsummary_proc1_secure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    irqsummary_proc1_nonsecure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    irqsummary_dormant_wake_secure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    irqsummary_dormant_wake_nonsecure: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    intr: [6]io_rw_32 = @import("std").mem.zeroes([6]io_rw_32),
    unnamed_0: union_unnamed_1 = @import("std").mem.zeroes(union_unnamed_1),
};
comptime {
    if (!(@sizeOf(io_bank0_hw_t) == @as(c_uint, 800))) @compileError("static assertion failed \"\"");
}
pub const TIMER0_IRQ_0: c_int = 0;
pub const TIMER0_IRQ_1: c_int = 1;
pub const TIMER0_IRQ_2: c_int = 2;
pub const TIMER0_IRQ_3: c_int = 3;
pub const TIMER1_IRQ_0: c_int = 4;
pub const TIMER1_IRQ_1: c_int = 5;
pub const TIMER1_IRQ_2: c_int = 6;
pub const TIMER1_IRQ_3: c_int = 7;
pub const PWM_IRQ_WRAP_0: c_int = 8;
pub const PWM_IRQ_WRAP_1: c_int = 9;
pub const DMA_IRQ_0: c_int = 10;
pub const DMA_IRQ_1: c_int = 11;
pub const DMA_IRQ_2: c_int = 12;
pub const DMA_IRQ_3: c_int = 13;
pub const USBCTRL_IRQ: c_int = 14;
pub const PIO0_IRQ_0: c_int = 15;
pub const PIO0_IRQ_1: c_int = 16;
pub const PIO1_IRQ_0: c_int = 17;
pub const PIO1_IRQ_1: c_int = 18;
pub const PIO2_IRQ_0: c_int = 19;
pub const PIO2_IRQ_1: c_int = 20;
pub const IO_IRQ_BANK0: c_int = 21;
pub const IO_IRQ_BANK0_NS: c_int = 22;
pub const IO_IRQ_QSPI: c_int = 23;
pub const IO_IRQ_QSPI_NS: c_int = 24;
pub const CLOCKS_IRQ: c_int = 30;
pub const SPI0_IRQ: c_int = 31;
pub const SPI1_IRQ: c_int = 32;
pub const UART0_IRQ: c_int = 33;
pub const UART1_IRQ: c_int = 34;
pub const ADC_IRQ_FIFO: c_int = 35;
pub const I2C0_IRQ: c_int = 36;
pub const I2C1_IRQ: c_int = 37;
pub const OTP_IRQ: c_int = 38;
pub const TRNG_IRQ: c_int = 39;
pub const PROC0_IRQ_CTI: c_int = 40;
pub const PROC1_IRQ_CTI: c_int = 41;
pub const PLL_SYS_IRQ: c_int = 42;
pub const PLL_USB_IRQ: c_int = 43;
pub const POWMAN_IRQ_POW: c_int = 44;
pub const POWMAN_IRQ_TIMER: c_int = 45;
pub const SPARE_IRQ_0: c_int = 46;
pub const SPARE_IRQ_1: c_int = 47;
pub const SPARE_IRQ_2: c_int = 48;
pub const SPARE_IRQ_3: c_int = 49;
pub const SPARE_IRQ_4: c_int = 50;
pub const SPARE_IRQ_5: c_int = 51;
pub const IRQ_COUNT: c_int = 52;
pub const enum_irq_num_rp2350 = c_uint;
pub const irq_num_t = enum_irq_num_rp2350;
pub const m33_hw_t = extern struct {
    itm_stim: [32]io_rw_32 = @import("std").mem.zeroes([32]io_rw_32),
    _pad0: [864]u32 = @import("std").mem.zeroes([864]u32),
    itm_ter0: io_rw_32 = 0,
    _pad1: [15]u32 = @import("std").mem.zeroes([15]u32),
    itm_tpr: io_rw_32 = 0,
    _pad2: [15]u32 = @import("std").mem.zeroes([15]u32),
    itm_tcr: io_rw_32 = 0,
    _pad3: [27]u32 = @import("std").mem.zeroes([27]u32),
    int_atready: io_ro_32 = 0,
    _pad4: u32 = 0,
    int_atvalid: io_rw_32 = 0,
    _pad5: u32 = 0,
    itm_itctrl: io_rw_32 = 0,
    _pad6: [46]u32 = @import("std").mem.zeroes([46]u32),
    itm_devarch: io_ro_32 = 0,
    _pad7: [3]u32 = @import("std").mem.zeroes([3]u32),
    itm_devtype: io_ro_32 = 0,
    itm_pidr4: io_ro_32 = 0,
    itm_pidr5: io_rw_32 = 0,
    itm_pidr6: io_rw_32 = 0,
    itm_pidr7: io_rw_32 = 0,
    itm_pidr0: io_ro_32 = 0,
    itm_pidr1: io_ro_32 = 0,
    itm_pidr2: io_ro_32 = 0,
    itm_pidr3: io_ro_32 = 0,
    itm_cidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    dwt_ctrl: io_rw_32 = 0,
    dwt_cyccnt: io_rw_32 = 0,
    _pad8: u32 = 0,
    dwt_exccnt: io_rw_32 = 0,
    _pad9: u32 = 0,
    dwt_lsucnt: io_rw_32 = 0,
    dwt_foldcnt: io_rw_32 = 0,
    _pad10: u32 = 0,
    dwt_comp0: io_rw_32 = 0,
    _pad11: u32 = 0,
    dwt_function0: io_rw_32 = 0,
    _pad12: u32 = 0,
    dwt_comp1: io_rw_32 = 0,
    _pad13: u32 = 0,
    dwt_function1: io_rw_32 = 0,
    _pad14: u32 = 0,
    dwt_comp2: io_rw_32 = 0,
    _pad15: u32 = 0,
    dwt_function2: io_rw_32 = 0,
    _pad16: u32 = 0,
    dwt_comp3: io_rw_32 = 0,
    _pad17: u32 = 0,
    dwt_function3: io_rw_32 = 0,
    _pad18: [984]u32 = @import("std").mem.zeroes([984]u32),
    dwt_devarch: io_ro_32 = 0,
    _pad19: [3]u32 = @import("std").mem.zeroes([3]u32),
    dwt_devtype: io_ro_32 = 0,
    dwt_pidr4: io_ro_32 = 0,
    dwt_pidr5: io_rw_32 = 0,
    dwt_pidr6: io_rw_32 = 0,
    dwt_pidr7: io_rw_32 = 0,
    dwt_pidr0: io_ro_32 = 0,
    dwt_pidr1: io_ro_32 = 0,
    dwt_pidr2: io_ro_32 = 0,
    dwt_pidr3: io_ro_32 = 0,
    dwt_cidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    fp_ctrl: io_rw_32 = 0,
    fp_remap: io_ro_32 = 0,
    fp_comp: [8]io_rw_32 = @import("std").mem.zeroes([8]io_rw_32),
    _pad20: [997]u32 = @import("std").mem.zeroes([997]u32),
    fp_devarch: io_ro_32 = 0,
    _pad21: [3]u32 = @import("std").mem.zeroes([3]u32),
    fp_devtype: io_ro_32 = 0,
    fp_pidr4: io_ro_32 = 0,
    fp_pidr5: io_rw_32 = 0,
    fp_pidr6: io_rw_32 = 0,
    fp_pidr7: io_rw_32 = 0,
    fp_pidr0: io_ro_32 = 0,
    fp_pidr1: io_ro_32 = 0,
    fp_pidr2: io_ro_32 = 0,
    fp_pidr3: io_ro_32 = 0,
    fp_cidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    _pad22: [11265]u32 = @import("std").mem.zeroes([11265]u32),
    ictr: io_ro_32 = 0,
    actlr: io_rw_32 = 0,
    _pad23: u32 = 0,
    syst_csr: io_rw_32 = 0,
    syst_rvr: io_rw_32 = 0,
    syst_cvr: io_rw_32 = 0,
    syst_calib: io_ro_32 = 0,
    _pad24: [56]u32 = @import("std").mem.zeroes([56]u32),
    nvic_iser: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad25: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_icer: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad26: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_ispr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad27: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_icpr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad28: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_iabr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad29: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_itns: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad30: [30]u32 = @import("std").mem.zeroes([30]u32),
    nvic_ipr: [16]io_rw_32 = @import("std").mem.zeroes([16]io_rw_32),
    _pad31: [560]u32 = @import("std").mem.zeroes([560]u32),
    cpuid: io_ro_32 = 0,
    icsr: io_rw_32 = 0,
    vtor: io_rw_32 = 0,
    aircr: io_rw_32 = 0,
    scr: io_rw_32 = 0,
    ccr: io_rw_32 = 0,
    shpr: [3]io_rw_32 = @import("std").mem.zeroes([3]io_rw_32),
    shcsr: io_rw_32 = 0,
    cfsr: io_rw_32 = 0,
    hfsr: io_rw_32 = 0,
    dfsr: io_rw_32 = 0,
    mmfar: io_rw_32 = 0,
    bfar: io_rw_32 = 0,
    _pad32: u32 = 0,
    id_pfr: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    id_dfr0: io_ro_32 = 0,
    id_afr0: io_ro_32 = 0,
    id_mmfr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    id_isar: [6]io_ro_32 = @import("std").mem.zeroes([6]io_ro_32),
    _pad33: u32 = 0,
    ctr: io_ro_32 = 0,
    _pad34: [2]u32 = @import("std").mem.zeroes([2]u32),
    cpacr: io_rw_32 = 0,
    nsacr: io_rw_32 = 0,
    mpu_type: io_ro_32 = 0,
    mpu_ctrl: io_rw_32 = 0,
    mpu_rnr: io_rw_32 = 0,
    mpu_rbar: io_rw_32 = 0,
    mpu_rlar: io_rw_32 = 0,
    mpu_rbar_a1: io_rw_32 = 0,
    mpu_rlar_a1: io_rw_32 = 0,
    mpu_rbar_a2: io_rw_32 = 0,
    mpu_rlar_a2: io_rw_32 = 0,
    mpu_rbar_a3: io_rw_32 = 0,
    mpu_rlar_a3: io_rw_32 = 0,
    _pad35: u32 = 0,
    mpu_mair: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad36: [2]u32 = @import("std").mem.zeroes([2]u32),
    sau_ctrl: io_rw_32 = 0,
    sau_type: io_ro_32 = 0,
    sau_rnr: io_rw_32 = 0,
    sau_rbar: io_rw_32 = 0,
    sau_rlar: io_rw_32 = 0,
    sfsr: io_rw_32 = 0,
    sfar: io_rw_32 = 0,
    _pad37: u32 = 0,
    dhcsr: io_rw_32 = 0,
    dcrsr: io_rw_32 = 0,
    dcrdr: io_rw_32 = 0,
    demcr: io_rw_32 = 0,
    _pad38: [2]u32 = @import("std").mem.zeroes([2]u32),
    dscsr: io_rw_32 = 0,
    _pad39: [61]u32 = @import("std").mem.zeroes([61]u32),
    stir: io_rw_32 = 0,
    _pad40: [12]u32 = @import("std").mem.zeroes([12]u32),
    fpccr: io_rw_32 = 0,
    fpcar: io_rw_32 = 0,
    fpdscr: io_rw_32 = 0,
    mvfr: [3]io_ro_32 = @import("std").mem.zeroes([3]io_ro_32),
    _pad41: [28]u32 = @import("std").mem.zeroes([28]u32),
    ddevarch: io_ro_32 = 0,
    _pad42: [3]u32 = @import("std").mem.zeroes([3]u32),
    ddevtype: io_ro_32 = 0,
    dpidr4: io_ro_32 = 0,
    dpidr5: io_rw_32 = 0,
    dpidr6: io_rw_32 = 0,
    dpidr7: io_rw_32 = 0,
    dpidr0: io_ro_32 = 0,
    dpidr1: io_ro_32 = 0,
    dpidr2: io_ro_32 = 0,
    dpidr3: io_ro_32 = 0,
    dcidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    _pad43: [51201]u32 = @import("std").mem.zeroes([51201]u32),
    trcprgctlr: io_rw_32 = 0,
    _pad44: u32 = 0,
    trcstatr: io_ro_32 = 0,
    trcconfigr: io_rw_32 = 0,
    _pad45: [3]u32 = @import("std").mem.zeroes([3]u32),
    trceventctl0r: io_rw_32 = 0,
    trceventctl1r: io_rw_32 = 0,
    _pad46: u32 = 0,
    trcstallctlr: io_rw_32 = 0,
    trctsctlr: io_rw_32 = 0,
    trcsyncpr: io_ro_32 = 0,
    trcccctlr: io_rw_32 = 0,
    _pad47: [17]u32 = @import("std").mem.zeroes([17]u32),
    trcvictlr: io_rw_32 = 0,
    _pad48: [47]u32 = @import("std").mem.zeroes([47]u32),
    trccntrldvr0: io_rw_32 = 0,
    _pad49: [15]u32 = @import("std").mem.zeroes([15]u32),
    trcidr8: io_ro_32 = 0,
    trcidr9: io_ro_32 = 0,
    trcidr10: io_ro_32 = 0,
    trcidr11: io_ro_32 = 0,
    trcidr12: io_ro_32 = 0,
    trcidr13: io_ro_32 = 0,
    _pad50: [10]u32 = @import("std").mem.zeroes([10]u32),
    trcimspec: io_ro_32 = 0,
    _pad51: [7]u32 = @import("std").mem.zeroes([7]u32),
    trcidr0: io_ro_32 = 0,
    trcidr1: io_ro_32 = 0,
    trcidr2: io_ro_32 = 0,
    trcidr3: io_ro_32 = 0,
    trcidr4: io_ro_32 = 0,
    trcidr5: io_ro_32 = 0,
    trcidr6: io_rw_32 = 0,
    trcidr7: io_rw_32 = 0,
    _pad52: [2]u32 = @import("std").mem.zeroes([2]u32),
    trcrsctlr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad53: [36]u32 = @import("std").mem.zeroes([36]u32),
    trcsscsr: io_rw_32 = 0,
    _pad54: [7]u32 = @import("std").mem.zeroes([7]u32),
    trcsspcicr: io_rw_32 = 0,
    _pad55: [19]u32 = @import("std").mem.zeroes([19]u32),
    trcpdcr: io_rw_32 = 0,
    trcpdsr: io_ro_32 = 0,
    _pad56: [755]u32 = @import("std").mem.zeroes([755]u32),
    trcitatbidr: io_rw_32 = 0,
    _pad57: [3]u32 = @import("std").mem.zeroes([3]u32),
    trcitiatbinr: io_rw_32 = 0,
    _pad58: u32 = 0,
    trcitiatboutr: io_rw_32 = 0,
    _pad59: [40]u32 = @import("std").mem.zeroes([40]u32),
    trcclaimset: io_rw_32 = 0,
    trcclaimclr: io_rw_32 = 0,
    _pad60: [4]u32 = @import("std").mem.zeroes([4]u32),
    trcauthstatus: io_ro_32 = 0,
    trcdevarch: io_ro_32 = 0,
    _pad61: [2]u32 = @import("std").mem.zeroes([2]u32),
    trcdevid: io_rw_32 = 0,
    trcdevtype: io_ro_32 = 0,
    trcpidr4: io_ro_32 = 0,
    trcpidr5: io_rw_32 = 0,
    trcpidr6: io_rw_32 = 0,
    trcpidr7: io_rw_32 = 0,
    trcpidr0: io_ro_32 = 0,
    trcpidr1: io_ro_32 = 0,
    trcpidr2: io_ro_32 = 0,
    trcpidr3: io_ro_32 = 0,
    trccidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    cticontrol: io_rw_32 = 0,
    _pad62: [3]u32 = @import("std").mem.zeroes([3]u32),
    ctiintack: io_rw_32 = 0,
    ctiappset: io_rw_32 = 0,
    ctiappclear: io_rw_32 = 0,
    ctiapppulse: io_rw_32 = 0,
    ctiinen: [8]io_rw_32 = @import("std").mem.zeroes([8]io_rw_32),
    _pad63: [24]u32 = @import("std").mem.zeroes([24]u32),
    ctiouten: [8]io_rw_32 = @import("std").mem.zeroes([8]io_rw_32),
    _pad64: [28]u32 = @import("std").mem.zeroes([28]u32),
    ctitriginstatus: io_ro_32 = 0,
    ctitrigoutstatus: io_ro_32 = 0,
    ctichinstatus: io_ro_32 = 0,
    _pad65: u32 = 0,
    ctigate: io_rw_32 = 0,
    asicctl: io_rw_32 = 0,
    _pad66: [871]u32 = @import("std").mem.zeroes([871]u32),
    itchout: io_rw_32 = 0,
    ittrigout: io_rw_32 = 0,
    _pad67: [2]u32 = @import("std").mem.zeroes([2]u32),
    itchin: io_ro_32 = 0,
    _pad68: [2]u32 = @import("std").mem.zeroes([2]u32),
    itctrl: io_rw_32 = 0,
    _pad69: [46]u32 = @import("std").mem.zeroes([46]u32),
    devarch: io_ro_32 = 0,
    _pad70: [2]u32 = @import("std").mem.zeroes([2]u32),
    devid: io_ro_32 = 0,
    devtype: io_ro_32 = 0,
    pidr4: io_ro_32 = 0,
    pidr5: io_rw_32 = 0,
    pidr6: io_rw_32 = 0,
    pidr7: io_rw_32 = 0,
    pidr0: io_ro_32 = 0,
    pidr1: io_ro_32 = 0,
    pidr2: io_ro_32 = 0,
    pidr3: io_ro_32 = 0,
    cidr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
};
comptime {
    if (!(@sizeOf(m33_hw_t) == @as(c_uint, 274432))) @compileError("static assertion failed \"\"");
}
pub const nvic_hw_t = extern struct {
    iser: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad0: [30]u32 = @import("std").mem.zeroes([30]u32),
    icer: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad1: [30]u32 = @import("std").mem.zeroes([30]u32),
    ispr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad2: [30]u32 = @import("std").mem.zeroes([30]u32),
    icpr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad3: [30]u32 = @import("std").mem.zeroes([30]u32),
    iabr: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad4: [30]u32 = @import("std").mem.zeroes([30]u32),
    itns: [2]io_rw_32 = @import("std").mem.zeroes([2]io_rw_32),
    _pad5: [30]u32 = @import("std").mem.zeroes([30]u32),
    ipr: [16]io_rw_32 = @import("std").mem.zeroes([16]io_rw_32),
};
comptime {
    if (!(@sizeOf(nvic_hw_t) == @as(c_uint, 832))) @compileError("static assertion failed \"\"");
}
pub const armv8m_scb_hw_t = extern struct {
    cpuid: io_ro_32 = 0,
    icsr: io_rw_32 = 0,
    vtor: io_rw_32 = 0,
    aircr: io_rw_32 = 0,
    scr: io_rw_32 = 0,
    ccr: io_rw_32 = 0,
    shpr: [3]io_rw_32 = @import("std").mem.zeroes([3]io_rw_32),
    shcsr: io_rw_32 = 0,
    cfsr: io_rw_32 = 0,
    hfsr: io_rw_32 = 0,
    dfsr: io_rw_32 = 0,
    mmfar: io_rw_32 = 0,
    bfar: io_rw_32 = 0,
    _pad0: u32 = 0,
    id_pfr: [2]io_ro_32 = @import("std").mem.zeroes([2]io_ro_32),
    id_dfr0: io_ro_32 = 0,
    id_afr0: io_ro_32 = 0,
    id_mmfr: [4]io_ro_32 = @import("std").mem.zeroes([4]io_ro_32),
    id_isar: [6]io_ro_32 = @import("std").mem.zeroes([6]io_ro_32),
    _pad1: u32 = 0,
    ctr: io_ro_32 = 0,
    _pad2: [2]u32 = @import("std").mem.zeroes([2]u32),
    cpacr: io_rw_32 = 0,
    nsacr: io_rw_32 = 0,
};
comptime {
    if (!(@sizeOf(armv8m_scb_hw_t) == @as(c_uint, 144))) @compileError("static assertion failed \"\"");
}
pub const irq_handler_t = ?*const fn () callconv(.c) void;
pub fn check_irq_param(arg_num: uint) callconv(.c) void {
    var num = arg_num;
    _ = &num;
    const static_local___func__ = struct {
        const __func__: [15:0]u8 = "check_irq_param".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(num >= @as(c_uint, 52))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_irq/include/hardware/irq.h", 198, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( num >= 52u)");
        }
    }
}
pub extern fn irq_set_priority(num: uint, hardware_priority: u8) void;
pub extern fn irq_get_priority(num: uint) uint;
pub extern fn irq_set_enabled(num: uint, enabled: bool) void;
pub extern fn irq_is_enabled(num: uint) bool;
pub extern fn irq_set_mask_enabled(mask: u32, enabled: bool) void;
pub extern fn irq_set_mask_n_enabled(n: uint, mask: u32, enabled: bool) void;
pub extern fn irq_set_exclusive_handler(num: uint, handler: irq_handler_t) void;
pub extern fn irq_get_exclusive_handler(num: uint) irq_handler_t;
pub extern fn irq_add_shared_handler(num: uint, handler: irq_handler_t, order_priority: u8) void;
pub extern fn irq_remove_handler(num: uint, handler: irq_handler_t) void;
pub extern fn irq_has_handler(num: uint) bool;
pub extern fn irq_has_shared_handler(num: uint) bool;
pub extern fn irq_get_vtable_handler(num: uint) irq_handler_t;
pub fn irq_clear(arg_int_num: uint) callconv(.c) void {
    var int_num = arg_int_num;
    _ = &int_num;
    @as([*c]nvic_hw_t, @ptrFromInt(@as(c_uint, 3758096384) +% @as(c_uint, 57600))).*.icpr[int_num / @as(uint, 32)] = @as(uint, 1) << @intCast(int_num % @as(uint, 32));
}
pub extern fn irq_set_pending(num: uint) void;
pub extern fn runtime_init_per_core_irq_priorities() void;
pub inline fn irq_init_priorities() void {
    runtime_init_per_core_irq_priorities();
}
pub extern fn user_irq_claim(irq_num: uint) void;
pub extern fn user_irq_unclaim(irq_num: uint) void;
pub extern fn user_irq_claim_unused(required: bool) c_int;
pub extern fn user_irq_is_claimed(irq_num: uint) bool;
pub extern fn __unhandled_user_irq() void;
pub const GPIO_OUT: c_int = 1;
pub const GPIO_IN: c_int = 0;
pub const enum_gpio_dir = c_uint;
pub const GPIO_IRQ_LEVEL_LOW: c_int = 1;
pub const GPIO_IRQ_LEVEL_HIGH: c_int = 2;
pub const GPIO_IRQ_EDGE_FALL: c_int = 4;
pub const GPIO_IRQ_EDGE_RISE: c_int = 8;
pub const enum_gpio_irq_level = c_uint;
pub const gpio_irq_callback_t = ?*const fn (gpio: uint, event_mask: u32) callconv(.c) void;
pub const GPIO_OVERRIDE_NORMAL: c_int = 0;
pub const GPIO_OVERRIDE_INVERT: c_int = 1;
pub const GPIO_OVERRIDE_LOW: c_int = 2;
pub const GPIO_OVERRIDE_HIGH: c_int = 3;
pub const enum_gpio_override = c_uint;
pub const GPIO_SLEW_RATE_SLOW: c_int = 0;
pub const GPIO_SLEW_RATE_FAST: c_int = 1;
pub const enum_gpio_slew_rate = c_uint;
pub const GPIO_DRIVE_STRENGTH_2MA: c_int = 0;
pub const GPIO_DRIVE_STRENGTH_4MA: c_int = 1;
pub const GPIO_DRIVE_STRENGTH_8MA: c_int = 2;
pub const GPIO_DRIVE_STRENGTH_12MA: c_int = 3;
pub const enum_gpio_drive_strength = c_uint;
pub fn check_gpio_param(arg_gpio: uint) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    const static_local___func__ = struct {
        const __func__: [16:0]u8 = "check_gpio_param".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(gpio >= @as(c_uint, 30))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h", 245, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( gpio >= 30u)");
        }
    }
}
pub extern fn gpio_set_function(gpio: uint, @"fn": gpio_function_t) void;
pub extern fn gpio_set_function_masked(gpio_mask: u32, @"fn": gpio_function_t) void;
pub extern fn gpio_set_function_masked64(gpio_mask: u64, @"fn": gpio_function_t) void;
pub extern fn gpio_get_function(gpio: uint) gpio_function_t;
pub extern fn gpio_set_pulls(gpio: uint, up: bool, down: bool) void;
pub fn gpio_pull_up(arg_gpio: uint) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    gpio_set_pulls(gpio, @as(c_int, 1) != 0, @as(c_int, 0) != 0);
}
pub fn gpio_is_pulled_up(arg_gpio: uint) callconv(.c) bool {
    var gpio = arg_gpio;
    _ = &gpio;
    return (@as([*c]pads_bank0_hw_t, @ptrFromInt(@as(c_uint, 1073971200))).*.io[gpio] & @as(c_uint, 8)) != @as(io_rw_32, 0);
}
pub fn gpio_pull_down(arg_gpio: uint) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    gpio_set_pulls(gpio, @as(c_int, 0) != 0, @as(c_int, 1) != 0);
}
pub fn gpio_is_pulled_down(arg_gpio: uint) callconv(.c) bool {
    var gpio = arg_gpio;
    _ = &gpio;
    return (@as([*c]pads_bank0_hw_t, @ptrFromInt(@as(c_uint, 1073971200))).*.io[gpio] & @as(c_uint, 4)) != @as(io_rw_32, 0);
}
pub fn gpio_disable_pulls(arg_gpio: uint) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    gpio_set_pulls(gpio, @as(c_int, 0) != 0, @as(c_int, 0) != 0);
}
pub extern fn gpio_set_irqover(gpio: uint, value: uint) void;
pub extern fn gpio_set_outover(gpio: uint, value: uint) void;
pub extern fn gpio_set_inover(gpio: uint, value: uint) void;
pub extern fn gpio_set_oeover(gpio: uint, value: uint) void;
pub extern fn gpio_set_input_enabled(gpio: uint, enabled: bool) void;
pub extern fn gpio_set_input_hysteresis_enabled(gpio: uint, enabled: bool) void;
pub extern fn gpio_is_input_hysteresis_enabled(gpio: uint) bool;
pub extern fn gpio_set_slew_rate(gpio: uint, slew: enum_gpio_slew_rate) void;
pub extern fn gpio_get_slew_rate(gpio: uint) enum_gpio_slew_rate;
pub extern fn gpio_set_drive_strength(gpio: uint, drive: enum_gpio_drive_strength) void;
pub extern fn gpio_get_drive_strength(gpio: uint) enum_gpio_drive_strength;
pub extern fn gpio_set_irq_enabled(gpio: uint, event_mask: u32, enabled: bool) void;
pub extern fn gpio_set_irq_callback(callback: gpio_irq_callback_t) void;
pub extern fn gpio_set_irq_enabled_with_callback(gpio: uint, event_mask: u32, enabled: bool, callback: gpio_irq_callback_t) void;
pub extern fn gpio_set_dormant_irq_enabled(gpio: uint, event_mask: u32, enabled: bool) void;
pub fn gpio_get_irq_event_mask(arg_gpio: uint) callconv(.c) u32 {
    var gpio = arg_gpio;
    _ = &gpio;
    check_gpio_param(gpio);
    var irq_ctrl_base: [*c]io_bank0_irq_ctrl_hw_t = if (get_core_num() != 0) &@as([*c]io_bank0_hw_t, @ptrFromInt(@as(c_uint, 1073905664))).*.unnamed_0.unnamed_0.proc1_irq_ctrl else &@as([*c]io_bank0_hw_t, @ptrFromInt(@as(c_uint, 1073905664))).*.unnamed_0.unnamed_0.proc0_irq_ctrl;
    _ = &irq_ctrl_base;
    var status_reg: [*c]const volatile io_ro_32 = &irq_ctrl_base.*.ints[gpio >> @intCast(3)];
    _ = &status_reg;
    return (status_reg.* >> @intCast(@as(uint, 4) *% (gpio & @as(c_uint, 7)))) & @as(c_uint, 15);
}
pub fn gpio_acknowledge_irq(arg_gpio: uint, arg_event_mask: u32) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var event_mask = arg_event_mask;
    _ = &event_mask;
    check_gpio_param(gpio);
    @as([*c]io_bank0_hw_t, @ptrFromInt(@as(c_uint, 1073905664))).*.intr[gpio / @as(uint, 8)] = event_mask << @intCast(@as(uint, 4) *% (gpio % @as(uint, 8)));
}
pub extern fn gpio_add_raw_irq_handler_with_order_priority_masked(gpio_mask: u32, handler: irq_handler_t, order_priority: u8) void;
pub extern fn gpio_add_raw_irq_handler_with_order_priority_masked64(gpio_mask: u64, handler: irq_handler_t, order_priority: u8) void;
pub fn gpio_add_raw_irq_handler_with_order_priority(arg_gpio: uint, arg_handler: irq_handler_t, arg_order_priority: u8) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var handler = arg_handler;
    _ = &handler;
    var order_priority = arg_order_priority;
    _ = &order_priority;
    check_gpio_param(gpio);
    gpio_add_raw_irq_handler_with_order_priority_masked(@as(c_uint, 1) << @intCast(gpio), handler, order_priority);
}
pub extern fn gpio_add_raw_irq_handler_masked(gpio_mask: u32, handler: irq_handler_t) void;
pub extern fn gpio_add_raw_irq_handler_masked64(gpio_mask: u64, handler: irq_handler_t) void;
pub fn gpio_add_raw_irq_handler(arg_gpio: uint, arg_handler: irq_handler_t) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var handler = arg_handler;
    _ = &handler;
    check_gpio_param(gpio);
    gpio_add_raw_irq_handler_masked(@as(c_uint, 1) << @intCast(gpio), handler);
}
pub extern fn gpio_remove_raw_irq_handler_masked(gpio_mask: u32, handler: irq_handler_t) void;
pub extern fn gpio_remove_raw_irq_handler_masked64(gpio_mask: u64, handler: irq_handler_t) void;
pub fn gpio_remove_raw_irq_handler(arg_gpio: uint, arg_handler: irq_handler_t) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var handler = arg_handler;
    _ = &handler;
    check_gpio_param(gpio);
    gpio_remove_raw_irq_handler_masked(@as(c_uint, 1) << @intCast(gpio), handler);
}
pub extern fn gpio_init(gpio: uint) void;
pub extern fn gpio_deinit(gpio: uint) void;
pub extern fn gpio_init_mask(gpio_mask: uint) void;
pub fn gpio_get(arg_gpio: uint) callconv(.c) bool {
    var gpio = arg_gpio;
    _ = &gpio;
    return (@as([*c]sio_hw_t, @ptrFromInt(@as(c_uint, 3489660928))).*.gpio_in & (@as(c_uint, 1) << @intCast(gpio))) != 0;
}
pub fn gpio_get_all() callconv(.c) u32 {
    return gpioc_lo_in_get();
}
pub fn gpio_get_all64() callconv(.c) u64 {
    return gpioc_hilo_in_get();
}
pub fn gpio_set_mask(arg_mask: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_lo_out_set(mask);
}
pub fn gpio_set_mask64(arg_mask: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_hilo_out_set(mask);
}
pub fn gpio_set_mask_n(arg_n: uint, arg_mask: u32) callconv(.c) void {
    var n = arg_n;
    _ = &n;
    var mask = arg_mask;
    _ = &mask;
    if (!(n != 0)) {
        gpio_set_mask(mask);
    } else if (n == @as(uint, 1)) {
        gpioc_hi_out_set(mask);
    }
}
pub fn gpio_clr_mask(arg_mask: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_lo_out_clr(mask);
}
pub fn gpio_clr_mask64(arg_mask: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_hilo_out_clr(mask);
}
pub fn gpio_clr_mask_n(arg_n: uint, arg_mask: u32) callconv(.c) void {
    var n = arg_n;
    _ = &n;
    var mask = arg_mask;
    _ = &mask;
    if (!(n != 0)) {
        gpio_clr_mask(mask);
    } else if (n == @as(uint, 1)) {
        gpioc_hi_out_clr(mask);
    }
}
pub fn gpio_xor_mask(arg_mask: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_lo_out_xor(mask);
}
pub fn gpio_xor_mask64(arg_mask: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_hilo_out_xor(mask);
}
pub fn gpio_xor_mask_n(arg_n: uint, arg_mask: u32) callconv(.c) void {
    var n = arg_n;
    _ = &n;
    var mask = arg_mask;
    _ = &mask;
    if (!(n != 0)) {
        gpio_xor_mask(mask);
    } else if (n == @as(uint, 1)) {
        gpioc_hi_out_xor(mask);
    }
}
pub fn gpio_put_masked(arg_mask: u32, arg_value: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    var value = arg_value;
    _ = &value;
    gpioc_lo_out_xor((gpioc_lo_out_get() ^ value) & mask);
}
pub fn gpio_put_masked64(arg_mask: u64, arg_value: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    var value = arg_value;
    _ = &value;
    gpioc_hilo_out_xor((gpioc_hilo_out_get() ^ value) & mask);
}
pub fn gpio_put_masked_n(arg_n: uint, arg_mask: u32, arg_value: u32) callconv(.c) void {
    var n = arg_n;
    _ = &n;
    var mask = arg_mask;
    _ = &mask;
    var value = arg_value;
    _ = &value;
    if (!(n != 0)) {
        gpio_put_masked(mask, value);
    } else if (n == @as(uint, 1)) {
        gpioc_hi_out_xor((gpioc_hi_out_get() ^ value) & mask);
    }
}
pub fn gpio_put_all(arg_value: u32) callconv(.c) void {
    var value = arg_value;
    _ = &value;
    gpioc_lo_out_put(value);
}
pub fn gpio_put_all64(arg_value: u64) callconv(.c) void {
    var value = arg_value;
    _ = &value;
    gpioc_hilo_out_put(value);
}
pub fn gpio_put(arg_gpio: uint, arg_value: bool) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var value = arg_value;
    _ = &value;
    gpioc_bit_out_put(gpio, value);
}
pub fn gpio_get_out_level(arg_gpio: uint) callconv(.c) bool {
    var gpio = arg_gpio;
    _ = &gpio;
    return (@as([*c]sio_hw_t, @ptrFromInt(@as(c_uint, 3489660928))).*.gpio_out & (@as(c_uint, 1) << @intCast(gpio))) != 0;
}
pub fn gpio_set_dir_out_masked(arg_mask: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_lo_oe_set(mask);
}
pub fn gpio_set_dir_out_masked64(arg_mask: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_hilo_oe_set(mask);
}
pub fn gpio_set_dir_in_masked(arg_mask: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_lo_oe_clr(mask);
}
pub fn gpio_set_dir_in_masked64(arg_mask: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    gpioc_hilo_oe_clr(mask);
}
pub fn gpio_set_dir_masked(arg_mask: u32, arg_value: u32) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    var value = arg_value;
    _ = &value;
    gpioc_lo_oe_xor((gpioc_lo_oe_get() ^ value) & mask);
}
pub fn gpio_set_dir_masked64(arg_mask: u64, arg_value: u64) callconv(.c) void {
    var mask = arg_mask;
    _ = &mask;
    var value = arg_value;
    _ = &value;
    gpioc_hilo_oe_xor((gpioc_hilo_oe_get() ^ value) & mask);
}
pub fn gpio_set_dir_all_bits(arg_values: u32) callconv(.c) void {
    var values = arg_values;
    _ = &values;
    gpioc_lo_oe_put(values);
}
pub fn gpio_set_dir_all_bits64(arg_values: u64) callconv(.c) void {
    var values = arg_values;
    _ = &values;
    gpioc_hilo_oe_put(values);
}
pub fn gpio_set_dir(arg_gpio: uint, arg_out: bool) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    var out = arg_out;
    _ = &out;
    gpioc_bit_oe_put(gpio, out);
}
pub fn gpio_is_dir_out(arg_gpio: uint) callconv(.c) bool {
    var gpio = arg_gpio;
    _ = &gpio;
    return (@as([*c]sio_hw_t, @ptrFromInt(@as(c_uint, 3489660928))).*.gpio_oe & (@as(c_uint, 1) << @intCast(gpio))) != 0;
}
pub fn gpio_get_dir(arg_gpio: uint) callconv(.c) uint {
    var gpio = arg_gpio;
    _ = &gpio;
    return @intFromBool(gpio_is_dir_out(gpio));
}
pub extern fn gpio_debug_pins_init() void;
pub extern fn adc_init() void;
pub fn adc_gpio_init(arg_gpio: uint) callconv(.c) void {
    var gpio = arg_gpio;
    _ = &gpio;
    const static_local___func__ = struct {
        const __func__: [13:0]u8 = "adc_gpio_init".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!((gpio < @as(c_uint, 26)) or (gpio >= ((@as(c_uint, 26) +% @as(c_uint, 5)) -% @as(c_uint, 1))))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h", 96, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( gpio < 26u || gpio >= 26u + 5u - 1)");
        }
    }
    gpio_set_function(gpio, GPIO_FUNC_NULL);
    gpio_disable_pulls(gpio);
    gpio_set_input_enabled(gpio, @as(c_int, 0) != 0);
}
pub fn adc_select_input(arg_input: uint) callconv(.c) void {
    var input = arg_input;
    _ = &input;
    const static_local___func__ = struct {
        const __func__: [16:0]u8 = "adc_select_input".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (input < @as(c_uint, 5)) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h", 119, @ptrCast(@alignCast(&static_local___func__.__func__)), "input < 5u");
        }
    }
    hw_write_masked(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, input << @intCast(12), 61440);
}
pub fn adc_get_selected_input() callconv(.c) uint {
    return (@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs & @as(c_uint, 61440)) >> @intCast(12);
}
pub fn adc_set_round_robin(arg_input_mask: uint) callconv(.c) void {
    var input_mask = arg_input_mask;
    _ = &input_mask;
    const static_local___func__ = struct {
        const __func__: [19:0]u8 = "adc_set_round_robin".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (input_mask < (@as(c_uint, 1) << @intCast(5))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h", 151, @ptrCast(@alignCast(&static_local___func__.__func__)), "input_mask < (1 << 5u)");
        }
    }
    hw_write_masked(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, input_mask << @intCast(16), 33488896);
}
pub fn adc_set_temp_sensor_enabled(arg_enable: bool) callconv(.c) void {
    var enable = arg_enable;
    _ = &enable;
    if (enable) {
        hw_set_bits(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, 2);
    } else {
        hw_clear_bits(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, 2);
    }
}
pub fn adc_read() callconv(.c) u16 {
    hw_set_bits(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, 4);
    while (!((@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs & @as(c_uint, 256)) != 0)) {
        tight_loop_contents();
    }
    return @truncate(@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.result);
}
pub fn adc_run(arg_run: bool) callconv(.c) void {
    var run = arg_run;
    _ = &run;
    if (run) {
        hw_set_bits(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, 8);
    } else {
        hw_clear_bits(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs, 8);
    }
}
pub fn adc_set_clkdiv(arg_clkdiv: f32) callconv(.c) void {
    var clkdiv = arg_clkdiv;
    _ = &clkdiv;
    const static_local___func__ = struct {
        const __func__: [14:0]u8 = "adc_set_clkdiv".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(clkdiv >= @as(f32, @floatFromInt(@as(c_uint, 1) << @intCast((@as(c_uint, 23) +% @as(c_uint, 1)) -% @as(c_uint, 8)))))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h", 205, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( clkdiv >= 1 << (23u + 1 - 8u))");
        }
    }
    const frac_bit_count: c_int = @bitCast(@as(c_uint, @truncate((@as(c_uint, 7) +% @as(c_uint, 1)) -% @as(c_uint, 0))));
    _ = &frac_bit_count;
    clkdiv += @as(f32, 0.5) / @as(f32, @floatFromInt(@as(c_int, 1) << @intCast(frac_bit_count)));
    @as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.div = @intFromFloat(clkdiv * @as(f32, @floatFromInt(@as(c_int, 1) << @intCast(frac_bit_count))));
}
pub fn adc_fifo_setup(arg_en: bool, arg_dreq_en: bool, arg_dreq_thresh: u16, arg_err_in_fifo: bool, arg_byte_shift: bool) callconv(.c) void {
    var en = arg_en;
    _ = &en;
    var dreq_en = arg_dreq_en;
    _ = &dreq_en;
    var dreq_thresh = arg_dreq_thresh;
    _ = &dreq_thresh;
    var err_in_fifo = arg_err_in_fifo;
    _ = &err_in_fifo;
    var byte_shift = arg_byte_shift;
    _ = &byte_shift;
    hw_write_masked(&@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.fcs, ((((@as(uint, @bitCast(@as(c_int, @intFromBool(!!en)))) << @intCast(0)) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!dreq_en)))) << @intCast(3))) | (@as(uint, dreq_thresh) << @intCast(24))) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!err_in_fifo)))) << @intCast(2))) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!byte_shift)))) << @intCast(1)), (((@as(c_uint, 1) | @as(c_uint, 8)) | @as(c_uint, 251658240)) | @as(c_uint, 4)) | @as(c_uint, 2));
}
pub fn adc_fifo_is_empty() callconv(.c) bool {
    return (@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.fcs & @as(c_uint, 256)) != 0;
}
pub fn adc_fifo_get_level() callconv(.c) u8 {
    return @truncate((@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.fcs & @as(c_uint, 983040)) >> @intCast(16));
}
pub fn adc_fifo_get() callconv(.c) u16 {
    return @truncate(@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.fifo);
}
pub fn adc_fifo_get_blocking() callconv(.c) u16 {
    while (adc_fifo_is_empty()) {
        tight_loop_contents();
    }
    return @truncate(@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.fifo);
}
pub fn adc_fifo_drain() callconv(.c) void {
    while (!((@as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.cs & @as(c_uint, 256)) != 0)) {
        tight_loop_contents();
    }
    while (!adc_fifo_is_empty()) {
        _ = adc_fifo_get();
    }
}
pub fn adc_irq_set_enabled(arg_enabled: bool) callconv(.c) void {
    var enabled = arg_enabled;
    _ = &enabled;
    @as([*c]adc_hw_t, @ptrFromInt(@as(c_uint, 1074397184))).*.inte = @bitCast(@as(c_int, @intFromBool(!!enabled)));
}
pub const timer_hw_t = extern struct {
    timehw: io_wo_32 = 0,
    timelw: io_wo_32 = 0,
    timehr: io_ro_32 = 0,
    timelr: io_ro_32 = 0,
    alarm: [4]io_rw_32 = @import("std").mem.zeroes([4]io_rw_32),
    armed: io_rw_32 = 0,
    timerawh: io_ro_32 = 0,
    timerawl: io_ro_32 = 0,
    dbgpause: io_rw_32 = 0,
    pause: io_rw_32 = 0,
    locked: io_rw_32 = 0,
    source: io_rw_32 = 0,
    intr: io_rw_32 = 0,
    inte: io_rw_32 = 0,
    intf: io_rw_32 = 0,
    ints: io_ro_32 = 0,
    pub const @"32" = timer_time_us_32;
    pub const @"64" = timer_time_us_64;
    pub const @"321" = timer_busy_wait_us_32;
    pub const us = timer_busy_wait_us;
    pub const ms = timer_busy_wait_ms;
    pub const until = timer_busy_wait_until;
    pub const reached = timer_time_reached;
    pub const claim = timer_hardware_alarm_claim;
    pub const unused = timer_hardware_alarm_claim_unused;
    pub const unclaim = timer_hardware_alarm_unclaim;
    pub const claimed = timer_hardware_alarm_is_claimed;
    pub const callback = timer_hardware_alarm_set_callback;
    pub const target = timer_hardware_alarm_set_target;
    pub const cancel = timer_hardware_alarm_cancel;
    pub const irq = timer_hardware_alarm_force_irq;
    pub const num = timer_hardware_alarm_get_irq_num;
    pub const index = timer_get_index;
};
comptime {
    if (!(@sizeOf(timer_hw_t) == @as(c_uint, 76))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(TIMER1_IRQ_3 == (TIMER0_IRQ_0 + @as(c_int, 7)))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(TIMER1_IRQ_3 == (TIMER0_IRQ_0 + @as(c_int, 7)))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(TIMER1_IRQ_3 == (TIMER0_IRQ_0 + @as(c_int, 7)))) @compileError("static assertion failed \"\"");
}
pub fn check_hardware_alarm_num_param(arg_alarm_num: uint) callconv(.c) void {
    var alarm_num = arg_alarm_num;
    _ = &alarm_num;
    const static_local___func__ = struct {
        const __func__: [30:0]u8 = "check_hardware_alarm_num_param".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(alarm_num >= @as(c_uint, 4))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_timer/include/hardware/timer.h", 189, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( alarm_num >= 4u)");
        }
    }
}
pub fn timer_time_us_32(arg_timer: [*c]timer_hw_t) callconv(.c) u32 {
    var timer = arg_timer;
    _ = &timer;
    return timer.*.timerawl;
}
pub fn time_us_32() callconv(.c) u32 {
    return timer_time_us_32(@as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074462720))));
}
pub extern fn timer_time_us_64(timer: [*c]timer_hw_t) u64;
pub extern fn time_us_64() u64;
pub extern fn timer_busy_wait_us_32(timer: [*c]timer_hw_t, delay_us: u32) void;
pub extern fn busy_wait_us_32(delay_us: u32) void;
pub extern fn timer_busy_wait_us(timer: [*c]timer_hw_t, delay_us: u64) void;
pub extern fn busy_wait_us(delay_us: u64) void;
pub extern fn timer_busy_wait_ms(timer: [*c]timer_hw_t, delay_ms: u32) void;
pub extern fn busy_wait_ms(delay_ms: u32) void;
pub extern fn timer_busy_wait_until(timer: [*c]timer_hw_t, t: absolute_time_t) void;
pub extern fn busy_wait_until(t: absolute_time_t) void;
pub fn timer_time_reached(arg_timer: [*c]timer_hw_t, arg_t: absolute_time_t) callconv(.c) bool {
    var timer = arg_timer;
    _ = &timer;
    var t = arg_t;
    _ = &t;
    var target: u64 = to_us_since_boot(t);
    _ = &target;
    var hi_target: u32 = @truncate(target >> @intCast(32));
    _ = &hi_target;
    var hi: u32 = timer.*.timerawh;
    _ = &hi;
    return (hi >= hi_target) and ((timer.*.timerawl >= @as(u32, @truncate(target))) or (hi != hi_target));
}
pub fn time_reached(arg_t: absolute_time_t) callconv(.c) bool {
    var t = arg_t;
    _ = &t;
    return timer_time_reached(@as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074462720))), t);
}
pub const hardware_alarm_callback_t = ?*const fn (alarm_num: uint) callconv(.c) void;
pub extern fn timer_hardware_alarm_claim(timer: [*c]timer_hw_t, alarm_num: uint) void;
pub extern fn hardware_alarm_claim(alarm_num: uint) void;
pub extern fn timer_hardware_alarm_claim_unused(timer: [*c]timer_hw_t, required: bool) c_int;
pub extern fn hardware_alarm_claim_unused(required: bool) c_int;
pub extern fn timer_hardware_alarm_unclaim(timer: [*c]timer_hw_t, alarm_num: uint) void;
pub extern fn hardware_alarm_unclaim(alarm_num: uint) void;
pub extern fn timer_hardware_alarm_is_claimed(timer: [*c]timer_hw_t, alarm_num: uint) bool;
pub extern fn hardware_alarm_is_claimed(alarm_num: uint) bool;
pub extern fn timer_hardware_alarm_set_callback(timer: [*c]timer_hw_t, alarm_num: uint, callback: hardware_alarm_callback_t) void;
pub extern fn hardware_alarm_set_callback(alarm_num: uint, callback: hardware_alarm_callback_t) void;
pub extern fn timer_hardware_alarm_set_target(timer: [*c]timer_hw_t, alarm_num: uint, t: absolute_time_t) bool;
pub extern fn hardware_alarm_set_target(alarm_num: uint, t: absolute_time_t) bool;
pub extern fn timer_hardware_alarm_cancel(timer: [*c]timer_hw_t, alarm_num: uint) void;
pub extern fn hardware_alarm_cancel(alarm_num: uint) void;
pub extern fn timer_hardware_alarm_force_irq(timer: [*c]timer_hw_t, alarm_num: uint) void;
pub extern fn hardware_alarm_force_irq(alarm_num: uint) void;
pub fn timer_hardware_alarm_get_irq_num(arg_timer: [*c]timer_hw_t, arg_alarm_num: uint) callconv(.c) uint {
    var timer = arg_timer;
    _ = &timer;
    var alarm_num = arg_alarm_num;
    _ = &alarm_num;
    check_hardware_alarm_num_param(alarm_num);
    return (@as(c_uint, TIMER0_IRQ_0) +% (@as(c_uint, @bitCast(@as(c_int, @intFromBool(timer == @as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074495488))))))) *% @as(c_uint, 4))) +% alarm_num;
}
pub fn hardware_alarm_get_irq_num(arg_alarm_num: uint) callconv(.c) uint {
    var alarm_num = arg_alarm_num;
    _ = &alarm_num;
    return timer_hardware_alarm_get_irq_num(@as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074462720))), alarm_num);
}
pub fn timer_get_index(arg_timer: [*c]timer_hw_t) callconv(.c) uint {
    var timer = arg_timer;
    _ = &timer;
    return @bitCast(@as(c_int, @intFromBool(timer == @as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074495488))))));
}
pub fn timer_get_instance(arg_timer_num: uint) callconv(.c) [*c]timer_hw_t {
    var timer_num = arg_timer_num;
    _ = &timer_num;
    const static_local___func__ = struct {
        const __func__: [18:0]u8 = "timer_get_instance".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(timer_num >= @as(c_uint, 2))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_timer/include/hardware/timer.h", 591, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( timer_num >= 2u)");
        }
    }
    return if (timer_num != 0) @as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074495488))) else @as([*c]timer_hw_t, @ptrFromInt(@as(c_uint, 1074462720)));
}
pub fn get_absolute_time() callconv(.c) absolute_time_t {
    var t: absolute_time_t = undefined;
    _ = &t;
    update_us_since_boot(&t, time_us_64());
    return t;
}
pub fn us_to_ms(arg_us: u64) callconv(.c) u32 {
    var us = arg_us;
    _ = &us;
    if ((us >> @intCast(32)) != 0) {
        return @truncate(us / @as(u64, 1000));
    } else {
        return @as(u32, @truncate(us)) / @as(c_uint, 1000);
    }
}
pub fn to_ms_since_boot(arg_t: absolute_time_t) callconv(.c) u32 {
    var t = arg_t;
    _ = &t;
    var us: u64 = to_us_since_boot(t);
    _ = &us;
    return us_to_ms(us);
}
pub fn delayed_by_us(t: absolute_time_t, arg_us: u64) callconv(.c) absolute_time_t {
    _ = &t;
    var us = arg_us;
    _ = &us;
    var t2: absolute_time_t = undefined;
    _ = &t2;
    var base: u64 = to_us_since_boot(t);
    _ = &base;
    var delayed: u64 = base +% us;
    _ = &delayed;
    if (@as(i64, @bitCast(@as(c_ulonglong, @truncate(delayed)))) < @as(i64, 0)) {
        delayed = @bitCast(@as(c_longlong, @as(c_longlong, 9223372036854775807)));
    }
    update_us_since_boot(&t2, delayed);
    return t2;
}
pub fn delayed_by_ms(t: absolute_time_t, arg_ms: u32) callconv(.c) absolute_time_t {
    _ = &t;
    var ms = arg_ms;
    _ = &ms;
    var t2: absolute_time_t = undefined;
    _ = &t2;
    var base: u64 = to_us_since_boot(t);
    _ = &base;
    var delayed: u64 = base +% (@as(c_ulonglong, ms) *% @as(c_ulonglong, 1000));
    _ = &delayed;
    if (@as(i64, @bitCast(@as(c_ulonglong, @truncate(delayed)))) < @as(i64, 0)) {
        delayed = @bitCast(@as(c_longlong, @as(c_longlong, 9223372036854775807)));
    }
    update_us_since_boot(&t2, delayed);
    return t2;
}
pub fn make_timeout_time_us(arg_us: u64) callconv(.c) absolute_time_t {
    var us = arg_us;
    _ = &us;
    return delayed_by_us(get_absolute_time(), us);
}
pub fn make_timeout_time_ms(arg_ms: u32) callconv(.c) absolute_time_t {
    var ms = arg_ms;
    _ = &ms;
    return delayed_by_ms(get_absolute_time(), ms);
}
pub fn absolute_time_diff_us(arg_from: absolute_time_t, arg_to: absolute_time_t) callconv(.c) i64 {
    var from = arg_from;
    _ = &from;
    var to = arg_to;
    _ = &to;
    return @bitCast(@as(c_ulonglong, @truncate(to_us_since_boot(to) -% to_us_since_boot(from))));
}
pub fn absolute_time_min(arg_a: absolute_time_t, arg_b: absolute_time_t) callconv(.c) absolute_time_t {
    var a = arg_a;
    _ = &a;
    var b = arg_b;
    _ = &b;
    return if (to_us_since_boot(a) < to_us_since_boot(b)) a else b;
}
pub extern const at_the_end_of_time: absolute_time_t;
pub fn is_at_the_end_of_time(arg_t: absolute_time_t) callconv(.c) bool {
    var t = arg_t;
    _ = &t;
    return to_us_since_boot(t) == to_us_since_boot(at_the_end_of_time);
}
pub extern const nil_time: absolute_time_t;
pub fn is_nil_time(arg_t: absolute_time_t) callconv(.c) bool {
    var t = arg_t;
    _ = &t;
    return !(to_us_since_boot(t) != 0);
}
pub extern fn sleep_until(target: absolute_time_t) void;
pub extern fn sleep_us(us: u64) void;
pub extern fn sleep_ms(ms: u32) void;
pub extern fn best_effort_wfe_or_timeout(timeout_timestamp: absolute_time_t) bool;
pub const alarm_id_t = i32;
pub const alarm_callback_t = ?*const fn (id: alarm_id_t, user_data: ?*anyopaque) callconv(.c) i64;
pub const struct_alarm_pool = opaque {
    pub const num = alarm_pool_timer_alarm_num;
    pub const num1 = alarm_pool_hardware_alarm_num;
    pub const num2 = alarm_pool_core_num;
    pub const destroy = alarm_pool_destroy;
    pub const at = alarm_pool_add_alarm_at;
    pub const context = alarm_pool_add_alarm_at_force_in_context;
    pub const us = alarm_pool_add_alarm_in_us;
    pub const ms = alarm_pool_add_alarm_in_ms;
    pub const us1 = alarm_pool_remaining_alarm_time_us;
    pub const ms1 = alarm_pool_remaining_alarm_time_ms;
    pub const alarm = alarm_pool_cancel_alarm;
    pub const us2 = alarm_pool_add_repeating_timer_us;
    pub const ms2 = alarm_pool_add_repeating_timer_ms;
};
pub const alarm_pool_t = struct_alarm_pool;
pub const alarm_pool_timer_t = anyopaque;
pub extern fn alarm_pool_init_default() void;
pub extern fn runtime_init_default_alarm_pool() void;
pub extern fn alarm_pool_get_default() ?*alarm_pool_t;
pub extern fn alarm_pool_create_on_timer(timer: ?*alarm_pool_timer_t, timer_alarm_num: uint, max_timers: uint) ?*alarm_pool_t;
pub extern fn alarm_pool_timer_for_timer_num(timer_num: uint) ?*alarm_pool_timer_t;
pub extern fn alarm_pool_get_default_timer() ?*alarm_pool_timer_t;
pub fn alarm_pool_create(arg_timer_alarm_num: uint, arg_max_timers: uint) callconv(.c) ?*alarm_pool_t {
    var timer_alarm_num = arg_timer_alarm_num;
    _ = &timer_alarm_num;
    var max_timers = arg_max_timers;
    _ = &max_timers;
    return alarm_pool_create_on_timer(alarm_pool_get_default_timer(), timer_alarm_num, max_timers);
}
pub extern fn alarm_pool_create_on_timer_with_unused_hardware_alarm(timer: ?*alarm_pool_timer_t, max_timers: uint) ?*alarm_pool_t;
pub fn alarm_pool_create_with_unused_hardware_alarm(arg_max_timers: uint) callconv(.c) ?*alarm_pool_t {
    var max_timers = arg_max_timers;
    _ = &max_timers;
    return alarm_pool_create_on_timer_with_unused_hardware_alarm(alarm_pool_get_default_timer(), max_timers);
}
pub extern fn alarm_pool_timer_alarm_num(pool: ?*alarm_pool_t) uint;
pub fn alarm_pool_hardware_alarm_num(arg_pool: ?*alarm_pool_t) callconv(.c) uint {
    var pool = arg_pool;
    _ = &pool;
    return alarm_pool_timer_alarm_num(pool);
}
pub extern fn alarm_pool_core_num(pool: ?*alarm_pool_t) uint;
pub extern fn alarm_pool_destroy(pool: ?*alarm_pool_t) void;
pub extern fn alarm_pool_add_alarm_at(pool: ?*alarm_pool_t, time: absolute_time_t, callback: alarm_callback_t, user_data: ?*anyopaque, fire_if_past: bool) alarm_id_t;
pub extern fn alarm_pool_add_alarm_at_force_in_context(pool: ?*alarm_pool_t, time: absolute_time_t, callback: alarm_callback_t, user_data: ?*anyopaque) alarm_id_t;
pub fn alarm_pool_add_alarm_in_us(arg_pool: ?*alarm_pool_t, arg_us: u64, arg_callback: alarm_callback_t, arg_user_data: ?*anyopaque, arg_fire_if_past: bool) callconv(.c) alarm_id_t {
    var pool = arg_pool;
    _ = &pool;
    var us = arg_us;
    _ = &us;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var fire_if_past = arg_fire_if_past;
    _ = &fire_if_past;
    return alarm_pool_add_alarm_at(pool, delayed_by_us(get_absolute_time(), us), callback, user_data, fire_if_past);
}
pub fn alarm_pool_add_alarm_in_ms(arg_pool: ?*alarm_pool_t, arg_ms: u32, arg_callback: alarm_callback_t, arg_user_data: ?*anyopaque, arg_fire_if_past: bool) callconv(.c) alarm_id_t {
    var pool = arg_pool;
    _ = &pool;
    var ms = arg_ms;
    _ = &ms;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var fire_if_past = arg_fire_if_past;
    _ = &fire_if_past;
    return alarm_pool_add_alarm_at(pool, delayed_by_ms(get_absolute_time(), ms), callback, user_data, fire_if_past);
}
pub extern fn alarm_pool_remaining_alarm_time_us(pool: ?*alarm_pool_t, alarm_id: alarm_id_t) i64;
pub extern fn alarm_pool_remaining_alarm_time_ms(pool: ?*alarm_pool_t, alarm_id: alarm_id_t) i32;
pub extern fn alarm_pool_cancel_alarm(pool: ?*alarm_pool_t, alarm_id: alarm_id_t) bool;
pub fn add_alarm_at(arg_time_1: absolute_time_t, arg_callback: alarm_callback_t, arg_user_data: ?*anyopaque, arg_fire_if_past: bool) callconv(.c) alarm_id_t {
    var time_1 = arg_time_1;
    _ = &time_1;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var fire_if_past = arg_fire_if_past;
    _ = &fire_if_past;
    return alarm_pool_add_alarm_at(alarm_pool_get_default(), time_1, callback, user_data, fire_if_past);
}
pub fn add_alarm_in_us(arg_us: u64, arg_callback: alarm_callback_t, arg_user_data: ?*anyopaque, arg_fire_if_past: bool) callconv(.c) alarm_id_t {
    var us = arg_us;
    _ = &us;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var fire_if_past = arg_fire_if_past;
    _ = &fire_if_past;
    return alarm_pool_add_alarm_in_us(alarm_pool_get_default(), us, callback, user_data, fire_if_past);
}
pub fn add_alarm_in_ms(arg_ms: u32, arg_callback: alarm_callback_t, arg_user_data: ?*anyopaque, arg_fire_if_past: bool) callconv(.c) alarm_id_t {
    var ms = arg_ms;
    _ = &ms;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var fire_if_past = arg_fire_if_past;
    _ = &fire_if_past;
    return alarm_pool_add_alarm_in_ms(alarm_pool_get_default(), ms, callback, user_data, fire_if_past);
}
pub fn cancel_alarm(arg_alarm_id: alarm_id_t) callconv(.c) bool {
    var alarm_id = arg_alarm_id;
    _ = &alarm_id;
    return alarm_pool_cancel_alarm(alarm_pool_get_default(), alarm_id);
}
pub extern fn remaining_alarm_time_us(alarm_id: alarm_id_t) i64;
pub extern fn remaining_alarm_time_ms(alarm_id: alarm_id_t) i32;
pub const repeating_timer_callback_t = ?*const fn (rt: [*c]repeating_timer_t) callconv(.c) bool;
pub const struct_repeating_timer = extern struct {
    delay_us: i64 = 0,
    pool: ?*alarm_pool_t = null,
    alarm_id: alarm_id_t = 0,
    callback: repeating_timer_callback_t = null,
    user_data: ?*anyopaque = null,
    pub const timer = cancel_repeating_timer;
};
pub const repeating_timer_t = struct_repeating_timer;
pub extern fn alarm_pool_add_repeating_timer_us(pool: ?*alarm_pool_t, delay_us: i64, callback: repeating_timer_callback_t, user_data: ?*anyopaque, out: [*c]repeating_timer_t) bool;
pub fn alarm_pool_add_repeating_timer_ms(arg_pool: ?*alarm_pool_t, arg_delay_ms: i32, arg_callback: repeating_timer_callback_t, arg_user_data: ?*anyopaque, arg_out: [*c]repeating_timer_t) callconv(.c) bool {
    var pool = arg_pool;
    _ = &pool;
    var delay_ms = arg_delay_ms;
    _ = &delay_ms;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var out = arg_out;
    _ = &out;
    return alarm_pool_add_repeating_timer_us(pool, @as(i64, delay_ms) * @as(i64, @as(c_int, 1000)), callback, user_data, out);
}
pub fn add_repeating_timer_us(arg_delay_us: i64, arg_callback: repeating_timer_callback_t, arg_user_data: ?*anyopaque, arg_out: [*c]repeating_timer_t) callconv(.c) bool {
    var delay_us = arg_delay_us;
    _ = &delay_us;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var out = arg_out;
    _ = &out;
    return alarm_pool_add_repeating_timer_us(alarm_pool_get_default(), delay_us, callback, user_data, out);
}
pub fn add_repeating_timer_ms(arg_delay_ms: i32, arg_callback: repeating_timer_callback_t, arg_user_data: ?*anyopaque, arg_out: [*c]repeating_timer_t) callconv(.c) bool {
    var delay_ms = arg_delay_ms;
    _ = &delay_ms;
    var callback = arg_callback;
    _ = &callback;
    var user_data = arg_user_data;
    _ = &user_data;
    var out = arg_out;
    _ = &out;
    return alarm_pool_add_repeating_timer_us(alarm_pool_get_default(), @as(i64, delay_ms) * @as(i64, @as(c_int, 1000)), callback, user_data, out);
}
pub extern fn cancel_repeating_timer(timer: [*c]repeating_timer_t) bool;
pub fn cyw43_hal_ticks_us() callconv(.c) u32 {
    return time_us_32();
}
pub fn cyw43_hal_ticks_ms() callconv(.c) u32 {
    return to_ms_since_boot(get_absolute_time());
}
pub fn cyw43_hal_pin_read(arg_pin: uint) callconv(.c) c_int {
    var pin = arg_pin;
    _ = &pin;
    return @intFromBool(gpio_get(pin));
}
pub fn cyw43_hal_pin_low(arg_pin: uint) callconv(.c) void {
    var pin = arg_pin;
    _ = &pin;
    gpio_put(pin, @as(c_int, 0) != 0);
}
pub fn cyw43_hal_pin_high(arg_pin: uint) callconv(.c) void {
    var pin = arg_pin;
    _ = &pin;
    gpio_put(pin, @as(c_int, 1) != 0);
}
pub fn cyw43_hal_pin_config(arg_pin: uint, arg_mode: u32, arg_pull: u32, arg_alt: u32) callconv(.c) void {
    var pin = arg_pin;
    _ = &pin;
    var mode = arg_mode;
    _ = &mode;
    var pull = arg_pull;
    _ = &pull;
    var alt = arg_alt;
    _ = &alt;
    const static_local___func__ = struct {
        const __func__: [20:0]u8 = "cyw43_hal_pin_config".*;
    };
    _ = &static_local___func__;
    if (((mode == @as(u32, @bitCast(@as(c_int, GPIO_IN)))) or (mode == @as(u32, @bitCast(@as(c_int, GPIO_OUT))))) and (alt == @as(u32, 0))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h", 155, @ptrCast(@alignCast(&static_local___func__.__func__)), "(mode == CYW43_HAL_PIN_MODE_INPUT || mode == CYW43_HAL_PIN_MODE_OUTPUT) && alt == 0");
    gpio_set_dir(pin, mode != 0);
    gpio_set_pulls(pin, pull == @as(u32, @bitCast(@as(c_int, @as(c_int, 1)))), pull == @as(u32, @bitCast(@as(c_int, @as(c_int, 2)))));
}
pub extern fn cyw43_hal_get_mac(idx: c_int, buf: [*c]u8) void;
pub extern fn cyw43_hal_generate_laa_mac(idx: c_int, buf: [*c]u8) void;
pub extern fn cyw43_thread_enter() void;
pub extern fn cyw43_thread_exit() void;
pub extern fn cyw43_thread_lock_check() void;
pub extern fn cyw43_await_background_or_timeout_us(timeout_us: u32) void;
pub extern fn cyw43_delay_ms(ms: u32) void;
pub extern fn cyw43_delay_us(us: u32) void;
pub extern fn cyw43_schedule_internal_poll_dispatch(func: ?*const fn () callconv(.c) void) void;
pub extern fn cyw43_post_poll_hook() void;
pub const struct___va_list_tag_3 = extern struct {
    unnamed_0: ?*anyopaque = null,
};
pub const __builtin_va_list = [1]struct___va_list_tag_3;
pub const va_list = __builtin_va_list;
pub const __gnuc_va_list = __builtin_va_list;
pub const __blkcnt_t = c_long;
pub const __blksize_t = c_long;
pub const __fsblkcnt_t = __uint64_t;
pub const __fsfilcnt_t = __uint32_t;
pub const _off_t = c_long;
pub const __pid_t = c_int;
pub const __dev_t = c_short;
pub const __uid_t = c_ushort;
pub const __gid_t = c_ushort;
pub const __id_t = __uint32_t;
pub const __ino_t = c_ushort;
pub const __mode_t = __uint32_t;
pub const _off64_t = c_longlong;
pub const __off_t = _off_t;
pub const __loff_t = _off64_t;
pub const __key_t = c_long;
pub const _fpos_t = c_long;
pub const __size_t = c_uint;
pub const _ssize_t = c_int;
pub const __ssize_t = _ssize_t;
const union_unnamed_4 = extern union {
    __wch: wint_t,
    __wchb: [4]u8,
};
pub const _mbstate_t = extern struct {
    __count: c_int = 0,
    __value: union_unnamed_4 = @import("std").mem.zeroes(union_unnamed_4),
};
pub const _iconv_t = ?*anyopaque;
pub const __clock_t = c_ulong;
pub const __time_t = __int_least64_t;
pub const __clockid_t = c_ulong;
pub const __daddr_t = c_long;
pub const __timer_t = c_ulong;
pub const __sa_family_t = __uint8_t;
pub const __socklen_t = __uint32_t;
pub const __nl_item = c_int;
pub const __nlink_t = c_ushort;
pub const __suseconds_t = c_long;
pub const __useconds_t = c_ulong;
pub const __va_list = __builtin_va_list;
pub const __ULong = c_ulong;
pub const struct___lock = opaque {
    pub const close = __retarget_lock_close;
    pub const recursive = __retarget_lock_close_recursive;
    pub const acquire = __retarget_lock_acquire;
    pub const recursive1 = __retarget_lock_acquire_recursive;
    pub const acquire1 = __retarget_lock_try_acquire;
    pub const recursive2 = __retarget_lock_try_acquire_recursive;
    pub const release = __retarget_lock_release;
    pub const recursive3 = __retarget_lock_release_recursive;
};
pub const _LOCK_T = ?*struct___lock;
pub extern fn __retarget_lock_init(lock: [*c]_LOCK_T) void;
pub extern fn __retarget_lock_init_recursive(lock: [*c]_LOCK_T) void;
pub extern fn __retarget_lock_close(lock: _LOCK_T) void;
pub extern fn __retarget_lock_close_recursive(lock: _LOCK_T) void;
pub extern fn __retarget_lock_acquire(lock: _LOCK_T) void;
pub extern fn __retarget_lock_acquire_recursive(lock: _LOCK_T) void;
pub extern fn __retarget_lock_try_acquire(lock: _LOCK_T) c_int;
pub extern fn __retarget_lock_try_acquire_recursive(lock: _LOCK_T) c_int;
pub extern fn __retarget_lock_release(lock: _LOCK_T) void;
pub extern fn __retarget_lock_release_recursive(lock: _LOCK_T) void;
pub const _flock_t = _LOCK_T;
pub const struct__Bigint = extern struct {
    _next: [*c]struct__Bigint = null,
    _k: c_int = 0,
    _maxwds: c_int = 0,
    _sign: c_int = 0,
    _wds: c_int = 0,
    _x: [1]__ULong = @import("std").mem.zeroes([1]__ULong),
};
pub const struct___tm = extern struct {
    __tm_sec: c_int = 0,
    __tm_min: c_int = 0,
    __tm_hour: c_int = 0,
    __tm_mday: c_int = 0,
    __tm_mon: c_int = 0,
    __tm_year: c_int = 0,
    __tm_wday: c_int = 0,
    __tm_yday: c_int = 0,
    __tm_isdst: c_int = 0,
};
pub const struct__on_exit_args = extern struct {
    _fnargs: [32]?*anyopaque = @import("std").mem.zeroes([32]?*anyopaque),
    _dso_handle: [32]?*anyopaque = @import("std").mem.zeroes([32]?*anyopaque),
    _fntypes: __ULong = 0,
    _is_cxa: __ULong = 0,
};
pub const struct__atexit = extern struct {
    _next: [*c]struct__atexit = null,
    _ind: c_int = 0,
    _fns: [32]?*const fn () callconv(.c) void = @import("std").mem.zeroes([32]?*const fn () callconv(.c) void),
    _on_exit_args: struct__on_exit_args = @import("std").mem.zeroes(struct__on_exit_args),
};
pub const struct___sbuf = extern struct {
    _base: [*c]u8 = null,
    _size: c_int = 0,
};
pub const __FILE = struct___sFILE;
pub const struct___locale_t = opaque {
    pub const l = __locale_ctype_ptr_l;
};
pub const struct__rand48 = extern struct {
    _seed: [3]c_ushort = @import("std").mem.zeroes([3]c_ushort),
    _mult: [3]c_ushort = @import("std").mem.zeroes([3]c_ushort),
    _add: c_ushort = 0,
};
const struct_unnamed_6 = extern struct {
    _strtok_last: [*c]u8 = null,
    _asctime_buf: [26]u8 = @import("std").mem.zeroes([26]u8),
    _localtime_buf: struct___tm = @import("std").mem.zeroes(struct___tm),
    _gamma_signgam: c_int = 0,
    _rand_next: c_ulonglong = 0,
    _r48: struct__rand48 = @import("std").mem.zeroes(struct__rand48),
    _mblen_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _mbtowc_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _wctomb_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _l64a_buf: [8]u8 = @import("std").mem.zeroes([8]u8),
    _signal_buf: [24]u8 = @import("std").mem.zeroes([24]u8),
    _getdate_err: c_int = 0,
    _mbrlen_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _mbrtowc_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _mbsrtowcs_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _wcrtomb_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _wcsrtombs_state: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _h_errno: c_int = 0,
    _getlocalename_l_buf: [32]u8 = @import("std").mem.zeroes([32]u8),
};
const union_unnamed_5 = extern union {
    _reent: struct_unnamed_6,
};
pub const struct__reent = extern struct {
    _errno: c_int = 0,
    _stdin: [*c]__FILE = null,
    _stdout: [*c]__FILE = null,
    _stderr: [*c]__FILE = null,
    _inc: c_int = 0,
    _emergency: [25]u8 = @import("std").mem.zeroes([25]u8),
    _locale: ?*struct___locale_t = null,
    __cleanup: ?*const fn ([*c]struct__reent) callconv(.c) void = null,
    _result: [*c]struct__Bigint = null,
    _result_k: c_int = 0,
    _p5s: [*c]struct__Bigint = null,
    _freelist: [*c][*c]struct__Bigint = null,
    _cvtlen: c_int = 0,
    _cvtbuf: [*c]u8 = null,
    _new: union_unnamed_5 = @import("std").mem.zeroes(union_unnamed_5),
    _sig_func: [*c]?*const fn (c_int) callconv(.c) void = null,
    pub const reent = _reclaim_reent;
    pub const sglue = _fwalk_sglue;
    pub const r = _asiprintf_r;
    pub const r1 = _asniprintf_r;
    pub const r2 = _asnprintf_r;
    pub const r3 = _asprintf_r;
    pub const r4 = _diprintf_r;
    pub const r5 = _dprintf_r;
    pub const r6 = _fclose_r;
    pub const r7 = _fcloseall_r;
    pub const r8 = _fdopen_r;
    pub const r9 = _fflush_r;
    pub const r10 = _fgetc_r;
    pub const r11 = _fgetc_unlocked_r;
    pub const r12 = _fgets_r;
    pub const r13 = _fgets_unlocked_r;
    pub const r14 = _fgetpos_r;
    pub const r15 = _fsetpos_r;
    pub const r16 = _fiprintf_r;
    pub const r17 = _fiscanf_r;
    pub const r18 = _fmemopen_r;
    pub const r19 = _fopen_r;
    pub const r20 = _freopen_r;
    pub const r21 = _fprintf_r;
    pub const r22 = _fpurge_r;
    pub const r23 = _fputc_r;
    pub const r24 = _fputc_unlocked_r;
    pub const r25 = _fputs_r;
    pub const r26 = _fputs_unlocked_r;
    pub const r27 = _fread_r;
    pub const r28 = _fread_unlocked_r;
    pub const r29 = _fscanf_r;
    pub const r30 = _fseek_r;
    pub const r31 = _fseeko_r;
    pub const r32 = _ftell_r;
    pub const r33 = _ftello_r;
    pub const r34 = _rewind_r;
    pub const r35 = _fwrite_r;
    pub const r36 = _fwrite_unlocked_r;
    pub const r37 = _getc_r;
    pub const r38 = _getc_unlocked_r;
    pub const r39 = _getchar_r;
    pub const r40 = _getchar_unlocked_r;
    pub const r41 = _gets_r;
    pub const r42 = _iprintf_r;
    pub const r43 = _iscanf_r;
    pub const r44 = _open_memstream_r;
    pub const r45 = _perror_r;
    pub const r46 = _printf_r;
    pub const r47 = _putc_r;
    pub const r48 = _putc_unlocked_r;
    pub const r49 = _putchar_unlocked_r;
    pub const r50 = _putchar_r;
    pub const r51 = _puts_r;
    pub const r52 = _remove_r;
    pub const r53 = _rename_r;
    pub const r54 = _scanf_r;
    pub const r55 = _siprintf_r;
    pub const r56 = _siscanf_r;
    pub const r57 = _sniprintf_r;
    pub const r58 = _snprintf_r;
    pub const r59 = _sprintf_r;
    pub const r60 = _sscanf_r;
    pub const r61 = _tempnam_r;
    pub const r62 = _tmpfile_r;
    pub const r63 = _tmpnam_r;
    pub const r64 = _ungetc_r;
    pub const r65 = _vasiprintf_r;
    pub const r66 = _vasniprintf_r;
    pub const r67 = _vasnprintf_r;
    pub const r68 = _vasprintf_r;
    pub const r69 = _vdiprintf_r;
    pub const r70 = _vdprintf_r;
    pub const r71 = _vfiprintf_r;
    pub const r72 = _vfiscanf_r;
    pub const r73 = _vfprintf_r;
    pub const r74 = _vfscanf_r;
    pub const r75 = _viprintf_r;
    pub const r76 = _viscanf_r;
    pub const r77 = _vprintf_r;
    pub const r78 = _vscanf_r;
    pub const r79 = _vsiprintf_r;
    pub const r80 = _vsiscanf_r;
    pub const r81 = _vsniprintf_r;
    pub const r82 = _vsnprintf_r;
    pub const r83 = _vsprintf_r;
    pub const r84 = _vsscanf_r;
    pub const r85 = __srget_r;
    pub const r86 = __swbuf_r;
    pub const r87 = _funopen_r;
    pub const r88 = __sputc_r;
    pub const r89 = _tzset_r;
    pub const r90 = _atoi_r;
    pub const r91 = _atol_r;
    pub const r92 = _getenv_r;
    pub const r93 = _findenv_r;
    pub const r94 = _mblen_r;
    pub const r95 = _mbtowc_r;
    pub const r96 = _wctomb_r;
    pub const r97 = _mbstowcs_r;
    pub const r98 = _wcstombs_r;
    pub const r99 = _mkdtemp_r;
    pub const r100 = _mkostemp_r;
    pub const r101 = _mkostemps_r;
    pub const r102 = _mkstemp_r;
    pub const r103 = _mkstemps_r;
    pub const r104 = _mktemp_r;
    pub const r105 = _strtod_r;
    pub const r106 = _strtol_r;
    pub const r107 = _strtoul_r;
    pub const r108 = _l64a_r;
    pub const r109 = _putenv_r;
    pub const r110 = _reallocf_r;
    pub const r111 = _setenv_r;
    pub const r112 = _drand48_r;
    pub const r113 = _erand48_r;
    pub const r114 = _jrand48_r;
    pub const r115 = _lcong48_r;
    pub const r116 = _lrand48_r;
    pub const r117 = _mrand48_r;
    pub const r118 = _nrand48_r;
    pub const r119 = _seed48_r;
    pub const r120 = _srand48_r;
    pub const r121 = _atoll_r;
    pub const r122 = _strtoll_r;
    pub const r123 = _strtoull_r;
    pub const r124 = _unsetenv_r;
    pub const r125 = _dtoa_r;
    pub const r126 = _malloc_r;
    pub const r127 = _calloc_r;
    pub const r128 = _free_r;
    pub const r129 = _realloc_r;
    pub const r130 = _mstats_r;
    pub const r131 = _system_r;
    pub const r132 = _strtold_r;
    pub const r133 = _strtoimax_r;
    pub const r134 = _strtoumax_r;
    pub const r135 = _wcstoimax_r;
    pub const r136 = _wcstoumax_r;
    pub const r137 = _strdup_r;
    pub const r138 = _strndup_r;
    pub const r139 = _strerror_r;
};
pub const struct___sFILE = extern struct {
    _p: [*c]u8 = null,
    _r: c_int = 0,
    _w: c_int = 0,
    _flags: c_short = 0,
    _file: c_short = 0,
    _bf: struct___sbuf = @import("std").mem.zeroes(struct___sbuf),
    _lbfsize: c_int = 0,
    _cookie: ?*anyopaque = null,
    _read: ?*const fn ([*c]struct__reent, ?*anyopaque, [*c]u8, c_int) callconv(.c) c_int = null,
    _write: ?*const fn ([*c]struct__reent, ?*anyopaque, [*c]const u8, c_int) callconv(.c) c_int = null,
    _seek: ?*const fn ([*c]struct__reent, ?*anyopaque, _fpos_t, c_int) callconv(.c) _fpos_t = null,
    _close: ?*const fn ([*c]struct__reent, ?*anyopaque) callconv(.c) c_int = null,
    _ub: struct___sbuf = @import("std").mem.zeroes(struct___sbuf),
    _up: [*c]u8 = null,
    _ur: c_int = 0,
    _ubuf: [3]u8 = @import("std").mem.zeroes([3]u8),
    _nbuf: [1]u8 = @import("std").mem.zeroes([1]u8),
    _lb: struct___sbuf = @import("std").mem.zeroes(struct___sbuf),
    _blksize: c_int = 0,
    _offset: _off_t = 0,
    _data: [*c]struct__reent = null,
    _lock: _flock_t = null,
    _mbstate: _mbstate_t = @import("std").mem.zeroes(_mbstate_t),
    _flags2: c_int = 0,
    pub const unlocked = getc_unlocked;
    pub const unlocked1 = clearerr_unlocked;
    pub const unlocked2 = feof_unlocked;
    pub const unlocked3 = ferror_unlocked;
    pub const unlocked4 = fileno_unlocked;
    pub const unlocked5 = fflush_unlocked;
    pub const unlocked6 = fgetc_unlocked;
};
pub extern var __sf: [3]__FILE;
pub const struct__glue = extern struct {
    _next: [*c]struct__glue = null,
    _niobs: c_int = 0,
    _iobs: [*c]__FILE = null,
};
pub extern var __sglue: struct__glue;
pub extern var _impure_ptr: [*c]struct__reent;
pub extern var _impure_data: struct__reent;
pub extern var __atexit: [*c]struct__atexit;
pub extern var __atexit0: struct__atexit;
pub extern var __stdio_exit_handler: ?*const fn () callconv(.c) void;
pub extern fn _reclaim_reent([*c]struct__reent) void;
pub extern fn _fwalk_sglue([*c]struct__reent, ?*const fn ([*c]struct__reent, [*c]__FILE) callconv(.c) c_int, [*c]struct__glue) c_int;
pub const FILE = __FILE;
pub const fpos_t = _fpos_t;
pub const off_t = __off_t;
pub extern fn ctermid([*c]u8) [*c]u8;
pub extern fn tmpfile() [*c]FILE;
pub extern fn tmpnam([*c]u8) [*c]u8;
pub extern fn tempnam([*c]const u8, [*c]const u8) [*c]u8;
pub extern fn fclose([*c]FILE) c_int;
pub extern fn fflush([*c]FILE) c_int;
pub extern fn freopen(noalias [*c]const u8, noalias [*c]const u8, noalias [*c]FILE) [*c]FILE;
pub extern fn setbuf(noalias [*c]FILE, noalias [*c]u8) void;
pub extern fn setvbuf(noalias [*c]FILE, noalias [*c]u8, c_int, usize) c_int;
pub extern fn fprintf(noalias [*c]FILE, noalias [*c]const u8, ...) c_int;
pub extern fn fscanf(noalias [*c]FILE, noalias [*c]const u8, ...) c_int;
pub extern fn printf(noalias [*c]const u8, ...) c_int;
pub extern fn scanf(noalias [*c]const u8, ...) c_int;
pub extern fn sscanf(noalias [*c]const u8, noalias [*c]const u8, ...) c_int;
pub extern fn vfprintf(noalias [*c]FILE, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vprintf([*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vsprintf(noalias [*c]u8, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn fgetc([*c]FILE) c_int;
pub extern fn fgets(noalias [*c]u8, c_int, noalias [*c]FILE) [*c]u8;
pub extern fn fputc(c_int, [*c]FILE) c_int;
pub extern fn fputs(noalias [*c]const u8, noalias [*c]FILE) c_int;
pub extern fn getc([*c]FILE) c_int;
pub extern fn getchar() c_int;
pub extern fn gets([*c]u8) [*c]u8;
pub extern fn putc(c_int, [*c]FILE) c_int;
pub extern fn putchar(c_int) c_int;
pub extern fn puts([*c]const u8) c_int;
pub extern fn ungetc(c_int, [*c]FILE) c_int;
pub extern fn fread(noalias ?*anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn fwrite(noalias ?*const anyopaque, _size: usize, _n: usize, [*c]FILE) usize;
pub extern fn fgetpos(noalias [*c]FILE, noalias [*c]fpos_t) c_int;
pub extern fn fseek([*c]FILE, c_long, c_int) c_int;
pub extern fn fsetpos([*c]FILE, [*c]const fpos_t) c_int;
pub extern fn ftell([*c]FILE) c_long;
pub extern fn rewind([*c]FILE) void;
pub extern fn clearerr([*c]FILE) void;
pub extern fn feof([*c]FILE) c_int;
pub extern fn ferror([*c]FILE) c_int;
pub extern fn perror([*c]const u8) void;
pub extern fn fopen(noalias _name: [*c]const u8, noalias _type: [*c]const u8) [*c]FILE;
pub extern fn sprintf(noalias [*c]u8, noalias [*c]const u8, ...) c_int;
pub extern fn remove([*c]const u8) c_int;
pub extern fn rename([*c]const u8, [*c]const u8) c_int;
pub extern fn fseeko([*c]FILE, off_t, c_int) c_int;
pub extern fn ftello([*c]FILE) off_t;
pub extern fn snprintf(noalias [*c]u8, usize, noalias [*c]const u8, ...) c_int;
pub extern fn vsnprintf(noalias [*c]u8, usize, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vfscanf(noalias [*c]FILE, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vscanf([*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vsscanf(noalias [*c]const u8, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn asiprintf([*c][*c]u8, [*c]const u8, ...) c_int;
pub extern fn asniprintf([*c]u8, [*c]usize, [*c]const u8, ...) [*c]u8;
pub extern fn asnprintf(noalias [*c]u8, noalias [*c]usize, noalias [*c]const u8, ...) [*c]u8;
pub extern fn diprintf(c_int, [*c]const u8, ...) c_int;
pub extern fn fiprintf([*c]FILE, [*c]const u8, ...) c_int;
pub extern fn fiscanf([*c]FILE, [*c]const u8, ...) c_int;
pub extern fn iprintf([*c]const u8, ...) c_int;
pub extern fn iscanf([*c]const u8, ...) c_int;
pub extern fn siprintf([*c]u8, [*c]const u8, ...) c_int;
pub extern fn siscanf([*c]const u8, [*c]const u8, ...) c_int;
pub extern fn sniprintf([*c]u8, usize, [*c]const u8, ...) c_int;
pub extern fn vasiprintf([*c][*c]u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vasniprintf([*c]u8, [*c]usize, [*c]const u8, [*c]struct___va_list_tag_3) [*c]u8;
pub extern fn vasnprintf([*c]u8, [*c]usize, [*c]const u8, [*c]struct___va_list_tag_3) [*c]u8;
pub extern fn vdiprintf(c_int, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vfiprintf([*c]FILE, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vfiscanf([*c]FILE, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn viprintf([*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn viscanf([*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vsiprintf([*c]u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vsiscanf([*c]const u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn vsniprintf([*c]u8, usize, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn fdopen(c_int, [*c]const u8) [*c]FILE;
pub extern fn fileno([*c]FILE) c_int;
pub extern fn pclose([*c]FILE) c_int;
pub extern fn popen([*c]const u8, [*c]const u8) [*c]FILE;
pub extern fn setbuffer([*c]FILE, [*c]u8, c_int) void;
pub extern fn setlinebuf([*c]FILE) c_int;
pub extern fn getw([*c]FILE) c_int;
pub extern fn putw(c_int, [*c]FILE) c_int;
pub extern fn getc_unlocked([*c]FILE) c_int;
pub extern fn getchar_unlocked() c_int;
pub extern fn flockfile([*c]FILE) void;
pub extern fn ftrylockfile([*c]FILE) c_int;
pub extern fn funlockfile([*c]FILE) void;
pub extern fn putc_unlocked(c_int, [*c]FILE) c_int;
pub extern fn putchar_unlocked(c_int) c_int;
pub extern fn dprintf(c_int, noalias [*c]const u8, ...) c_int;
pub extern fn fmemopen(noalias ?*anyopaque, usize, noalias [*c]const u8) [*c]FILE;
pub extern fn open_memstream([*c][*c]u8, [*c]usize) [*c]FILE;
pub extern fn vdprintf(c_int, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn renameat(c_int, [*c]const u8, c_int, [*c]const u8) c_int;
pub extern fn _asiprintf_r([*c]struct__reent, [*c][*c]u8, [*c]const u8, ...) c_int;
pub extern fn _asniprintf_r([*c]struct__reent, [*c]u8, [*c]usize, [*c]const u8, ...) [*c]u8;
pub extern fn _asnprintf_r([*c]struct__reent, noalias [*c]u8, noalias [*c]usize, noalias [*c]const u8, ...) [*c]u8;
pub extern fn _asprintf_r([*c]struct__reent, noalias [*c][*c]u8, noalias [*c]const u8, ...) c_int;
pub extern fn _diprintf_r([*c]struct__reent, c_int, [*c]const u8, ...) c_int;
pub extern fn _dprintf_r([*c]struct__reent, c_int, noalias [*c]const u8, ...) c_int;
pub extern fn _fclose_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _fcloseall_r([*c]struct__reent) c_int;
pub extern fn _fdopen_r([*c]struct__reent, c_int, [*c]const u8) [*c]FILE;
pub extern fn _fflush_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _fgetc_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _fgetc_unlocked_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _fgets_r([*c]struct__reent, noalias [*c]u8, c_int, noalias [*c]FILE) [*c]u8;
pub extern fn _fgets_unlocked_r([*c]struct__reent, noalias [*c]u8, c_int, noalias [*c]FILE) [*c]u8;
pub extern fn _fgetpos_r([*c]struct__reent, [*c]FILE, [*c]fpos_t) c_int;
pub extern fn _fsetpos_r([*c]struct__reent, [*c]FILE, [*c]const fpos_t) c_int;
pub extern fn _fiprintf_r([*c]struct__reent, [*c]FILE, [*c]const u8, ...) c_int;
pub extern fn _fiscanf_r([*c]struct__reent, [*c]FILE, [*c]const u8, ...) c_int;
pub extern fn _fmemopen_r([*c]struct__reent, noalias ?*anyopaque, usize, noalias [*c]const u8) [*c]FILE;
pub extern fn _fopen_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]const u8) [*c]FILE;
pub extern fn _freopen_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]const u8, noalias [*c]FILE) [*c]FILE;
pub extern fn _fprintf_r([*c]struct__reent, noalias [*c]FILE, noalias [*c]const u8, ...) c_int;
pub extern fn _fpurge_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _fputc_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn _fputc_unlocked_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn _fputs_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]FILE) c_int;
pub extern fn _fputs_unlocked_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]FILE) c_int;
pub extern fn _fread_r([*c]struct__reent, noalias ?*anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn _fread_unlocked_r([*c]struct__reent, noalias ?*anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn _fscanf_r([*c]struct__reent, noalias [*c]FILE, noalias [*c]const u8, ...) c_int;
pub extern fn _fseek_r([*c]struct__reent, [*c]FILE, c_long, c_int) c_int;
pub extern fn _fseeko_r([*c]struct__reent, [*c]FILE, _off_t, c_int) c_int;
pub extern fn _ftell_r([*c]struct__reent, [*c]FILE) c_long;
pub extern fn _ftello_r([*c]struct__reent, [*c]FILE) _off_t;
pub extern fn _rewind_r([*c]struct__reent, [*c]FILE) void;
pub extern fn _fwrite_r([*c]struct__reent, noalias ?*const anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn _fwrite_unlocked_r([*c]struct__reent, noalias ?*const anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn _getc_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _getc_unlocked_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn _getchar_r([*c]struct__reent) c_int;
pub extern fn _getchar_unlocked_r([*c]struct__reent) c_int;
pub extern fn _gets_r([*c]struct__reent, [*c]u8) [*c]u8;
pub extern fn _iprintf_r([*c]struct__reent, [*c]const u8, ...) c_int;
pub extern fn _iscanf_r([*c]struct__reent, [*c]const u8, ...) c_int;
pub extern fn _open_memstream_r([*c]struct__reent, [*c][*c]u8, [*c]usize) [*c]FILE;
pub extern fn _perror_r([*c]struct__reent, [*c]const u8) void;
pub extern fn _printf_r([*c]struct__reent, noalias [*c]const u8, ...) c_int;
pub extern fn _putc_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn _putc_unlocked_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn _putchar_unlocked_r([*c]struct__reent, c_int) c_int;
pub extern fn _putchar_r([*c]struct__reent, c_int) c_int;
pub extern fn _puts_r([*c]struct__reent, [*c]const u8) c_int;
pub extern fn _remove_r([*c]struct__reent, [*c]const u8) c_int;
pub extern fn _rename_r([*c]struct__reent, _old: [*c]const u8, _new: [*c]const u8) c_int;
pub extern fn _scanf_r([*c]struct__reent, noalias [*c]const u8, ...) c_int;
pub extern fn _siprintf_r([*c]struct__reent, [*c]u8, [*c]const u8, ...) c_int;
pub extern fn _siscanf_r([*c]struct__reent, [*c]const u8, [*c]const u8, ...) c_int;
pub extern fn _sniprintf_r([*c]struct__reent, [*c]u8, usize, [*c]const u8, ...) c_int;
pub extern fn _snprintf_r([*c]struct__reent, noalias [*c]u8, usize, noalias [*c]const u8, ...) c_int;
pub extern fn _sprintf_r([*c]struct__reent, noalias [*c]u8, noalias [*c]const u8, ...) c_int;
pub extern fn _sscanf_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]const u8, ...) c_int;
pub extern fn _tempnam_r([*c]struct__reent, [*c]const u8, [*c]const u8) [*c]u8;
pub extern fn _tmpfile_r([*c]struct__reent) [*c]FILE;
pub extern fn _tmpnam_r([*c]struct__reent, [*c]u8) [*c]u8;
pub extern fn _ungetc_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn _vasiprintf_r([*c]struct__reent, [*c][*c]u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vasniprintf_r([*c]struct__reent, [*c]u8, [*c]usize, [*c]const u8, [*c]struct___va_list_tag_3) [*c]u8;
pub extern fn _vasnprintf_r([*c]struct__reent, [*c]u8, [*c]usize, [*c]const u8, [*c]struct___va_list_tag_3) [*c]u8;
pub extern fn _vasprintf_r([*c]struct__reent, [*c][*c]u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vdiprintf_r([*c]struct__reent, c_int, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vdprintf_r([*c]struct__reent, c_int, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vfiprintf_r([*c]struct__reent, [*c]FILE, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vfiscanf_r([*c]struct__reent, [*c]FILE, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vfprintf_r([*c]struct__reent, noalias [*c]FILE, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vfscanf_r([*c]struct__reent, noalias [*c]FILE, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _viprintf_r([*c]struct__reent, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _viscanf_r([*c]struct__reent, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vprintf_r([*c]struct__reent, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vscanf_r([*c]struct__reent, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsiprintf_r([*c]struct__reent, [*c]u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsiscanf_r([*c]struct__reent, [*c]const u8, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsniprintf_r([*c]struct__reent, [*c]u8, usize, [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsnprintf_r([*c]struct__reent, noalias [*c]u8, usize, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsprintf_r([*c]struct__reent, noalias [*c]u8, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn _vsscanf_r([*c]struct__reent, noalias [*c]const u8, noalias [*c]const u8, [*c]struct___va_list_tag_3) c_int;
pub extern fn fpurge([*c]FILE) c_int;
pub extern fn __getdelim([*c][*c]u8, [*c]usize, c_int, [*c]FILE) isize;
pub extern fn __getline([*c][*c]u8, [*c]usize, [*c]FILE) isize;
pub extern fn clearerr_unlocked([*c]FILE) void;
pub extern fn feof_unlocked([*c]FILE) c_int;
pub extern fn ferror_unlocked([*c]FILE) c_int;
pub extern fn fileno_unlocked([*c]FILE) c_int;
pub extern fn fflush_unlocked([*c]FILE) c_int;
pub extern fn fgetc_unlocked([*c]FILE) c_int;
pub extern fn fputc_unlocked(c_int, [*c]FILE) c_int;
pub extern fn fread_unlocked(noalias ?*anyopaque, _size: usize, _n: usize, noalias [*c]FILE) usize;
pub extern fn fwrite_unlocked(noalias ?*const anyopaque, _size: usize, _n: usize, [*c]FILE) usize;
pub extern fn __srget_r([*c]struct__reent, [*c]FILE) c_int;
pub extern fn __swbuf_r([*c]struct__reent, c_int, [*c]FILE) c_int;
pub extern fn funopen(__cookie: ?*const anyopaque, __readfn: ?*const fn (__cookie: ?*anyopaque, __buf: [*c]u8, __n: c_int) callconv(.c) c_int, __writefn: ?*const fn (__cookie: ?*anyopaque, __buf: [*c]const u8, __n: c_int) callconv(.c) c_int, __seekfn: ?*const fn (__cookie: ?*anyopaque, __off: fpos_t, __whence: c_int) callconv(.c) fpos_t, __closefn: ?*const fn (__cookie: ?*anyopaque) callconv(.c) c_int) [*c]FILE;
pub extern fn _funopen_r([*c]struct__reent, __cookie: ?*const anyopaque, __readfn: ?*const fn (__cookie: ?*anyopaque, __buf: [*c]u8, __n: c_int) callconv(.c) c_int, __writefn: ?*const fn (__cookie: ?*anyopaque, __buf: [*c]const u8, __n: c_int) callconv(.c) c_int, __seekfn: ?*const fn (__cookie: ?*anyopaque, __off: fpos_t, __whence: c_int) callconv(.c) fpos_t, __closefn: ?*const fn (__cookie: ?*anyopaque) callconv(.c) c_int) [*c]FILE;
pub inline fn __sputc_r(arg__ptr: [*c]struct__reent, arg__c: c_int, arg__p: [*c]FILE) c_int {
    var _ptr = arg__ptr;
    _ = &_ptr;
    var _c = arg__c;
    _ = &_c;
    var _p = arg__p;
    _ = &_p;
    if (((blk: {
        const ref = &_p.*._w;
        ref.* -= 1;
        break :blk ref.*;
    }) >= @as(c_int, 0)) or ((_p.*._w >= _p.*._lbfsize) and (@as(c_int, @as(u8, @bitCast(@as(i8, @truncate(_c))))) != @as(c_int, '\n')))) return blk: {
        const tmp = @as(u8, @bitCast(@as(i8, @truncate(_c))));
        (blk_1: {
            const ref = &_p.*._p;
            const tmp_2 = ref.*;
            ref.* += 1;
            break :blk_1 tmp_2;
        }).* = tmp;
        break :blk tmp;
    } else return __swbuf_r(_ptr, _c, _p);
}
pub fn _getchar_unlocked() callconv(.c) c_int {
    var _ptr: [*c]struct__reent = undefined;
    _ = &_ptr;
    _ptr = _impure_ptr;
    return if ((blk: {
        const ref = &_ptr.*._stdin.*._r;
        ref.* -= 1;
        break :blk ref.*;
    }) < @as(c_int, 0)) __srget_r(_ptr, _ptr.*._stdin) else @as(c_int, (blk: {
        const ref = &_ptr.*._stdin.*._p;
        const tmp = ref.*;
        ref.* += 1;
        break :blk tmp;
    }).*);
}
pub fn _putchar_unlocked(arg__c: c_int) callconv(.c) c_int {
    var _c = arg__c;
    _ = &_c;
    var _ptr: [*c]struct__reent = undefined;
    _ = &_ptr;
    _ptr = _impure_ptr;
    return __sputc_r(_ptr, _c, _ptr.*._stdout);
}
pub const suseconds_t = __suseconds_t;
pub const time_t = __int_least64_t;
pub const struct_timeval = extern struct {
    tv_sec: time_t = 0,
    tv_usec: suseconds_t = 0,
};
pub const u_int8_t = __uint8_t;
pub const u_int16_t = __uint16_t;
pub const u_int32_t = __uint32_t;
pub const u_int64_t = __uint64_t;
pub const register_t = __intptr_t;
pub const __sigset_t = c_ulong;
pub const struct_timespec = extern struct {
    tv_sec: time_t = 0,
    tv_nsec: c_long = 0,
};
pub const struct_itimerspec = extern struct {
    it_interval: struct_timespec = @import("std").mem.zeroes(struct_timespec),
    it_value: struct_timespec = @import("std").mem.zeroes(struct_timespec),
};
pub const sigset_t = __sigset_t;
pub const __fd_mask = c_ulong;
pub const fd_mask = __fd_mask;
pub const struct_fd_set = extern struct {
    __fds_bits: [2]__fd_mask = @import("std").mem.zeroes([2]__fd_mask),
};
pub const fd_set = struct_fd_set;
pub extern fn select(__n: c_int, __readfds: [*c]fd_set, __writefds: [*c]fd_set, __exceptfds: [*c]fd_set, __timeout: [*c]struct_timeval) c_int;
pub extern fn pselect(__n: c_int, __readfds: [*c]fd_set, __writefds: [*c]fd_set, __exceptfds: [*c]fd_set, __timeout: [*c]const struct_timespec, __set: [*c]const sigset_t) c_int;
pub const in_addr_t = __uint32_t;
pub const in_port_t = __uint16_t;
pub const u_register_t = __uintptr_t;
pub const u_char = u8;
pub const u_short = c_ushort;
pub const u_int = c_uint;
pub const u_long = c_ulong;
pub const ushort = c_ushort;
pub const ulong = c_ulong;
pub const blkcnt_t = __blkcnt_t;
pub const blksize_t = __blksize_t;
pub const clock_t = c_ulong;
pub const daddr_t = __daddr_t;
pub const caddr_t = [*c]u8;
pub const fsblkcnt_t = __fsblkcnt_t;
pub const fsfilcnt_t = __fsfilcnt_t;
pub const id_t = __id_t;
pub const ino_t = __ino_t;
pub const dev_t = __dev_t;
pub const uid_t = __uid_t;
pub const gid_t = __gid_t;
pub const pid_t = __pid_t;
pub const key_t = __key_t;
pub const mode_t = __mode_t;
pub const nlink_t = __nlink_t;
pub const clockid_t = __clockid_t;
pub const timer_t = __timer_t;
pub const useconds_t = __useconds_t;
pub const sbintime_t = __int64_t;
pub const struct_sched_param = extern struct {
    sched_priority: c_int = 0,
};
pub const pthread_t = __uint32_t;
pub const pthread_attr_t = extern struct {
    is_initialized: c_int = 0,
    stackaddr: ?*anyopaque = null,
    stacksize: c_int = 0,
    contentionscope: c_int = 0,
    inheritsched: c_int = 0,
    schedpolicy: c_int = 0,
    schedparam: struct_sched_param = @import("std").mem.zeroes(struct_sched_param),
    detachstate: c_int = 0,
};
pub const pthread_mutex_t = __uint32_t;
pub const pthread_mutexattr_t = extern struct {
    is_initialized: c_int = 0,
    recursive: c_int = 0,
};
pub const pthread_cond_t = __uint32_t;
pub const pthread_condattr_t = extern struct {
    is_initialized: c_int = 0,
    clock: clock_t = 0,
};
pub const pthread_key_t = __uint32_t;
pub const pthread_once_t = extern struct {
    is_initialized: c_int = 0,
    init_executed: c_int = 0,
};
pub const struct_timezone = extern struct {
    tz_minuteswest: c_int = 0,
    tz_dsttime: c_int = 0,
};
pub const struct_bintime = extern struct {
    sec: time_t = 0,
    frac: u64 = 0,
    pub const addx = bintime_addx;
    pub const add = bintime_add;
    pub const sub = bintime_sub;
    pub const mul = bintime_mul;
    pub const shift = bintime_shift;
};
pub fn bintime_addx(arg__bt: [*c]struct_bintime, arg__x: u64) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _x = arg__x;
    _ = &_x;
    var _u_1: u64 = undefined;
    _ = &_u_1;
    _u_1 = _bt.*.frac;
    _bt.*.frac +%= _x;
    if (_u_1 > _bt.*.frac) {
        _bt.*.sec += 1;
    }
}
pub fn bintime_add(arg__bt: [*c]struct_bintime, arg__bt2: [*c]const struct_bintime) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _bt2 = arg__bt2;
    _ = &_bt2;
    var _u_1: u64 = undefined;
    _ = &_u_1;
    _u_1 = _bt.*.frac;
    _bt.*.frac +%= _bt2.*.frac;
    if (_u_1 > _bt.*.frac) {
        _bt.*.sec += 1;
    }
    _bt.*.sec += _bt2.*.sec;
}
pub fn bintime_sub(arg__bt: [*c]struct_bintime, arg__bt2: [*c]const struct_bintime) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _bt2 = arg__bt2;
    _ = &_bt2;
    var _u_1: u64 = undefined;
    _ = &_u_1;
    _u_1 = _bt.*.frac;
    _bt.*.frac -%= _bt2.*.frac;
    if (_u_1 < _bt.*.frac) {
        _bt.*.sec -= 1;
    }
    _bt.*.sec -= _bt2.*.sec;
}
pub fn bintime_mul(arg__bt: [*c]struct_bintime, arg__x: u_int) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _x = arg__x;
    _ = &_x;
    var _p1: u64 = undefined;
    _ = &_p1;
    var _p2: u64 = undefined;
    _ = &_p2;
    _p1 = (_bt.*.frac & @as(c_ulonglong, 4294967295)) *% @as(u64, _x);
    _p2 = ((_bt.*.frac >> @intCast(32)) *% @as(u64, _x)) +% (_p1 >> @intCast(32));
    _bt.*.sec *= _x;
    {
        const ref = &_bt.*.sec;
        ref.* = @bitCast(@as(c_ulonglong, @truncate(@as(u64, @bitCast(@as(c_longlong, ref.*))) +% (_p2 >> @intCast(32)))));
    }
    _bt.*.frac = (_p2 << @intCast(32)) | (_p1 & @as(c_ulonglong, 4294967295));
}
pub fn sbintime_getsec(arg__sbt: sbintime_t) callconv(.c) c_int {
    var _sbt = arg__sbt;
    _ = &_sbt;
    return @truncate(_sbt >> @intCast(32));
}
pub fn bttosbt(_bt: struct_bintime) callconv(.c) sbintime_t {
    _ = &_bt;
    return @bitCast(@as(c_ulonglong, @truncate(@as(u64, @bitCast(@as(c_longlong, _bt.sec << @intCast(32)))) +% (_bt.frac >> @intCast(32)))));
}
pub fn sbttobt(arg__sbt: sbintime_t) callconv(.c) struct_bintime {
    var _sbt = arg__sbt;
    _ = &_sbt;
    var _bt: struct_bintime = undefined;
    _ = &_bt;
    _bt.sec = _sbt >> @intCast(32);
    _bt.frac = @bitCast(@as(c_longlong, _sbt << @intCast(32)));
    return _bt;
}
pub fn sbttons(arg__sbt: sbintime_t) callconv(.c) i64 {
    var _sbt = arg__sbt;
    _ = &_sbt;
    var ns: u64 = undefined;
    _ = &ns;
    ns = @bitCast(@as(c_longlong, _sbt));
    if (ns >= @as(u64, @bitCast(@as(c_longlong, @as(sbintime_t, @as(c_int, 1)) << @intCast(32))))) {
        ns = (ns >> @intCast(32)) *% @as(u64, 1000000000);
    } else {
        ns = 0;
    }
    return @bitCast(@as(c_ulonglong, @truncate(ns +% @as(u64, @bitCast(@as(c_longlong, (@as(sbintime_t, 1000000000) * (_sbt & @as(sbintime_t, 4294967295))) >> @intCast(32)))))));
}
pub fn nstosbt(arg__ns: i64) callconv(.c) sbintime_t {
    var _ns = arg__ns;
    _ = &_ns;
    var sb: sbintime_t = 0;
    _ = &sb;
    if (_ns >= (@as(sbintime_t, @as(c_int, 1)) << @intCast(32))) {
        sb = @divTrunc(_ns, @as(i64, 1000000000)) * (@as(sbintime_t, @as(c_int, 1)) << @intCast(32));
        _ns = __helpers.signedRemainder(_ns, @as(i64, 1000000000));
    }
    {
        const ref = &sb;
        ref.* = @bitCast(@as(c_ulonglong, @truncate(@as(c_ulonglong, @bitCast(@as(c_longlong, ref.*))) +% (((@as(c_ulonglong, @bitCast(@as(c_longlong, _ns))) *% @as(c_ulonglong, 9223372037)) +% @as(c_ulonglong, 2147483647)) >> @intCast(31)))));
    }
    return sb;
}
pub fn sbttous(arg__sbt: sbintime_t) callconv(.c) i64 {
    var _sbt = arg__sbt;
    _ = &_sbt;
    return (@as(sbintime_t, 1000000) * _sbt) >> @intCast(32);
}
pub fn ustosbt(arg__us: i64) callconv(.c) sbintime_t {
    var _us = arg__us;
    _ = &_us;
    var sb: sbintime_t = 0;
    _ = &sb;
    if (_us >= (@as(sbintime_t, @as(c_int, 1)) << @intCast(32))) {
        sb = @divTrunc(_us, @as(i64, 1000000)) * (@as(sbintime_t, @as(c_int, 1)) << @intCast(32));
        _us = __helpers.signedRemainder(_us, @as(i64, 1000000));
    }
    {
        const ref = &sb;
        ref.* = @bitCast(@as(c_ulonglong, @truncate(@as(c_ulonglong, @bitCast(@as(c_longlong, ref.*))) +% (((@as(c_ulonglong, @bitCast(@as(c_longlong, _us))) *% @as(c_ulonglong, 9223372036855)) +% @as(c_ulonglong, 2147483647)) >> @intCast(31)))));
    }
    return sb;
}
pub fn sbttoms(arg__sbt: sbintime_t) callconv(.c) i64 {
    var _sbt = arg__sbt;
    _ = &_sbt;
    return (@as(sbintime_t, 1000) * _sbt) >> @intCast(32);
}
pub fn mstosbt(arg__ms: i64) callconv(.c) sbintime_t {
    var _ms = arg__ms;
    _ = &_ms;
    var sb: sbintime_t = 0;
    _ = &sb;
    if (_ms >= (@as(sbintime_t, @as(c_int, 1)) << @intCast(32))) {
        sb = @divTrunc(_ms, @as(i64, 1000)) * (@as(sbintime_t, @as(c_int, 1)) << @intCast(32));
        _ms = __helpers.signedRemainder(_ms, @as(i64, 1000));
    }
    {
        const ref = &sb;
        ref.* = @bitCast(@as(c_ulonglong, @truncate(@as(c_ulonglong, @bitCast(@as(c_longlong, ref.*))) +% (((@as(c_ulonglong, @bitCast(@as(c_longlong, _ms))) *% @as(c_ulonglong, 9223372036854776)) +% @as(c_ulonglong, 2147483647)) >> @intCast(31)))));
    }
    return sb;
}
pub fn bintime2timespec(arg__bt: [*c]const struct_bintime, arg__ts: [*c]struct_timespec) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _ts = arg__ts;
    _ = &_ts;
    _ts.*.tv_sec = _bt.*.sec;
    _ts.*.tv_nsec = @bitCast(@as(c_ulong, @truncate((@as(u64, @bitCast(@as(c_longlong, @as(c_int, 1000000000)))) *% @as(u64, @as(u32, @truncate(_bt.*.frac >> @intCast(32))))) >> @intCast(32))));
}
pub fn timespec2bintime(arg__ts: [*c]const struct_timespec, arg__bt: [*c]struct_bintime) callconv(.c) void {
    var _ts = arg__ts;
    _ = &_ts;
    var _bt = arg__bt;
    _ = &_bt;
    _bt.*.sec = _ts.*.tv_sec;
    _bt.*.frac = @as(u64, @bitCast(@as(c_longlong, _ts.*.tv_nsec))) *% @as(u64, @bitCast(@as(c_longlong, @as(c_longlong, 18446744073))));
}
pub fn bintime2timeval(arg__bt: [*c]const struct_bintime, arg__tv: [*c]struct_timeval) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _tv = arg__tv;
    _ = &_tv;
    _tv.*.tv_sec = _bt.*.sec;
    _tv.*.tv_usec = @bitCast(@as(c_ulong, @truncate((@as(u64, @bitCast(@as(c_longlong, @as(c_int, 1000000)))) *% @as(u64, @as(u32, @truncate(_bt.*.frac >> @intCast(32))))) >> @intCast(32))));
}
pub fn timeval2bintime(arg__tv: [*c]const struct_timeval, arg__bt: [*c]struct_bintime) callconv(.c) void {
    var _tv = arg__tv;
    _ = &_tv;
    var _bt = arg__bt;
    _ = &_bt;
    _bt.*.sec = _tv.*.tv_sec;
    _bt.*.frac = @as(u64, @bitCast(@as(c_longlong, _tv.*.tv_usec))) *% @as(u64, @bitCast(@as(c_longlong, @as(c_longlong, 18446744073709))));
}
pub fn sbttots(arg__sbt: sbintime_t) callconv(.c) struct_timespec {
    var _sbt = arg__sbt;
    _ = &_sbt;
    var _ts: struct_timespec = undefined;
    _ = &_ts;
    _ts.tv_sec = _sbt >> @intCast(32);
    _ts.tv_nsec = @truncate(sbttons(@as(u32, @bitCast(@as(c_int, @truncate(_sbt))))));
    return _ts;
}
pub fn tstosbt(arg__ts: struct_timespec) callconv(.c) sbintime_t {
    var _ts = arg__ts;
    _ = &_ts;
    return (_ts.tv_sec << @intCast(32)) + nstosbt(_ts.tv_nsec);
}
pub fn sbttotv(arg__sbt: sbintime_t) callconv(.c) struct_timeval {
    var _sbt = arg__sbt;
    _ = &_sbt;
    var _tv: struct_timeval = undefined;
    _ = &_tv;
    _tv.tv_sec = _sbt >> @intCast(32);
    _tv.tv_usec = @truncate(sbttous(@as(u32, @bitCast(@as(c_int, @truncate(_sbt))))));
    return _tv;
}
pub fn tvtosbt(arg__tv: struct_timeval) callconv(.c) sbintime_t {
    var _tv = arg__tv;
    _ = &_tv;
    return (_tv.tv_sec << @intCast(32)) + ustosbt(_tv.tv_usec);
}
pub const struct_itimerval = extern struct {
    it_interval: struct_timeval = @import("std").mem.zeroes(struct_timeval),
    it_value: struct_timeval = @import("std").mem.zeroes(struct_timeval),
};
pub const locale_t = ?*struct___locale_t;
pub const struct_tm = extern struct {
    tm_sec: c_int = 0,
    tm_min: c_int = 0,
    tm_hour: c_int = 0,
    tm_mday: c_int = 0,
    tm_mon: c_int = 0,
    tm_year: c_int = 0,
    tm_wday: c_int = 0,
    tm_yday: c_int = 0,
    tm_isdst: c_int = 0,
};
pub extern fn clock() clock_t;
pub extern fn difftime(_time2: time_t, _time1: time_t) f64;
pub extern fn mktime(_timeptr: [*c]struct_tm) time_t;
pub extern fn time(_timer: [*c]time_t) time_t;
pub extern fn asctime(_tblock: [*c]const struct_tm) [*c]u8;
pub extern fn ctime(_time: [*c]const time_t) [*c]u8;
pub extern fn gmtime(_timer: [*c]const time_t) [*c]struct_tm;
pub extern fn localtime(_timer: [*c]const time_t) [*c]struct_tm;
pub extern fn strftime(noalias _s: [*c]u8, _maxsize: usize, noalias _fmt: [*c]const u8, noalias _t: [*c]const struct_tm) usize;
pub extern fn strftime_l(noalias _s: [*c]u8, _maxsize: usize, noalias _fmt: [*c]const u8, noalias _t: [*c]const struct_tm, _l: locale_t) usize;
pub extern fn asctime_r(noalias [*c]const struct_tm, noalias [*c]u8) [*c]u8;
pub extern fn ctime_r([*c]const time_t, [*c]u8) [*c]u8;
pub extern fn gmtime_r(noalias [*c]const time_t, noalias [*c]struct_tm) [*c]struct_tm;
pub extern fn localtime_r(noalias [*c]const time_t, noalias [*c]struct_tm) [*c]struct_tm;
pub extern fn tzset() void;
pub extern fn _tzset_r([*c]struct__reent) void;
pub extern var _timezone: c_long;
pub extern var _daylight: c_int;
pub extern var _tzname: [2][*c]u8;
pub extern fn utimes([*c]const u8, [*c]const struct_timeval) c_int;
pub extern fn adjtime([*c]const struct_timeval, [*c]struct_timeval) c_int;
pub extern fn futimes(c_int, [*c]const struct_timeval) c_int;
pub extern fn lutimes([*c]const u8, [*c]const struct_timeval) c_int;
pub extern fn settimeofday([*c]const struct_timeval, [*c]const struct_timezone) c_int;
pub extern fn getitimer(__which: c_int, __value: [*c]struct_itimerval) c_int;
pub extern fn setitimer(__which: c_int, noalias __value: [*c]const struct_itimerval, noalias __ovalue: [*c]struct_itimerval) c_int;
pub extern fn gettimeofday(noalias __p: [*c]struct_timeval, noalias __tz: ?*anyopaque) c_int;
pub const sys_prot_t = c_int;
pub const struct_rng_128 = extern struct {
    r: [2]u64 = @import("std").mem.zeroes([2]u64),
    pub const @"128" = get_rand_128;
};
pub const rng_128_t = struct_rng_128;
pub extern fn get_rand_128(rand128: [*c]rng_128_t) void;
pub extern fn get_rand_64() u64;
pub extern fn get_rand_32() u32;
pub const div_t = extern struct {
    quot: c_int = 0,
    rem: c_int = 0,
};
pub const ldiv_t = extern struct {
    quot: c_long = 0,
    rem: c_long = 0,
};
pub const lldiv_t = extern struct {
    quot: c_longlong = 0,
    rem: c_longlong = 0,
};
pub const __compar_fn_t = ?*const fn (?*const anyopaque, ?*const anyopaque) callconv(.c) c_int;
pub extern fn __locale_mb_cur_max() c_int;
pub extern fn abort() noreturn;
pub extern fn abs(c_int) c_int;
pub extern fn arc4random() __uint32_t;
pub extern fn arc4random_uniform(__uint32_t) __uint32_t;
pub extern fn arc4random_buf(?*anyopaque, usize) void;
pub extern fn atexit(__func: ?*const fn () callconv(.c) void) c_int;
pub extern fn atof(__nptr: [*c]const u8) f64;
pub extern fn atoff(__nptr: [*c]const u8) f32;
pub extern fn atoi(__nptr: [*c]const u8) c_int;
pub extern fn _atoi_r([*c]struct__reent, __nptr: [*c]const u8) c_int;
pub extern fn atol(__nptr: [*c]const u8) c_long;
pub extern fn _atol_r([*c]struct__reent, __nptr: [*c]const u8) c_long;
pub extern fn bsearch(__key: ?*const anyopaque, __base: ?*const anyopaque, __nmemb: usize, __size: usize, _compar: __compar_fn_t) ?*anyopaque;
pub extern fn calloc(usize, usize) ?*anyopaque;
pub extern fn div(__numer: c_int, __denom: c_int) div_t;
pub extern fn exit(__status: c_int) noreturn;
pub extern fn free(?*anyopaque) void;
pub extern fn getenv(__string: [*c]const u8) [*c]u8;
pub extern fn _getenv_r([*c]struct__reent, __string: [*c]const u8) [*c]u8;
pub extern fn _findenv([*c]const u8, [*c]c_int) [*c]u8;
pub extern fn _findenv_r([*c]struct__reent, [*c]const u8, [*c]c_int) [*c]u8;
pub extern var suboptarg: [*c]u8;
pub extern fn getsubopt([*c][*c]u8, [*c]const [*c]u8, [*c][*c]u8) c_int;
pub extern fn labs(c_long) c_long;
pub extern fn ldiv(__numer: c_long, __denom: c_long) ldiv_t;
pub extern fn malloc(usize) ?*anyopaque;
pub extern fn mblen([*c]const u8, usize) c_int;
pub extern fn _mblen_r([*c]struct__reent, [*c]const u8, usize, [*c]_mbstate_t) c_int;
pub extern fn mbtowc(noalias [*c]wchar_t, noalias [*c]const u8, usize) c_int;
pub extern fn _mbtowc_r([*c]struct__reent, noalias [*c]wchar_t, noalias [*c]const u8, usize, [*c]_mbstate_t) c_int;
pub extern fn wctomb([*c]u8, wchar_t) c_int;
pub extern fn _wctomb_r([*c]struct__reent, [*c]u8, wchar_t, [*c]_mbstate_t) c_int;
pub extern fn mbstowcs(noalias [*c]wchar_t, noalias [*c]const u8, usize) usize;
pub extern fn _mbstowcs_r([*c]struct__reent, noalias [*c]wchar_t, noalias [*c]const u8, usize, [*c]_mbstate_t) usize;
pub extern fn wcstombs(noalias [*c]u8, noalias [*c]const wchar_t, usize) usize;
pub extern fn _wcstombs_r([*c]struct__reent, noalias [*c]u8, noalias [*c]const wchar_t, usize, [*c]_mbstate_t) usize;
pub extern fn mkdtemp([*c]u8) [*c]u8;
pub extern fn mkstemp([*c]u8) c_int;
pub extern fn mkstemps([*c]u8, c_int) c_int;
pub extern fn mktemp([*c]u8) [*c]u8;
pub extern fn _mkdtemp_r([*c]struct__reent, [*c]u8) [*c]u8;
pub extern fn _mkostemp_r([*c]struct__reent, [*c]u8, c_int) c_int;
pub extern fn _mkostemps_r([*c]struct__reent, [*c]u8, c_int, c_int) c_int;
pub extern fn _mkstemp_r([*c]struct__reent, [*c]u8) c_int;
pub extern fn _mkstemps_r([*c]struct__reent, [*c]u8, c_int) c_int;
pub extern fn _mktemp_r([*c]struct__reent, [*c]u8) [*c]u8;
pub extern fn qsort(__base: ?*anyopaque, __nmemb: usize, __size: usize, _compar: __compar_fn_t) void;
pub extern fn rand() c_int;
pub extern fn realloc(?*anyopaque, usize) ?*anyopaque;
pub extern fn reallocarray(?*anyopaque, usize, usize) ?*anyopaque;
pub extern fn reallocf(?*anyopaque, usize) ?*anyopaque;
pub extern fn realpath(noalias path: [*c]const u8, noalias resolved_path: [*c]u8) [*c]u8;
pub extern fn rpmatch(response: [*c]const u8) c_int;
pub extern fn srand(__seed: c_uint) void;
pub extern fn strtod(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8) f64;
pub extern fn _strtod_r([*c]struct__reent, noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8) f64;
pub extern fn strtof(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8) f32;
pub extern fn strtol(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_long;
pub extern fn _strtol_r([*c]struct__reent, noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_long;
pub extern fn strtoul(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_ulong;
pub extern fn _strtoul_r([*c]struct__reent, noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_ulong;
pub extern fn system(__string: [*c]const u8) c_int;
pub extern fn a64l(__input: [*c]const u8) c_long;
pub extern fn l64a(__input: c_long) [*c]u8;
pub extern fn _l64a_r([*c]struct__reent, __input: c_long) [*c]u8;
pub extern fn on_exit(__func: ?*const fn (c_int, ?*anyopaque) callconv(.c) void, __arg: ?*anyopaque) c_int;
pub extern fn _Exit(__status: c_int) noreturn;
pub extern fn putenv(__string: [*c]u8) c_int;
pub extern fn _putenv_r([*c]struct__reent, __string: [*c]u8) c_int;
pub extern fn _reallocf_r([*c]struct__reent, ?*anyopaque, usize) ?*anyopaque;
pub extern fn setenv(__string: [*c]const u8, __value: [*c]const u8, __overwrite: c_int) c_int;
pub extern fn _setenv_r([*c]struct__reent, __string: [*c]const u8, __value: [*c]const u8, __overwrite: c_int) c_int;
pub extern fn __itoa(c_int, [*c]u8, c_int) [*c]u8;
pub extern fn __utoa(c_uint, [*c]u8, c_int) [*c]u8;
pub extern fn itoa(c_int, [*c]u8, c_int) [*c]u8;
pub extern fn utoa(c_uint, [*c]u8, c_int) [*c]u8;
pub extern fn rand_r(__seed: [*c]c_uint) c_int;
pub extern fn drand48() f64;
pub extern fn _drand48_r([*c]struct__reent) f64;
pub extern fn erand48([*c]c_ushort) f64;
pub extern fn _erand48_r([*c]struct__reent, [*c]c_ushort) f64;
pub extern fn jrand48([*c]c_ushort) c_long;
pub extern fn _jrand48_r([*c]struct__reent, [*c]c_ushort) c_long;
pub extern fn lcong48([*c]c_ushort) void;
pub extern fn _lcong48_r([*c]struct__reent, [*c]c_ushort) void;
pub extern fn lrand48() c_long;
pub extern fn _lrand48_r([*c]struct__reent) c_long;
pub extern fn mrand48() c_long;
pub extern fn _mrand48_r([*c]struct__reent) c_long;
pub extern fn nrand48([*c]c_ushort) c_long;
pub extern fn _nrand48_r([*c]struct__reent, [*c]c_ushort) c_long;
pub extern fn seed48([*c]c_ushort) [*c]c_ushort;
pub extern fn _seed48_r([*c]struct__reent, [*c]c_ushort) [*c]c_ushort;
pub extern fn srand48(c_long) void;
pub extern fn _srand48_r([*c]struct__reent, c_long) void;
pub extern fn initstate(c_uint, [*c]u8, usize) [*c]u8;
pub extern fn random() c_long;
pub extern fn setstate([*c]u8) [*c]u8;
pub extern fn srandom(c_uint) void;
pub extern fn atoll(__nptr: [*c]const u8) c_longlong;
pub extern fn _atoll_r([*c]struct__reent, __nptr: [*c]const u8) c_longlong;
pub extern fn llabs(c_longlong) c_longlong;
pub extern fn lldiv(__numer: c_longlong, __denom: c_longlong) lldiv_t;
pub extern fn strtoll(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_longlong;
pub extern fn _strtoll_r([*c]struct__reent, noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_longlong;
pub extern fn strtoull(noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_ulonglong;
pub extern fn _strtoull_r([*c]struct__reent, noalias __n: [*c]const u8, noalias __end_PTR: [*c][*c]u8, __base: c_int) c_ulonglong;
pub extern fn cfree(?*anyopaque) void;
pub extern fn unsetenv(__string: [*c]const u8) c_int;
pub extern fn _unsetenv_r([*c]struct__reent, __string: [*c]const u8) c_int;
pub extern fn posix_memalign([*c]?*anyopaque, usize, usize) c_int;
pub extern fn _dtoa_r([*c]struct__reent, f64, c_int, c_int, [*c]c_int, [*c]c_int, [*c][*c]u8) [*c]u8;
pub extern fn _malloc_r([*c]struct__reent, usize) ?*anyopaque;
pub extern fn _calloc_r([*c]struct__reent, usize, usize) ?*anyopaque;
pub extern fn _free_r([*c]struct__reent, ?*anyopaque) void;
pub extern fn _realloc_r([*c]struct__reent, ?*anyopaque, usize) ?*anyopaque;
pub extern fn _mstats_r([*c]struct__reent, [*c]u8) void;
pub extern fn _system_r([*c]struct__reent, [*c]const u8) c_int;
pub extern fn __eprintf([*c]const u8, [*c]const u8, c_uint, [*c]const u8) void;
pub extern fn qsort_r(__base: ?*anyopaque, __nmemb: usize, __size: usize, __thunk: ?*anyopaque, _compar: ?*const fn (?*anyopaque, ?*const anyopaque, ?*const anyopaque) callconv(.c) c_int) void;
pub extern fn _strtold_r([*c]struct__reent, noalias [*c]const u8, noalias [*c][*c]u8) c_longdouble;
pub extern fn strtold(noalias [*c]const u8, noalias [*c][*c]u8) c_longdouble;
pub extern fn aligned_alloc(usize, usize) ?*anyopaque;
pub extern fn at_quick_exit(?*const fn () callconv(.c) void) c_int;
pub extern fn quick_exit(c_int) noreturn;
pub const u8_t = u8;
pub const s8_t = i8;
pub const u16_t = u16;
pub const s16_t = i16;
pub const u32_t = u32;
pub const s32_t = i32;
pub const u64_t = u64;
pub const s64_t = i64;
pub const mem_ptr_t = usize;
pub const imaxdiv_t = extern struct {
    quot: intmax_t = 0,
    rem: intmax_t = 0,
};
pub extern fn imaxabs(intmax_t) intmax_t;
pub extern fn imaxdiv(__numer: intmax_t, __denomer: intmax_t) imaxdiv_t;
pub extern fn strtoimax(noalias [*c]const u8, noalias [*c][*c]u8, c_int) intmax_t;
pub extern fn _strtoimax_r([*c]struct__reent, noalias [*c]const u8, noalias [*c][*c]u8, c_int) intmax_t;
pub extern fn strtoumax(noalias [*c]const u8, noalias [*c][*c]u8, c_int) uintmax_t;
pub extern fn _strtoumax_r([*c]struct__reent, noalias [*c]const u8, noalias [*c][*c]u8, c_int) uintmax_t;
pub extern fn wcstoimax(noalias [*c]const wchar_t, noalias [*c][*c]wchar_t, c_int) intmax_t;
pub extern fn _wcstoimax_r([*c]struct__reent, noalias [*c]const wchar_t, noalias [*c][*c]wchar_t, c_int) intmax_t;
pub extern fn wcstoumax(noalias [*c]const wchar_t, noalias [*c][*c]wchar_t, c_int) uintmax_t;
pub extern fn _wcstoumax_r([*c]struct__reent, noalias [*c]const wchar_t, noalias [*c][*c]wchar_t, c_int) uintmax_t;
pub extern fn strtoimax_l(noalias [*c]const u8, _restrict: [*c][*c]u8, c_int, locale_t) intmax_t;
pub extern fn strtoumax_l(noalias [*c]const u8, _restrict: [*c][*c]u8, c_int, locale_t) uintmax_t;
pub extern fn wcstoimax_l(noalias [*c]const wchar_t, _restrict: [*c][*c]wchar_t, c_int, locale_t) intmax_t;
pub extern fn wcstoumax_l(noalias [*c]const wchar_t, _restrict: [*c][*c]wchar_t, c_int, locale_t) uintmax_t;
pub extern fn isalnum(__c: c_int) c_int;
pub extern fn isalpha(__c: c_int) c_int;
pub extern fn iscntrl(__c: c_int) c_int;
pub extern fn isdigit(__c: c_int) c_int;
pub extern fn isgraph(__c: c_int) c_int;
pub extern fn islower(__c: c_int) c_int;
pub extern fn isprint(__c: c_int) c_int;
pub extern fn ispunct(__c: c_int) c_int;
pub extern fn isspace(__c: c_int) c_int;
pub extern fn isupper(__c: c_int) c_int;
pub extern fn isxdigit(__c: c_int) c_int;
pub extern fn tolower(__c: c_int) c_int;
pub extern fn toupper(__c: c_int) c_int;
pub extern fn isblank(__c: c_int) c_int;
pub extern fn isascii(__c: c_int) c_int;
pub extern fn toascii(__c: c_int) c_int;
pub extern fn isalnum_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isalpha_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isblank_l(__c: c_int, __l: locale_t) c_int;
pub extern fn iscntrl_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isdigit_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isgraph_l(__c: c_int, __l: locale_t) c_int;
pub extern fn islower_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isprint_l(__c: c_int, __l: locale_t) c_int;
pub extern fn ispunct_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isspace_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isupper_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isxdigit_l(__c: c_int, __l: locale_t) c_int;
pub extern fn tolower_l(__c: c_int, __l: locale_t) c_int;
pub extern fn toupper_l(__c: c_int, __l: locale_t) c_int;
pub extern fn isascii_l(__c: c_int, __l: locale_t) c_int;
pub extern fn toascii_l(__c: c_int, __l: locale_t) c_int;
pub const _ctype_: [*c]const u8 = @extern([*c]const u8, .{
    .name = "_ctype_",
});
pub fn __locale_ctype_ptr_l(arg__l: locale_t) callconv(.c) [*c]const u8 {
    var _l = arg__l;
    _ = &_l;
    _ = &_l;
    return _ctype_;
}
pub const ERR_OK: c_int = 0;
pub const ERR_MEM: c_int = -1;
pub const ERR_BUF: c_int = -2;
pub const ERR_TIMEOUT: c_int = -3;
pub const ERR_RTE: c_int = -4;
pub const ERR_INPROGRESS: c_int = -5;
pub const ERR_VAL: c_int = -6;
pub const ERR_WOULDBLOCK: c_int = -7;
pub const ERR_USE: c_int = -8;
pub const ERR_ALREADY: c_int = -9;
pub const ERR_ISCONN: c_int = -10;
pub const ERR_CONN: c_int = -11;
pub const ERR_IF: c_int = -12;
pub const ERR_ABRT: c_int = -13;
pub const ERR_RST: c_int = -14;
pub const ERR_CLSD: c_int = -15;
pub const ERR_ARG: c_int = -16;
pub const err_enum_t = c_int;
pub const err_t = s8_t;
pub extern fn lwip_strerr(err: err_t) [*c]const u8;
pub extern fn lwip_htons(x: u16_t) u16_t;
pub extern fn lwip_htonl(x: u32_t) u32_t;
pub extern fn lwip_itoa(result: [*c]u8, bufsize: usize, number: c_int) void;
pub extern fn lwip_strnicmp(str1: [*c]const u8, str2: [*c]const u8, len: usize) c_int;
pub extern fn lwip_stricmp(str1: [*c]const u8, str2: [*c]const u8) c_int;
pub extern fn lwip_strnstr(buffer: [*c]const u8, token: [*c]const u8, n: usize) [*c]u8;
pub extern fn lwip_strnistr(buffer: [*c]const u8, token: [*c]const u8, n: usize) [*c]u8;
pub extern fn lwip_memcmp_consttime(s1: ?*const anyopaque, s2: ?*const anyopaque, len: usize) c_int;
pub const struct_ip4_addr = extern struct {
    addr: u32_t = 0,
    pub const ntoa = ip4addr_ntoa;
    pub const r = ip4addr_ntoa_r;
    pub const route = ip4_route;
    pub const changed = udp_netif_ip_addr_changed;
};
pub const ip4_addr_t = struct_ip4_addr;
pub const ip_addr_t = ip4_addr_t;
pub const struct_pbuf = extern struct {
    next: [*c]struct_pbuf = null,
    payload: ?*anyopaque = null,
    tot_len: u16_t = 0,
    len: u16_t = 0,
    type_internal: u8_t = 0,
    flags: u8_t = 0,
    ref: u8_t = 0,
    if_idx: u8_t = 0,
    pub const realloc = pbuf_realloc;
    pub const header = pbuf_header;
    pub const force = pbuf_header_force;
    pub const header1 = pbuf_add_header;
    pub const force1 = pbuf_add_header_force;
    pub const header2 = pbuf_remove_header;
    pub const header3 = pbuf_free_header;
    pub const ref1 = pbuf_ref;
    pub const free = pbuf_free;
    pub const cat = pbuf_cat;
    pub const chain = pbuf_chain;
    pub const dechain = pbuf_dechain;
    pub const copy = pbuf_copy;
    pub const pbuf = pbuf_copy_partial_pbuf;
    pub const take = pbuf_take;
    pub const at = pbuf_take_at;
    pub const skip = pbuf_skip;
    pub const coalesce = pbuf_coalesce;
    pub const at1 = pbuf_put_at;
    pub const input = netif_input;
    pub const input1 = ip4_input;
    pub const output = ip4_output;
    pub const @"if" = ip4_output_if;
    pub const src = ip4_output_if_src;
    pub const input2 = udp_input;
};
pub const netif_input_fn = ?*const fn (p: [*c]struct_pbuf, inp: [*c]struct_netif) callconv(.c) err_t;
pub const netif_output_fn = ?*const fn (netif: [*c]struct_netif, p: [*c]struct_pbuf, ipaddr: [*c]const ip4_addr_t) callconv(.c) err_t;
pub const netif_linkoutput_fn = ?*const fn (netif: [*c]struct_netif, p: [*c]struct_pbuf) callconv(.c) err_t;
pub const netif_status_callback_fn = ?*const fn (netif: [*c]struct_netif) callconv(.c) void;
pub const struct_netif = extern struct {
    next: [*c]struct_netif = null,
    ip_addr: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    netmask: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    gw: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    input: netif_input_fn = null,
    output: netif_output_fn = null,
    linkoutput: netif_linkoutput_fn = null,
    status_callback: netif_status_callback_fn = null,
    link_callback: netif_status_callback_fn = null,
    state: ?*anyopaque = null,
    client_data: [1]?*anyopaque = @import("std").mem.zeroes([1]?*anyopaque),
    hostname: [*c]const u8 = null,
    mtu: u16_t = 0,
    hwaddr: [6]u8_t = @import("std").mem.zeroes([6]u8_t),
    hwaddr_len: u8_t = 0,
    flags: u8_t = 0,
    name: [2]u8 = @import("std").mem.zeroes([2]u8),
    num: u8_t = 0,
    pub const address = dhcp_supplied_address;
};
pub extern fn ip4_addr_isbroadcast_u32(addr: u32_t, netif: [*c]const struct_netif) u8_t;
pub extern fn ip4_addr_netmask_valid(netmask: u32_t) u8_t;
pub extern fn ipaddr_addr(cp: [*c]const u8) u32_t;
pub extern fn ip4addr_aton(cp: [*c]const u8, addr: [*c]ip4_addr_t) c_int;
pub extern fn ip4addr_ntoa(addr: [*c]const ip4_addr_t) [*c]u8;
pub extern fn ip4addr_ntoa_r(addr: [*c]const ip4_addr_t, buf: [*c]u8, buflen: c_int) [*c]u8;
pub const IPADDR_TYPE_V4: c_int = 0;
pub const IPADDR_TYPE_V6: c_int = 6;
pub const IPADDR_TYPE_ANY: c_int = 46;
pub const enum_lwip_ip_addr_type = c_uint;
pub extern const ip_addr_any: ip_addr_t;
pub extern const ip_addr_broadcast: ip_addr_t;
pub const PBUF_TRANSPORT: c_int = 54;
pub const PBUF_IP: c_int = 34;
pub const PBUF_LINK: c_int = 14;
pub const PBUF_RAW_TX: c_int = 0;
pub const PBUF_RAW: c_int = 0;
pub const pbuf_layer = c_uint;
pub const PBUF_RAM: c_int = 640;
pub const PBUF_ROM: c_int = 1;
pub const PBUF_REF: c_int = 65;
pub const PBUF_POOL: c_int = 386;
pub const pbuf_type = c_uint;
pub const struct_pbuf_rom = extern struct {
    next: [*c]struct_pbuf = null,
    payload: ?*const anyopaque = null,
};
pub extern var pbuf_free_ooseq_pending: u8_t;
pub extern fn pbuf_free_ooseq() void;
pub extern fn pbuf_alloc(l: pbuf_layer, length: u16_t, @"type": pbuf_type) [*c]struct_pbuf;
pub extern fn pbuf_alloc_reference(payload: ?*anyopaque, length: u16_t, @"type": pbuf_type) [*c]struct_pbuf;
pub extern fn pbuf_realloc(p: [*c]struct_pbuf, size: u16_t) void;
pub extern fn pbuf_header(p: [*c]struct_pbuf, header_size: s16_t) u8_t;
pub extern fn pbuf_header_force(p: [*c]struct_pbuf, header_size: s16_t) u8_t;
pub extern fn pbuf_add_header(p: [*c]struct_pbuf, header_size_increment: usize) u8_t;
pub extern fn pbuf_add_header_force(p: [*c]struct_pbuf, header_size_increment: usize) u8_t;
pub extern fn pbuf_remove_header(p: [*c]struct_pbuf, header_size: usize) u8_t;
pub extern fn pbuf_free_header(q: [*c]struct_pbuf, size: u16_t) [*c]struct_pbuf;
pub extern fn pbuf_ref(p: [*c]struct_pbuf) void;
pub extern fn pbuf_free(p: [*c]struct_pbuf) u8_t;
pub extern fn pbuf_clen(p: [*c]const struct_pbuf) u16_t;
pub extern fn pbuf_cat(head: [*c]struct_pbuf, tail: [*c]struct_pbuf) void;
pub extern fn pbuf_chain(head: [*c]struct_pbuf, tail: [*c]struct_pbuf) void;
pub extern fn pbuf_dechain(p: [*c]struct_pbuf) [*c]struct_pbuf;
pub extern fn pbuf_copy(p_to: [*c]struct_pbuf, p_from: [*c]const struct_pbuf) err_t;
pub extern fn pbuf_copy_partial_pbuf(p_to: [*c]struct_pbuf, p_from: [*c]const struct_pbuf, copy_len: u16_t, offset: u16_t) err_t;
pub extern fn pbuf_copy_partial(p: [*c]const struct_pbuf, dataptr: ?*anyopaque, len: u16_t, offset: u16_t) u16_t;
pub extern fn pbuf_get_contiguous(p: [*c]const struct_pbuf, buffer: ?*anyopaque, bufsize: usize, len: u16_t, offset: u16_t) ?*anyopaque;
pub extern fn pbuf_take(buf: [*c]struct_pbuf, dataptr: ?*const anyopaque, len: u16_t) err_t;
pub extern fn pbuf_take_at(buf: [*c]struct_pbuf, dataptr: ?*const anyopaque, len: u16_t, offset: u16_t) err_t;
pub extern fn pbuf_skip(in: [*c]struct_pbuf, in_offset: u16_t, out_offset: [*c]u16_t) [*c]struct_pbuf;
pub extern fn pbuf_coalesce(p: [*c]struct_pbuf, layer: pbuf_layer) [*c]struct_pbuf;
pub extern fn pbuf_clone(l: pbuf_layer, @"type": pbuf_type, p: [*c]struct_pbuf) [*c]struct_pbuf;
pub extern fn pbuf_get_at(p: [*c]const struct_pbuf, offset: u16_t) u8_t;
pub extern fn pbuf_try_get_at(p: [*c]const struct_pbuf, offset: u16_t) c_int;
pub extern fn pbuf_put_at(p: [*c]struct_pbuf, offset: u16_t, data: u8_t) void;
pub extern fn pbuf_memcmp(p: [*c]const struct_pbuf, offset: u16_t, s2: ?*const anyopaque, n: u16_t) u16_t;
pub extern fn pbuf_memfind(p: [*c]const struct_pbuf, mem: ?*const anyopaque, mem_len: u16_t, start_offset: u16_t) u16_t;
pub extern fn pbuf_strstr(p: [*c]const struct_pbuf, substr: [*c]const u8) u16_t;
pub const mem_size_t = u16_t;
pub extern fn mem_init() void;
pub extern fn mem_trim(mem: ?*anyopaque, size: mem_size_t) ?*anyopaque;
pub extern fn mem_malloc(size: mem_size_t) ?*anyopaque;
pub extern fn mem_calloc(count: mem_size_t, size: mem_size_t) ?*anyopaque;
pub extern fn mem_free(mem: ?*anyopaque) void;
pub const MEMP_RAW_PCB: c_int = 0;
pub const MEMP_UDP_PCB: c_int = 1;
pub const MEMP_TCP_PCB: c_int = 2;
pub const MEMP_TCP_PCB_LISTEN: c_int = 3;
pub const MEMP_TCP_SEG: c_int = 4;
pub const MEMP_REASSDATA: c_int = 5;
pub const MEMP_SYS_TIMEOUT: c_int = 6;
pub const MEMP_PBUF: c_int = 7;
pub const MEMP_PBUF_POOL: c_int = 8;
pub const MEMP_MAX: c_int = 9;
pub const memp_t = c_uint;
pub const struct_memp = extern struct {
    next: [*c]struct_memp = null,
};
pub const struct_memp_desc = extern struct {
    desc: [*c]const u8 = null,
    size: u16_t = 0,
    num: u16_t = 0,
    base: [*c]u8_t = null,
    tab: [*c][*c]struct_memp = null,
};
pub extern fn memp_init_pool(desc: [*c]const struct_memp_desc) void;
pub extern fn memp_malloc_pool(desc: [*c]const struct_memp_desc) ?*anyopaque;
pub extern fn memp_free_pool(desc: [*c]const struct_memp_desc, mem: ?*anyopaque) void;
pub extern const memp_pools: [9][*c]const struct_memp_desc;
pub extern fn memp_init() void;
pub extern fn memp_malloc(@"type": memp_t) ?*anyopaque;
pub extern fn memp_free(@"type": memp_t, mem: ?*anyopaque) void;
pub const struct_stats_proto = extern struct {
    xmit: u16_t = 0,
    recv: u16_t = 0,
    fw: u16_t = 0,
    drop: u16_t = 0,
    chkerr: u16_t = 0,
    lenerr: u16_t = 0,
    memerr: u16_t = 0,
    rterr: u16_t = 0,
    proterr: u16_t = 0,
    opterr: u16_t = 0,
    err: u16_t = 0,
    cachehit: u16_t = 0,
    pub const proto = stats_display_proto;
};
pub const struct_stats_igmp = extern struct {
    xmit: u16_t = 0,
    recv: u16_t = 0,
    drop: u16_t = 0,
    chkerr: u16_t = 0,
    lenerr: u16_t = 0,
    memerr: u16_t = 0,
    proterr: u16_t = 0,
    rx_v1: u16_t = 0,
    rx_group: u16_t = 0,
    rx_general: u16_t = 0,
    rx_report: u16_t = 0,
    tx_join: u16_t = 0,
    tx_leave: u16_t = 0,
    tx_report: u16_t = 0,
    pub const igmp = stats_display_igmp;
};
pub const struct_stats_mem = extern struct {
    name: [*c]const u8 = null,
    err: u16_t = 0,
    avail: mem_size_t = 0,
    used: mem_size_t = 0,
    max: mem_size_t = 0,
    illegal: u16_t = 0,
    pub const mem = stats_display_mem;
    pub const memp = stats_display_memp;
};
pub const struct_stats_syselem = extern struct {
    used: u16_t = 0,
    max: u16_t = 0,
    err: u16_t = 0,
};
pub const struct_stats_sys = extern struct {
    sem: struct_stats_syselem = @import("std").mem.zeroes(struct_stats_syselem),
    mutex: struct_stats_syselem = @import("std").mem.zeroes(struct_stats_syselem),
    mbox: struct_stats_syselem = @import("std").mem.zeroes(struct_stats_syselem),
    pub const sys = stats_display_sys;
};
pub const struct_stats_mib2 = extern struct {
    ipinhdrerrors: u32_t = 0,
    ipinaddrerrors: u32_t = 0,
    ipinunknownprotos: u32_t = 0,
    ipindiscards: u32_t = 0,
    ipindelivers: u32_t = 0,
    ipoutrequests: u32_t = 0,
    ipoutdiscards: u32_t = 0,
    ipoutnoroutes: u32_t = 0,
    ipreasmoks: u32_t = 0,
    ipreasmfails: u32_t = 0,
    ipfragoks: u32_t = 0,
    ipfragfails: u32_t = 0,
    ipfragcreates: u32_t = 0,
    ipreasmreqds: u32_t = 0,
    ipforwdatagrams: u32_t = 0,
    ipinreceives: u32_t = 0,
    ip6reasmoks: u32_t = 0,
    tcpactiveopens: u32_t = 0,
    tcppassiveopens: u32_t = 0,
    tcpattemptfails: u32_t = 0,
    tcpestabresets: u32_t = 0,
    tcpoutsegs: u32_t = 0,
    tcpretranssegs: u32_t = 0,
    tcpinsegs: u32_t = 0,
    tcpinerrs: u32_t = 0,
    tcpoutrsts: u32_t = 0,
    udpindatagrams: u32_t = 0,
    udpnoports: u32_t = 0,
    udpinerrors: u32_t = 0,
    udpoutdatagrams: u32_t = 0,
    icmpinmsgs: u32_t = 0,
    icmpinerrors: u32_t = 0,
    icmpindestunreachs: u32_t = 0,
    icmpintimeexcds: u32_t = 0,
    icmpinparmprobs: u32_t = 0,
    icmpinsrcquenchs: u32_t = 0,
    icmpinredirects: u32_t = 0,
    icmpinechos: u32_t = 0,
    icmpinechoreps: u32_t = 0,
    icmpintimestamps: u32_t = 0,
    icmpintimestampreps: u32_t = 0,
    icmpinaddrmasks: u32_t = 0,
    icmpinaddrmaskreps: u32_t = 0,
    icmpoutmsgs: u32_t = 0,
    icmpouterrors: u32_t = 0,
    icmpoutdestunreachs: u32_t = 0,
    icmpouttimeexcds: u32_t = 0,
    icmpoutechos: u32_t = 0,
    icmpoutechoreps: u32_t = 0,
};
pub const struct_stats_mib2_netif_ctrs = extern struct {
    ifinoctets: u32_t = 0,
    ifinucastpkts: u32_t = 0,
    ifinnucastpkts: u32_t = 0,
    ifindiscards: u32_t = 0,
    ifinerrors: u32_t = 0,
    ifinunknownprotos: u32_t = 0,
    ifoutoctets: u32_t = 0,
    ifoutucastpkts: u32_t = 0,
    ifoutnucastpkts: u32_t = 0,
    ifoutdiscards: u32_t = 0,
    ifouterrors: u32_t = 0,
};
pub const struct_stats_ = extern struct {
    etharp: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
    ip_frag: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
    ip: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
    icmp: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
    udp: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
    tcp: struct_stats_proto = @import("std").mem.zeroes(struct_stats_proto),
};
pub extern var lwip_stats: struct_stats_;
pub extern fn stats_init() void;
pub extern fn stats_display() void;
pub extern fn stats_display_proto(proto: [*c]struct_stats_proto, name: [*c]const u8) void;
pub extern fn stats_display_igmp(igmp: [*c]struct_stats_igmp, name: [*c]const u8) void;
pub extern fn stats_display_mem(mem: [*c]struct_stats_mem, name: [*c]const u8) void;
pub extern fn stats_display_memp(mem: [*c]struct_stats_mem, index: c_int) void;
pub extern fn stats_display_sys(sys: [*c]struct_stats_sys) void;
pub const LWIP_NETIF_CLIENT_DATA_INDEX_DHCP: c_int = 0;
pub const LWIP_NETIF_CLIENT_DATA_INDEX_MAX: c_int = 1;
pub const enum_lwip_internal_netif_client_data_index = c_uint;
pub const NETIF_DEL_MAC_FILTER: c_int = 0;
pub const NETIF_ADD_MAC_FILTER: c_int = 1;
pub const enum_netif_mac_filter_action = c_uint;
pub const netif_init_fn = ?*const fn (netif: [*c]struct_netif) callconv(.c) err_t;
pub const netif_addr_idx_t = u8_t;
pub extern var netif_list: [*c]struct_netif;
pub extern var netif_default: [*c]struct_netif;
pub extern fn netif_init() void;
pub extern fn netif_add_noaddr(netif: [*c]struct_netif, state: ?*anyopaque, init: netif_init_fn, input: netif_input_fn) [*c]struct_netif;
pub extern fn netif_add(netif: [*c]struct_netif, ipaddr: [*c]const ip4_addr_t, netmask: [*c]const ip4_addr_t, gw: [*c]const ip4_addr_t, state: ?*anyopaque, init: netif_init_fn, input: netif_input_fn) [*c]struct_netif;
pub extern fn netif_set_addr(netif: [*c]struct_netif, ipaddr: [*c]const ip4_addr_t, netmask: [*c]const ip4_addr_t, gw: [*c]const ip4_addr_t) void;
pub extern fn netif_remove(netif: [*c]struct_netif) void;
pub extern fn netif_find(name: [*c]const u8) [*c]struct_netif;
pub extern fn netif_set_default(netif: [*c]struct_netif) void;
pub extern fn netif_set_ipaddr(netif: [*c]struct_netif, ipaddr: [*c]const ip4_addr_t) void;
pub extern fn netif_set_netmask(netif: [*c]struct_netif, netmask: [*c]const ip4_addr_t) void;
pub extern fn netif_set_gw(netif: [*c]struct_netif, gw: [*c]const ip4_addr_t) void;
pub extern fn netif_set_up(netif: [*c]struct_netif) void;
pub extern fn netif_set_down(netif: [*c]struct_netif) void;
pub extern fn netif_set_status_callback(netif: [*c]struct_netif, status_callback: netif_status_callback_fn) void;
pub extern fn netif_set_link_up(netif: [*c]struct_netif) void;
pub extern fn netif_set_link_down(netif: [*c]struct_netif) void;
pub extern fn netif_set_link_callback(netif: [*c]struct_netif, link_callback: netif_status_callback_fn) void;
pub extern fn netif_input(p: [*c]struct_pbuf, inp: [*c]struct_netif) err_t;
pub extern fn netif_name_to_index(name: [*c]const u8) u8_t;
pub extern fn netif_index_to_name(idx: u8_t, name: [*c]u8) [*c]u8;
pub extern fn netif_get_by_index(idx: u8_t) [*c]struct_netif;
pub const netif_nsc_reason_t = u16_t;
pub const struct_link_changed_s_7 = extern struct {
    state: u8_t = 0,
};
pub const struct_status_changed_s_8 = extern struct {
    state: u8_t = 0,
};
pub const struct_ipv4_changed_s_9 = extern struct {
    old_address: [*c]const ip_addr_t = null,
    old_netmask: [*c]const ip_addr_t = null,
    old_gw: [*c]const ip_addr_t = null,
};
pub const struct_ipv6_set_s_10 = extern struct {
    addr_index: s8_t = 0,
    old_address: [*c]const ip_addr_t = null,
};
pub const struct_ipv6_addr_state_changed_s_11 = extern struct {
    addr_index: s8_t = 0,
    old_state: u8_t = 0,
    address: [*c]const ip_addr_t = null,
};
pub const netif_ext_callback_args_t = extern union {
    link_changed: struct_link_changed_s_7,
    status_changed: struct_status_changed_s_8,
    ipv4_changed: struct_ipv4_changed_s_9,
    ipv6_set: struct_ipv6_set_s_10,
    ipv6_addr_state_changed: struct_ipv6_addr_state_changed_s_11,
};
pub const netif_ext_callback_fn = ?*const fn (netif: [*c]struct_netif, reason: netif_nsc_reason_t, args: [*c]const netif_ext_callback_args_t) callconv(.c) void;
pub const struct_ip4_addr_packed = extern struct {
    addr: u32_t align(1) = 0,
};
pub const ip4_addr_p_t = struct_ip4_addr_packed;
pub const struct_ip_hdr = extern struct {
    _v_hl: u8_t align(1) = 0,
    _tos: u8_t = 0,
    _len: u16_t align(1) = 0,
    _id: u16_t align(1) = 0,
    _offset: u16_t align(1) = 0,
    _ttl: u8_t = 0,
    _proto: u8_t = 0,
    _chksum: u16_t align(1) = 0,
    src: ip4_addr_p_t = @import("std").mem.zeroes(ip4_addr_p_t),
    dest: ip4_addr_p_t = @import("std").mem.zeroes(ip4_addr_p_t),
};
pub extern fn ip4_route(dest: [*c]const ip4_addr_t) [*c]struct_netif;
pub extern fn ip4_input(p: [*c]struct_pbuf, inp: [*c]struct_netif) err_t;
pub extern fn ip4_output(p: [*c]struct_pbuf, src: [*c]const ip4_addr_t, dest: [*c]const ip4_addr_t, ttl: u8_t, tos: u8_t, proto: u8_t) err_t;
pub extern fn ip4_output_if(p: [*c]struct_pbuf, src: [*c]const ip4_addr_t, dest: [*c]const ip4_addr_t, ttl: u8_t, tos: u8_t, proto: u8_t, netif: [*c]struct_netif) err_t;
pub extern fn ip4_output_if_src(p: [*c]struct_pbuf, src: [*c]const ip4_addr_t, dest: [*c]const ip4_addr_t, ttl: u8_t, tos: u8_t, proto: u8_t, netif: [*c]struct_netif) err_t;
pub const struct_ip_pcb = extern struct {
    local_ip: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    remote_ip: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    netif_idx: u8_t = 0,
    so_options: u8_t = 0,
    tos: u8_t = 0,
    ttl: u8_t = 0,
};
pub const struct_ip_globals = extern struct {
    current_netif: [*c]struct_netif = null,
    current_input_netif: [*c]struct_netif = null,
    current_ip4_header: [*c]const struct_ip_hdr = null,
    current_ip_header_tot_len: u16_t = 0,
    current_iphdr_src: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    current_iphdr_dest: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
};
pub extern var ip_data: struct_ip_globals;
pub const struct_udp_hdr = extern struct {
    src: u16_t align(1) = 0,
    dest: u16_t align(1) = 0,
    len: u16_t align(1) = 0,
    chksum: u16_t align(1) = 0,
};
pub const struct_udp_pcb = extern struct {
    local_ip: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    remote_ip: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    netif_idx: u8_t = 0,
    so_options: u8_t = 0,
    tos: u8_t = 0,
    ttl: u8_t = 0,
    next: [*c]struct_udp_pcb = null,
    flags: u8_t = 0,
    local_port: u16_t = 0,
    remote_port: u16_t = 0,
    recv: udp_recv_fn = null,
    recv_arg: ?*anyopaque = null,
    pub const remove = udp_remove;
    pub const bind = udp_bind;
    pub const netif = udp_bind_netif;
    pub const connect = udp_connect;
    pub const disconnect = udp_disconnect;
    pub const recv1 = udp_recv;
    pub const @"if" = udp_sendto_if;
    pub const src = udp_sendto_if_src;
    pub const sendto = udp_sendto;
    pub const send = udp_send;
};
pub const udp_recv_fn = ?*const fn (arg: ?*anyopaque, pcb: [*c]struct_udp_pcb, p: [*c]struct_pbuf, addr: [*c]const ip_addr_t, port: u16_t) callconv(.c) void;
pub extern var udp_pcbs: [*c]struct_udp_pcb;
pub extern fn udp_new() [*c]struct_udp_pcb;
pub extern fn udp_new_ip_type(@"type": u8_t) [*c]struct_udp_pcb;
pub extern fn udp_remove(pcb: [*c]struct_udp_pcb) void;
pub extern fn udp_bind(pcb: [*c]struct_udp_pcb, ipaddr: [*c]const ip_addr_t, port: u16_t) err_t;
pub extern fn udp_bind_netif(pcb: [*c]struct_udp_pcb, netif: [*c]const struct_netif) void;
pub extern fn udp_connect(pcb: [*c]struct_udp_pcb, ipaddr: [*c]const ip_addr_t, port: u16_t) err_t;
pub extern fn udp_disconnect(pcb: [*c]struct_udp_pcb) void;
pub extern fn udp_recv(pcb: [*c]struct_udp_pcb, recv: udp_recv_fn, recv_arg: ?*anyopaque) void;
pub extern fn udp_sendto_if(pcb: [*c]struct_udp_pcb, p: [*c]struct_pbuf, dst_ip: [*c]const ip_addr_t, dst_port: u16_t, netif: [*c]struct_netif) err_t;
pub extern fn udp_sendto_if_src(pcb: [*c]struct_udp_pcb, p: [*c]struct_pbuf, dst_ip: [*c]const ip_addr_t, dst_port: u16_t, netif: [*c]struct_netif, src_ip: [*c]const ip_addr_t) err_t;
pub extern fn udp_sendto(pcb: [*c]struct_udp_pcb, p: [*c]struct_pbuf, dst_ip: [*c]const ip_addr_t, dst_port: u16_t) err_t;
pub extern fn udp_send(pcb: [*c]struct_udp_pcb, p: [*c]struct_pbuf) err_t;
pub extern fn udp_input(p: [*c]struct_pbuf, inp: [*c]struct_netif) void;
pub extern fn udp_init() void;
pub extern fn udp_netif_ip_addr_changed(old_addr: [*c]const ip_addr_t, new_addr: [*c]const ip_addr_t) void;
pub const dhcp_timeout_t = u16_t;
pub const DHCP_AUTOIP_COOP_STATE_OFF: c_int = 0;
pub const DHCP_AUTOIP_COOP_STATE_ON: c_int = 1;
pub const dhcp_autoip_coop_state_enum_t = c_uint;
pub const struct_dhcp = extern struct {
    xid: u32_t = 0,
    pcb_allocated: u8_t = 0,
    state: u8_t = 0,
    tries: u8_t = 0,
    flags: u8_t = 0,
    request_timeout: dhcp_timeout_t = 0,
    t1_timeout: dhcp_timeout_t = 0,
    t2_timeout: dhcp_timeout_t = 0,
    t1_renew_time: dhcp_timeout_t = 0,
    t2_rebind_time: dhcp_timeout_t = 0,
    lease_used: dhcp_timeout_t = 0,
    t0_timeout: dhcp_timeout_t = 0,
    server_ip_addr: ip_addr_t = @import("std").mem.zeroes(ip_addr_t),
    offered_ip_addr: ip4_addr_t = @import("std").mem.zeroes(ip4_addr_t),
    offered_sn_mask: ip4_addr_t = @import("std").mem.zeroes(ip4_addr_t),
    offered_gw_addr: ip4_addr_t = @import("std").mem.zeroes(ip4_addr_t),
    offered_t0_lease: u32_t = 0,
    offered_t1_renew: u32_t = 0,
    offered_t2_rebind: u32_t = 0,
};
pub extern fn dhcp_set_struct(netif: [*c]struct_netif, dhcp: [*c]struct_dhcp) void;
pub extern fn dhcp_cleanup(netif: [*c]struct_netif) void;
pub extern fn dhcp_start(netif: [*c]struct_netif) err_t;
pub extern fn dhcp_renew(netif: [*c]struct_netif) err_t;
pub extern fn dhcp_release(netif: [*c]struct_netif) err_t;
pub extern fn dhcp_stop(netif: [*c]struct_netif) void;
pub extern fn dhcp_release_and_stop(netif: [*c]struct_netif) void;
pub extern fn dhcp_inform(netif: [*c]struct_netif) void;
pub extern fn dhcp_network_changed_link_up(netif: [*c]struct_netif) void;
pub extern fn dhcp_supplied_address(netif: [*c]const struct_netif) u8_t;
pub extern fn dhcp_coarse_tmr() void;
pub extern fn dhcp_fine_tmr() void;
pub const CYW43_ITF_STA: c_int = 0;
pub const CYW43_ITF_AP: c_int = 1;
const enum_unnamed_12 = c_uint;
pub const struct__cyw43_ev_scan_result_t = extern struct {
    _0: [5]u32 = @import("std").mem.zeroes([5]u32),
    bssid: [6]u8 = @import("std").mem.zeroes([6]u8),
    _1: [2]u16 = @import("std").mem.zeroes([2]u16),
    ssid_len: u8 = 0,
    ssid: [32]u8 = @import("std").mem.zeroes([32]u8),
    _2: [5]u32 = @import("std").mem.zeroes([5]u32),
    channel: u16 = 0,
    _3: u16 = 0,
    auth_mode: u8 = 0,
    rssi: i16 = 0,
};
pub const cyw43_ev_scan_result_t = struct__cyw43_ev_scan_result_t;
const union_unnamed_13 = extern union {
    scan_result: cyw43_ev_scan_result_t,
};
pub const struct__cyw43_async_event_t = extern struct {
    _0: u16 = 0,
    flags: u16 = 0,
    event_type: u32 = 0,
    status: u32 = 0,
    reason: u32 = 0,
    _1: [30]u8 = @import("std").mem.zeroes([30]u8),
    interface: u8 = 0,
    _2: u8 = 0,
    u: union_unnamed_13 = @import("std").mem.zeroes(union_unnamed_13),
};
pub const cyw43_async_event_t = struct__cyw43_async_event_t;
pub const struct__cyw43_wifi_scan_options_t = extern struct {
    version: u32 = 0,
    action: u16 = 0,
    _: u16 = 0,
    ssid_len: u32 = 0,
    ssid: [32]u8 = @import("std").mem.zeroes([32]u8),
    bssid: [6]u8 = @import("std").mem.zeroes([6]u8),
    bss_type: i8 = 0,
    scan_type: i8 = 0,
    nprobes: i32 = 0,
    active_time: i32 = 0,
    passive_time: i32 = 0,
    home_time: i32 = 0,
    channel_num: i32 = 0,
    channel_list: [1]u16 = @import("std").mem.zeroes([1]u16),
};
pub const cyw43_wifi_scan_options_t = struct__cyw43_wifi_scan_options_t;
pub const struct__cyw43_ll_t = extern struct {
    @"opaque": [532]u32 = @import("std").mem.zeroes([532]u32),
    pub const init = cyw43_ll_init;
    pub const deinit = cyw43_ll_deinit;
    pub const init1 = cyw43_ll_bus_init;
    pub const sleep = cyw43_ll_bus_sleep;
    pub const packets = cyw43_ll_process_packets;
    pub const ioctl = cyw43_ll_ioctl;
    pub const ethernet = cyw43_ll_send_ethernet;
    pub const on = cyw43_ll_wifi_on;
    pub const pm = cyw43_ll_wifi_pm;
    pub const pm1 = cyw43_ll_wifi_get_pm;
    pub const scan = cyw43_ll_wifi_scan;
    pub const join = cyw43_ll_wifi_join;
    pub const auth = cyw43_ll_wifi_set_wpa_auth;
    pub const rejoin = cyw43_ll_wifi_rejoin;
    pub const bssid = cyw43_ll_wifi_get_bssid;
    pub const init2 = cyw43_ll_wifi_ap_init;
    pub const up = cyw43_ll_wifi_ap_set_up;
    pub const stas = cyw43_ll_wifi_ap_get_stas;
    pub const set = cyw43_ll_gpio_set;
    pub const get = cyw43_ll_gpio_get;
    pub const mac = cyw43_ll_wifi_get_mac;
    pub const filter = cyw43_ll_wifi_update_multicast_filter;
    pub const work = cyw43_ll_has_work;
    pub const work1 = cyw43_ll_bt_has_work;
    pub const reg = cyw43_ll_write_backplane_reg;
    pub const reg1 = cyw43_ll_read_backplane_reg;
    pub const mem = cyw43_ll_write_backplane_mem;
    pub const mem1 = cyw43_ll_read_backplane_mem;
};
pub const cyw43_ll_t = struct__cyw43_ll_t;
pub extern fn cyw43_ll_init(self: [*c]cyw43_ll_t, cb_data: ?*anyopaque) void;
pub extern fn cyw43_ll_deinit(self: [*c]cyw43_ll_t) void;
pub extern fn cyw43_ll_bus_init(self: [*c]cyw43_ll_t, mac: [*c]const u8) c_int;
pub extern fn cyw43_ll_bus_sleep(self: [*c]cyw43_ll_t, can_sleep: bool) void;
pub extern fn cyw43_ll_process_packets(self: [*c]cyw43_ll_t) void;
pub extern fn cyw43_ll_ioctl(self: [*c]cyw43_ll_t, cmd: u32, len: usize, buf: [*c]u8, iface: u32) c_int;
pub extern fn cyw43_ll_send_ethernet(self: [*c]cyw43_ll_t, itf: c_int, len: usize, buf: ?*const anyopaque, is_pbuf: bool) c_int;
pub extern fn cyw43_ll_wifi_on(self: [*c]cyw43_ll_t, country: u32) c_int;
pub extern fn cyw43_ll_wifi_pm(self: [*c]cyw43_ll_t, pm: u32, pm_sleep_ret: u32, li_bcn: u32, li_dtim: u32, li_assoc: u32) c_int;
pub extern fn cyw43_ll_wifi_get_pm(self: [*c]cyw43_ll_t, pm: [*c]u32, pm_sleep_ret: [*c]u32, li_bcn: [*c]u32, li_dtim: [*c]u32, li_assoc: [*c]u32) c_int;
pub extern fn cyw43_ll_wifi_scan(self: [*c]cyw43_ll_t, opts: [*c]cyw43_wifi_scan_options_t) c_int;
pub extern fn cyw43_ll_wifi_join(self: [*c]cyw43_ll_t, ssid_len: usize, ssid: [*c]const u8, key_len: usize, key: [*c]const u8, auth_type: u32, bssid: [*c]const u8, channel: u32) c_int;
pub extern fn cyw43_ll_wifi_set_wpa_auth(self: [*c]cyw43_ll_t) void;
pub extern fn cyw43_ll_wifi_rejoin(self: [*c]cyw43_ll_t) void;
pub extern fn cyw43_ll_wifi_get_bssid(self_in: [*c]cyw43_ll_t, bssid: [*c]u8) c_int;
pub extern fn cyw43_ll_wifi_ap_init(self: [*c]cyw43_ll_t, ssid_len: usize, ssid: [*c]const u8, auth: u32, key_len: usize, key: [*c]const u8, channel: u32) c_int;
pub extern fn cyw43_ll_wifi_ap_set_up(self: [*c]cyw43_ll_t, up: bool) c_int;
pub extern fn cyw43_ll_wifi_ap_get_stas(self: [*c]cyw43_ll_t, num_stas: [*c]c_int, macs: [*c]u8) c_int;
pub extern fn cyw43_ll_gpio_set(self: [*c]cyw43_ll_t, gpio_n: c_int, gpio_en: bool) c_int;
pub extern fn cyw43_ll_gpio_get(self_in: [*c]cyw43_ll_t, gpio_n: c_int, gpio_en: [*c]bool) c_int;
pub extern fn cyw43_ll_wifi_get_mac(self_in: [*c]cyw43_ll_t, addr: [*c]u8) c_int;
pub extern fn cyw43_ll_wifi_update_multicast_filter(self_in: [*c]cyw43_ll_t, addr: [*c]u8, add: bool) c_int;
pub extern fn cyw43_ll_has_work(self: [*c]cyw43_ll_t) bool;
pub extern fn cyw43_ll_bt_has_work(self: [*c]cyw43_ll_t) bool;
pub extern fn cyw43_cb_read_host_interrupt_pin(cb_data: ?*anyopaque) c_int;
pub extern fn cyw43_cb_ensure_awake(cb_data: ?*anyopaque) void;
pub extern fn cyw43_cb_process_async_event(cb_data: ?*anyopaque, ev: [*c]const cyw43_async_event_t) void;
pub extern fn cyw43_cb_process_ethernet(cb_data: ?*anyopaque, itf: c_int, len: usize, buf: [*c]const u8) void;
pub extern fn cyw43_ll_write_backplane_reg(self_in: [*c]cyw43_ll_t, addr: u32, val: u32) void;
pub extern fn cyw43_ll_read_backplane_reg(self_in: [*c]cyw43_ll_t, addr: u32) u32;
pub extern fn cyw43_ll_write_backplane_mem(self_in: [*c]cyw43_ll_t, addr: u32, len: u32, buf: [*c]const u8) c_int;
pub extern fn cyw43_ll_read_backplane_mem(self_in: [*c]cyw43_ll_t, addr: u32, len: u32, buf: [*c]u8) c_int;
pub extern fn bcmp(?*const anyopaque, ?*const anyopaque, usize) c_int;
pub extern fn bcopy(?*const anyopaque, ?*anyopaque, usize) void;
pub extern fn bzero(?*anyopaque, usize) void;
pub extern fn explicit_bzero(?*anyopaque, usize) void;
pub extern fn ffs(c_int) c_int;
pub extern fn ffsl(c_long) c_int;
pub extern fn ffsll(c_longlong) c_int;
pub extern fn fls(c_int) c_int;
pub extern fn flsl(c_long) c_int;
pub extern fn flsll(c_longlong) c_int;
pub extern fn index([*c]const u8, c_int) [*c]u8;
pub extern fn rindex([*c]const u8, c_int) [*c]u8;
pub extern fn strcasecmp([*c]const u8, [*c]const u8) c_int;
pub extern fn strncasecmp([*c]const u8, [*c]const u8, usize) c_int;
pub extern fn strcasecmp_l([*c]const u8, [*c]const u8, locale_t) c_int;
pub extern fn strncasecmp_l([*c]const u8, [*c]const u8, usize, locale_t) c_int;
pub extern fn memchr(?*const anyopaque, c_int, usize) ?*anyopaque;
pub extern fn memcmp(?*const anyopaque, ?*const anyopaque, usize) c_int;
pub extern fn memcpy(noalias ?*anyopaque, noalias ?*const anyopaque, usize) ?*anyopaque;
pub extern fn memmove(?*anyopaque, ?*const anyopaque, usize) ?*anyopaque;
pub extern fn memset(?*anyopaque, c_int, usize) ?*anyopaque;
pub extern fn strcat(noalias [*c]u8, noalias [*c]const u8) [*c]u8;
pub extern fn strchr([*c]const u8, c_int) [*c]u8;
pub extern fn strcmp([*c]const u8, [*c]const u8) c_int;
pub extern fn strcoll([*c]const u8, [*c]const u8) c_int;
pub extern fn strcpy(noalias [*c]u8, noalias [*c]const u8) [*c]u8;
pub extern fn strcspn([*c]const u8, [*c]const u8) usize;
pub extern fn strerror(c_int) [*c]u8;
pub extern fn strlen([*c]const u8) usize;
pub extern fn strncat(noalias [*c]u8, noalias [*c]const u8, usize) [*c]u8;
pub extern fn strncmp([*c]const u8, [*c]const u8, usize) c_int;
pub extern fn strncpy(noalias [*c]u8, noalias [*c]const u8, usize) [*c]u8;
pub extern fn strpbrk([*c]const u8, [*c]const u8) [*c]u8;
pub extern fn strrchr([*c]const u8, c_int) [*c]u8;
pub extern fn strspn([*c]const u8, [*c]const u8) usize;
pub extern fn strstr([*c]const u8, [*c]const u8) [*c]u8;
pub extern fn strtok(noalias [*c]u8, noalias [*c]const u8) [*c]u8;
pub extern fn strxfrm(noalias [*c]u8, noalias [*c]const u8, usize) usize;
pub extern fn strcoll_l([*c]const u8, [*c]const u8, locale_t) c_int;
pub extern fn strerror_l(c_int, locale_t) [*c]u8;
pub extern fn strxfrm_l(noalias [*c]u8, noalias [*c]const u8, usize, locale_t) usize;
pub extern fn strtok_r(noalias [*c]u8, noalias [*c]const u8, noalias [*c][*c]u8) [*c]u8;
pub extern fn timingsafe_bcmp(?*const anyopaque, ?*const anyopaque, usize) c_int;
pub extern fn timingsafe_memcmp(?*const anyopaque, ?*const anyopaque, usize) c_int;
pub extern fn memccpy(noalias ?*anyopaque, noalias ?*const anyopaque, c_int, usize) ?*anyopaque;
pub extern fn stpcpy(noalias [*c]u8, noalias [*c]const u8) [*c]u8;
pub extern fn stpncpy(noalias [*c]u8, noalias [*c]const u8, usize) [*c]u8;
pub extern fn strdup([*c]const u8) [*c]u8;
pub extern fn _strdup_r([*c]struct__reent, [*c]const u8) [*c]u8;
pub extern fn strndup([*c]const u8, usize) [*c]u8;
pub extern fn _strndup_r([*c]struct__reent, [*c]const u8, usize) [*c]u8;
pub extern fn strerror_r(c_int, [*c]u8, usize) c_int;
pub extern fn _strerror_r([*c]struct__reent, c_int, c_int, [*c]c_int) [*c]u8;
pub extern fn strlcat([*c]u8, [*c]const u8, usize) usize;
pub extern fn strlcpy([*c]u8, [*c]const u8, usize) usize;
pub extern fn strnlen([*c]const u8, usize) usize;
pub extern fn strsep([*c][*c]u8, [*c]const u8) [*c]u8;
pub extern fn strnstr([*c]const u8, [*c]const u8, usize) [*c]u8;
pub extern fn strlwr([*c]u8) [*c]u8;
pub extern fn strupr([*c]u8) [*c]u8;
pub extern fn strsignal(__signo: c_int) [*c]u8;
pub const struct__cyw43_t = extern struct {
    cyw43_ll: cyw43_ll_t = @import("std").mem.zeroes(cyw43_ll_t),
    itf_state: u8 = 0,
    trace_flags: u32 = 0,
    wifi_scan_state: u32 = 0,
    wifi_join_state: u32 = 0,
    wifi_scan_env: ?*anyopaque = null,
    wifi_scan_cb: ?*const fn (?*anyopaque, [*c]const cyw43_ev_scan_result_t) callconv(.c) c_int = null,
    initted: bool = false,
    pend_disassoc: bool = false,
    pend_rejoin: bool = false,
    pend_rejoin_wpa: bool = false,
    ap_auth: u32 = 0,
    ap_channel: u8 = 0,
    ap_ssid_len: u8 = 0,
    ap_key_len: u8 = 0,
    ap_ssid: [32]u8 = @import("std").mem.zeroes([32]u8),
    ap_key: [64]u8 = @import("std").mem.zeroes([64]u8),
    netif: [2]struct_netif = @import("std").mem.zeroes([2]struct_netif),
    dhcp_client: struct_dhcp = @import("std").mem.zeroes(struct_dhcp),
    mac: [6]u8 = @import("std").mem.zeroes([6]u8),
    pub const init = cyw43_init;
    pub const deinit = cyw43_deinit;
    pub const ioctl = cyw43_ioctl;
    pub const ethernet = cyw43_send_ethernet;
    pub const pm = cyw43_wifi_pm;
    pub const pm1 = cyw43_wifi_get_pm;
    pub const status = cyw43_wifi_link_status;
    pub const up = cyw43_wifi_set_up;
    pub const mac1 = cyw43_wifi_get_mac;
    pub const filter = cyw43_wifi_update_multicast_filter;
    pub const scan = cyw43_wifi_scan;
    pub const active = cyw43_wifi_scan_active;
    pub const join = cyw43_wifi_join;
    pub const leave = cyw43_wifi_leave;
    pub const rssi = cyw43_wifi_get_rssi;
    pub const bssid = cyw43_wifi_get_bssid;
    pub const ssid = cyw43_wifi_ap_get_ssid;
    pub const auth = cyw43_wifi_ap_get_auth;
    pub const channel = cyw43_wifi_ap_set_channel;
    pub const ssid1 = cyw43_wifi_ap_set_ssid;
    pub const password = cyw43_wifi_ap_set_password;
    pub const auth1 = cyw43_wifi_ap_set_auth;
    pub const stas = cyw43_wifi_ap_get_max_stas;
    pub const stas1 = cyw43_wifi_ap_get_stas;
    pub const initialized = cyw43_is_initialized;
    pub const init1 = cyw43_cb_tcpip_init;
    pub const deinit1 = cyw43_cb_tcpip_deinit;
    pub const up1 = cyw43_cb_tcpip_set_link_up;
    pub const down = cyw43_cb_tcpip_set_link_down;
    pub const status1 = cyw43_tcpip_link_status;
    pub const set = cyw43_gpio_set;
    pub const get = cyw43_gpio_get;
};
pub const cyw43_t = struct__cyw43_t;
pub extern var cyw43_state: cyw43_t;
pub extern var cyw43_poll: ?*const fn () callconv(.c) void;
pub extern var cyw43_sleep: u32;
pub extern fn cyw43_init(self: [*c]cyw43_t) void;
pub extern fn cyw43_deinit(self: [*c]cyw43_t) void;
pub extern fn cyw43_ioctl(self: [*c]cyw43_t, cmd: u32, len: usize, buf: [*c]u8, iface: u32) c_int;
pub extern fn cyw43_send_ethernet(self: [*c]cyw43_t, itf: c_int, len: usize, buf: ?*const anyopaque, is_pbuf: bool) c_int;
pub extern fn cyw43_wifi_pm(self: [*c]cyw43_t, pm: u32) c_int;
pub extern fn cyw43_wifi_get_pm(self: [*c]cyw43_t, pm: [*c]u32) c_int;
pub extern fn cyw43_wifi_link_status(self: [*c]cyw43_t, itf: c_int) c_int;
pub extern fn cyw43_wifi_set_up(self: [*c]cyw43_t, itf: c_int, up: bool, country: u32) void;
pub extern fn cyw43_wifi_get_mac(self: [*c]cyw43_t, itf: c_int, mac: [*c]u8) c_int;
pub extern fn cyw43_wifi_update_multicast_filter(self: [*c]cyw43_t, addr: [*c]u8, add: bool) c_int;
pub extern fn cyw43_wifi_scan(self: [*c]cyw43_t, opts: [*c]cyw43_wifi_scan_options_t, env: ?*anyopaque, result_cb: ?*const fn (?*anyopaque, [*c]const cyw43_ev_scan_result_t) callconv(.c) c_int) c_int;
pub fn cyw43_wifi_scan_active(arg_self: [*c]cyw43_t) callconv(.c) bool {
    var self = arg_self;
    _ = &self;
    return self.*.wifi_scan_state == @as(u32, 1);
}
pub extern fn cyw43_wifi_join(self: [*c]cyw43_t, ssid_len: usize, ssid: [*c]const u8, key_len: usize, key: [*c]const u8, auth_type: u32, bssid: [*c]const u8, channel: u32) c_int;
pub extern fn cyw43_wifi_leave(self: [*c]cyw43_t, itf: c_int) c_int;
pub extern fn cyw43_wifi_get_rssi(self: [*c]cyw43_t, rssi: [*c]i32) c_int;
pub extern fn cyw43_wifi_get_bssid(self: [*c]cyw43_t, bssid: [*c]u8) c_int;
pub fn cyw43_wifi_ap_get_ssid(arg_self: [*c]cyw43_t, arg_len: [*c]usize, arg_buf: [*c][*c]const u8) callconv(.c) void {
    var self = arg_self;
    _ = &self;
    var len = arg_len;
    _ = &len;
    var buf = arg_buf;
    _ = &buf;
    len.* = self.*.ap_ssid_len;
    buf.* = @ptrCast(@alignCast(&self.*.ap_ssid));
}
pub fn cyw43_wifi_ap_get_auth(arg_self: [*c]cyw43_t) callconv(.c) u32 {
    var self = arg_self;
    _ = &self;
    return self.*.ap_auth;
}
pub fn cyw43_wifi_ap_set_channel(arg_self: [*c]cyw43_t, arg_channel: u32) callconv(.c) void {
    var self = arg_self;
    _ = &self;
    var channel = arg_channel;
    _ = &channel;
    self.*.ap_channel = @truncate(channel);
}
pub fn cyw43_wifi_ap_set_ssid(arg_self: [*c]cyw43_t, arg_len: usize, arg_buf: [*c]const u8) callconv(.c) void {
    var self = arg_self;
    _ = &self;
    var len = arg_len;
    _ = &len;
    var buf = arg_buf;
    _ = &buf;
    self.*.ap_ssid_len = @truncate(if (@sizeOf(@TypeOf(self.*.ap_ssid)) > len) len else @sizeOf(@TypeOf(self.*.ap_ssid)));
    _ = memcpy(@ptrCast(@alignCast(@as([*c]u8, @ptrCast(@alignCast(&self.*.ap_ssid))))), @ptrCast(@alignCast(buf)), self.*.ap_ssid_len);
}
pub fn cyw43_wifi_ap_set_password(arg_self: [*c]cyw43_t, arg_len: usize, arg_buf: [*c]const u8) callconv(.c) void {
    var self = arg_self;
    _ = &self;
    var len = arg_len;
    _ = &len;
    var buf = arg_buf;
    _ = &buf;
    self.*.ap_key_len = @truncate(if (@sizeOf(@TypeOf(self.*.ap_key)) > len) len else @sizeOf(@TypeOf(self.*.ap_key)));
    _ = memcpy(@ptrCast(@alignCast(@as([*c]u8, @ptrCast(@alignCast(&self.*.ap_key))))), @ptrCast(@alignCast(buf)), self.*.ap_key_len);
}
pub fn cyw43_wifi_ap_set_auth(arg_self: [*c]cyw43_t, arg_auth: u32) callconv(.c) void {
    var self = arg_self;
    _ = &self;
    var auth = arg_auth;
    _ = &auth;
    self.*.ap_auth = auth;
}
pub extern fn cyw43_wifi_ap_get_max_stas(self: [*c]cyw43_t, max_stas: [*c]c_int) void;
pub extern fn cyw43_wifi_ap_get_stas(self: [*c]cyw43_t, num_stas: [*c]c_int, macs: [*c]u8) void;
pub fn cyw43_is_initialized(arg_self: [*c]cyw43_t) callconv(.c) bool {
    var self = arg_self;
    _ = &self;
    return self.*.initted;
}
pub extern fn cyw43_cb_tcpip_init(self: [*c]cyw43_t, itf: c_int) void;
pub extern fn cyw43_cb_tcpip_deinit(self: [*c]cyw43_t, itf: c_int) void;
pub extern fn cyw43_cb_tcpip_set_link_up(self: [*c]cyw43_t, itf: c_int) void;
pub extern fn cyw43_cb_tcpip_set_link_down(self: [*c]cyw43_t, itf: c_int) void;
pub extern fn cyw43_tcpip_link_status(self: [*c]cyw43_t, itf: c_int) c_int;
pub extern fn cyw43_gpio_set(self: [*c]cyw43_t, gpio: c_int, val: bool) c_int;
pub extern fn cyw43_gpio_get(self: [*c]cyw43_t, gpio: c_int, val: [*c]bool) c_int;
pub fn cyw43_pm_value(arg_pm_mode: u8, arg_pm2_sleep_ret_ms: u16, arg_li_beacon_period: u8, arg_li_dtim_period: u8, arg_li_assoc: u8) callconv(.c) u32 {
    var pm_mode = arg_pm_mode;
    _ = &pm_mode;
    var pm2_sleep_ret_ms = arg_pm2_sleep_ret_ms;
    _ = &pm2_sleep_ret_ms;
    var li_beacon_period = arg_li_beacon_period;
    _ = &li_beacon_period;
    var li_dtim_period = arg_li_dtim_period;
    _ = &li_dtim_period;
    var li_assoc = arg_li_assoc;
    _ = &li_assoc;
    return @bitCast(@as(c_int, ((((@as(c_int, li_assoc) << @intCast(20)) | (@as(c_int, li_dtim_period) << @intCast(16))) | (@as(c_int, li_beacon_period) << @intCast(12))) | (@divTrunc(@as(c_int, pm2_sleep_ret_ms), @as(c_int, 10)) << @intCast(4))) | @as(c_int, pm_mode)));
}
pub const ASYNC_CONTEXT_POLL: c_int = 1;
pub const ASYNC_CONTEXT_THREADSAFE_BACKGROUND: c_int = 2;
pub const ASYNC_CONTEXT_FREERTOS: c_int = 3;
const enum_unnamed_14 = c_uint;
pub const struct_async_work_on_timeout = extern struct {
    next: [*c]struct_async_work_on_timeout = null,
    do_work: ?*const fn (context: [*c]async_context_t, timeout: [*c]struct_async_work_on_timeout) callconv(.c) void = null,
    next_time: absolute_time_t = 0,
    user_data: ?*anyopaque = null,
};
pub const async_at_time_worker_t = struct_async_work_on_timeout;
pub const struct_async_when_pending_worker = extern struct {
    next: [*c]struct_async_when_pending_worker = null,
    do_work: ?*const fn (context: [*c]async_context_t, worker: [*c]struct_async_when_pending_worker) callconv(.c) void = null,
    work_pending: bool = false,
    user_data: ?*anyopaque = null,
};
pub const async_when_pending_worker_t = struct_async_when_pending_worker;
pub const struct_async_context_type = extern struct {
    type: u16 = 0,
    acquire_lock_blocking: ?*const fn (self: [*c]async_context_t) callconv(.c) void = null,
    release_lock: ?*const fn (self: [*c]async_context_t) callconv(.c) void = null,
    lock_check: ?*const fn (self: [*c]async_context_t) callconv(.c) void = null,
    execute_sync: ?*const fn (context: [*c]async_context_t, func: ?*const fn (param: ?*anyopaque) callconv(.c) u32, param: ?*anyopaque) callconv(.c) u32 = null,
    add_at_time_worker: ?*const fn (self: [*c]async_context_t, worker: [*c]async_at_time_worker_t) callconv(.c) bool = null,
    remove_at_time_worker: ?*const fn (self: [*c]async_context_t, worker: [*c]async_at_time_worker_t) callconv(.c) bool = null,
    add_when_pending_worker: ?*const fn (self: [*c]async_context_t, worker: [*c]async_when_pending_worker_t) callconv(.c) bool = null,
    remove_when_pending_worker: ?*const fn (self: [*c]async_context_t, worker: [*c]async_when_pending_worker_t) callconv(.c) bool = null,
    set_work_pending: ?*const fn (self: [*c]async_context_t, worker: [*c]async_when_pending_worker_t) callconv(.c) void = null,
    poll: ?*const fn (self: [*c]async_context_t) callconv(.c) void = null,
    wait_until: ?*const fn (self: [*c]async_context_t, until: absolute_time_t) callconv(.c) void = null,
    wait_for_work_until: ?*const fn (self: [*c]async_context_t, until: absolute_time_t) callconv(.c) void = null,
    deinit: ?*const fn (self: [*c]async_context_t) callconv(.c) void = null,
};
pub const async_context_type_t = struct_async_context_type;
pub const struct_async_context = extern struct {
    type: [*c]const async_context_type_t = null,
    when_pending_list: [*c]async_when_pending_worker_t = null,
    at_time_list: [*c]async_at_time_worker_t = null,
    next_time: absolute_time_t = 0,
    flags: u16 = 0,
    core_num: u8 = 0,
    pub const blocking = async_context_acquire_lock_blocking;
    pub const lock = async_context_release_lock;
    pub const check = async_context_lock_check;
    pub const sync = async_context_execute_sync;
    pub const worker = async_context_add_at_time_worker;
    pub const at = async_context_add_at_time_worker_at;
    pub const ms = async_context_add_at_time_worker_in_ms;
    pub const worker1 = async_context_remove_at_time_worker;
    pub const worker2 = async_context_add_when_pending_worker;
    pub const worker3 = async_context_remove_when_pending_worker;
    pub const pending = async_context_set_work_pending;
    pub const poll = async_context_poll;
    pub const until = async_context_wait_until;
    pub const until1 = async_context_wait_for_work_until;
    pub const ms1 = async_context_wait_for_work_ms;
    pub const num = async_context_core_num;
    pub const deinit = async_context_deinit;
    pub const context = cyw43_arch_set_async_context;
};
pub const async_context_t = struct_async_context;
pub fn async_context_acquire_lock_blocking(arg_context: [*c]async_context_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    context.*.type.*.acquire_lock_blocking.?(context);
}
pub fn async_context_release_lock(arg_context: [*c]async_context_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    context.*.type.*.release_lock.?(context);
}
pub fn async_context_lock_check(arg_context: [*c]async_context_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    context.*.type.*.lock_check.?(context);
}
pub fn async_context_execute_sync(arg_context: [*c]async_context_t, arg_func: ?*const fn (param: ?*anyopaque) callconv(.c) u32, arg_param: ?*anyopaque) callconv(.c) u32 {
    var context = arg_context;
    _ = &context;
    var func = arg_func;
    _ = &func;
    var param = arg_param;
    _ = &param;
    return context.*.type.*.execute_sync.?(context, func, param);
}
pub fn async_context_add_at_time_worker(arg_context: [*c]async_context_t, arg_worker: [*c]async_at_time_worker_t) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    return context.*.type.*.add_at_time_worker.?(context, worker);
}
pub fn async_context_add_at_time_worker_at(arg_context: [*c]async_context_t, arg_worker: [*c]async_at_time_worker_t, arg_at: absolute_time_t) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    var at = arg_at;
    _ = &at;
    worker.*.next_time = at;
    return context.*.type.*.add_at_time_worker.?(context, worker);
}
pub fn async_context_add_at_time_worker_in_ms(arg_context: [*c]async_context_t, arg_worker: [*c]async_at_time_worker_t, arg_ms: u32) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    var ms = arg_ms;
    _ = &ms;
    worker.*.next_time = make_timeout_time_ms(ms);
    return context.*.type.*.add_at_time_worker.?(context, worker);
}
pub fn async_context_remove_at_time_worker(arg_context: [*c]async_context_t, arg_worker: [*c]async_at_time_worker_t) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    return context.*.type.*.remove_at_time_worker.?(context, worker);
}
pub fn async_context_add_when_pending_worker(arg_context: [*c]async_context_t, arg_worker: [*c]async_when_pending_worker_t) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    return context.*.type.*.add_when_pending_worker.?(context, worker);
}
pub fn async_context_remove_when_pending_worker(arg_context: [*c]async_context_t, arg_worker: [*c]async_when_pending_worker_t) callconv(.c) bool {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    return context.*.type.*.remove_when_pending_worker.?(context, worker);
}
pub fn async_context_set_work_pending(arg_context: [*c]async_context_t, arg_worker: [*c]async_when_pending_worker_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    var worker = arg_worker;
    _ = &worker;
    context.*.type.*.set_work_pending.?(context, worker);
}
pub fn async_context_poll(arg_context: [*c]async_context_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    if (context.*.type.*.poll != null) {
        context.*.type.*.poll.?(context);
    }
}
pub fn async_context_wait_until(arg_context: [*c]async_context_t, arg_until: absolute_time_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    var until = arg_until;
    _ = &until;
    context.*.type.*.wait_until.?(context, until);
}
pub fn async_context_wait_for_work_until(arg_context: [*c]async_context_t, arg_until: absolute_time_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    var until = arg_until;
    _ = &until;
    context.*.type.*.wait_for_work_until.?(context, until);
}
pub fn async_context_wait_for_work_ms(arg_context: [*c]async_context_t, arg_ms: u32) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    var ms = arg_ms;
    _ = &ms;
    async_context_wait_for_work_until(context, make_timeout_time_ms(ms));
}
pub fn async_context_core_num(arg_context: [*c]const async_context_t) callconv(.c) uint {
    var context = arg_context;
    _ = &context;
    return context.*.core_num;
}
pub fn async_context_deinit(arg_context: [*c]async_context_t) callconv(.c) void {
    var context = arg_context;
    _ = &context;
    context.*.type.*.deinit.?(context);
}
pub extern fn cyw43_arch_init() c_int;
pub extern fn cyw43_arch_init_with_country(country: u32) c_int;
pub extern fn cyw43_arch_deinit() void;
pub extern fn cyw43_arch_async_context() [*c]async_context_t;
pub extern fn cyw43_arch_set_async_context(context: [*c]async_context_t) void;
pub extern fn cyw43_arch_init_default_async_context() [*c]async_context_t;
pub extern fn cyw43_arch_poll() void;
pub extern fn cyw43_arch_wait_for_work_until(until: absolute_time_t) void;
pub fn cyw43_arch_lwip_begin() callconv(.c) void {
    cyw43_thread_enter();
}
pub fn cyw43_arch_lwip_end() callconv(.c) void {
    cyw43_thread_exit();
}
pub fn cyw43_arch_lwip_protect(arg_func: ?*const fn (param: ?*anyopaque) callconv(.c) c_int, arg_param: ?*anyopaque) callconv(.c) c_int {
    var func = arg_func;
    _ = &func;
    var param = arg_param;
    _ = &param;
    cyw43_arch_lwip_begin();
    var rc: c_int = func.?(param);
    _ = &rc;
    cyw43_arch_lwip_end();
    return rc;
}
pub extern fn cyw43_arch_get_country_code() u32;
pub extern fn cyw43_arch_enable_sta_mode() void;
pub extern fn cyw43_arch_disable_sta_mode() void;
pub extern fn cyw43_arch_enable_ap_mode(ssid: [*c]const u8, password: [*c]const u8, auth: u32) void;
pub extern fn cyw43_arch_disable_ap_mode() void;
pub extern fn cyw43_arch_wifi_connect_blocking(ssid: [*c]const u8, pw: [*c]const u8, auth: u32) c_int;
pub extern fn cyw43_arch_wifi_connect_bssid_blocking(ssid: [*c]const u8, bssid: [*c]const u8, pw: [*c]const u8, auth: u32) c_int;
pub extern fn cyw43_arch_wifi_connect_timeout_ms(ssid: [*c]const u8, pw: [*c]const u8, auth: u32, timeout: u32) c_int;
pub extern fn cyw43_arch_wifi_connect_bssid_timeout_ms(ssid: [*c]const u8, bssid: [*c]const u8, pw: [*c]const u8, auth: u32, timeout: u32) c_int;
pub extern fn cyw43_arch_wifi_connect_async(ssid: [*c]const u8, pw: [*c]const u8, auth: u32) c_int;
pub extern fn cyw43_arch_wifi_connect_bssid_async(ssid: [*c]const u8, bssid: [*c]const u8, pw: [*c]const u8, auth: u32) c_int;
pub extern fn cyw43_arch_gpio_put(wl_gpio: uint, value: bool) void;
pub extern fn cyw43_arch_gpio_get(wl_gpio: uint) bool;
pub const struct_stdio_driver = opaque {
    pub const enabled = stdio_set_driver_enabled;
    pub const driver = stdio_filter_driver;
    pub const crlf = stdio_set_translate_crlf;
};
pub const stdio_driver_t = struct_stdio_driver;
pub extern fn stdio_init_all() bool;
pub extern fn stdio_deinit_all() bool;
pub extern fn stdio_flush() void;
pub extern fn stdio_getchar_timeout_us(timeout_us: u32) c_int;
pub fn getchar_timeout_us(arg_timeout_us: u32) callconv(.c) c_int {
    var timeout_us = arg_timeout_us;
    _ = &timeout_us;
    return stdio_getchar_timeout_us(timeout_us);
}
pub extern fn stdio_set_driver_enabled(driver: ?*stdio_driver_t, enabled: bool) void;
pub extern fn stdio_filter_driver(driver: ?*stdio_driver_t) void;
pub extern fn stdio_set_translate_crlf(driver: ?*stdio_driver_t, translate: bool) void;
pub extern fn stdio_putchar_raw(c: c_int) c_int;
pub fn putchar_raw(arg_c: c_int) callconv(.c) c_int {
    var c = arg_c;
    _ = &c;
    return stdio_putchar_raw(c);
}
pub extern fn stdio_puts_raw(s: [*c]const u8) c_int;
pub fn puts_raw(arg_s: [*c]const u8) callconv(.c) c_int {
    var s = arg_s;
    _ = &s;
    return stdio_puts_raw(s);
}
pub extern fn stdio_set_chars_available_callback(@"fn": ?*const fn (?*anyopaque) callconv(.c) void, param: ?*anyopaque) void;
pub extern fn stdio_get_until(buf: [*c]u8, len: c_int, until: absolute_time_t) c_int;
pub extern fn stdio_put_string(s: [*c]const u8, len: c_int, newline: bool, cr_translation: bool) c_int;
pub extern fn stdio_getchar() c_int;
pub extern fn stdio_putchar(c_int) c_int;
pub extern fn stdio_puts(s: [*c]const u8) c_int;
pub extern fn stdio_vprintf(format: [*c]const u8, va: [*c]struct___va_list_tag_3) c_int;
pub extern fn stdio_printf(format: [*c]const u8, ...) c_int;
pub const uart_hw_t = extern struct {
    dr: io_rw_32 = 0,
    rsr: io_rw_32 = 0,
    _pad0: [4]u32 = @import("std").mem.zeroes([4]u32),
    fr: io_ro_32 = 0,
    _pad1: u32 = 0,
    ilpr: io_rw_32 = 0,
    ibrd: io_rw_32 = 0,
    fbrd: io_rw_32 = 0,
    lcr_h: io_rw_32 = 0,
    cr: io_rw_32 = 0,
    ifls: io_rw_32 = 0,
    imsc: io_rw_32 = 0,
    ris: io_ro_32 = 0,
    mis: io_ro_32 = 0,
    icr: io_rw_32 = 0,
    dmacr: io_rw_32 = 0,
};
comptime {
    if (!(@sizeOf(uart_hw_t) == @as(c_uint, 76))) @compileError("static assertion failed \"\"");
}
pub const struct_uart_inst = opaque {
    pub const index = uart_get_index;
    pub const hw = uart_get_hw;
    pub const init = uart_init;
    pub const deinit = uart_deinit;
    pub const baudrate = uart_set_baudrate;
    pub const flow = uart_set_hw_flow;
    pub const format = uart_set_format;
    pub const enabled = uart_set_irqs_enabled;
    pub const enables = uart_set_irq_enables;
    pub const enabled1 = uart_is_enabled;
    pub const enabled2 = uart_set_fifo_enabled;
    pub const writable = uart_is_writable;
    pub const blocking = uart_tx_wait_blocking;
    pub const readable = uart_is_readable;
    pub const blocking1 = uart_write_blocking;
    pub const blocking2 = uart_read_blocking;
    pub const raw = uart_putc_raw;
    pub const putc = uart_putc;
    pub const puts = uart_puts;
    pub const getc = uart_getc;
    pub const @"break" = uart_set_break;
    pub const crlf = uart_set_translate_crlf;
    pub const us = uart_is_readable_within_us;
    pub const num = uart_get_dreq_num;
    pub const num1 = uart_get_reset_num;
    pub const dreq = uart_get_dreq;
};
pub const uart_inst_t = struct_uart_inst;
comptime {
    if (!(@as(c_uint, 2) == @as(c_uint, 2))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(@as(c_uint, 2) == @as(c_uint, 2))) @compileError("static assertion failed \"\"");
}
pub const DREQ_PIO0_TX0: c_int = 0;
pub const DREQ_PIO0_TX1: c_int = 1;
pub const DREQ_PIO0_TX2: c_int = 2;
pub const DREQ_PIO0_TX3: c_int = 3;
pub const DREQ_PIO0_RX0: c_int = 4;
pub const DREQ_PIO0_RX1: c_int = 5;
pub const DREQ_PIO0_RX2: c_int = 6;
pub const DREQ_PIO0_RX3: c_int = 7;
pub const DREQ_PIO1_TX0: c_int = 8;
pub const DREQ_PIO1_TX1: c_int = 9;
pub const DREQ_PIO1_TX2: c_int = 10;
pub const DREQ_PIO1_TX3: c_int = 11;
pub const DREQ_PIO1_RX0: c_int = 12;
pub const DREQ_PIO1_RX1: c_int = 13;
pub const DREQ_PIO1_RX2: c_int = 14;
pub const DREQ_PIO1_RX3: c_int = 15;
pub const DREQ_PIO2_TX0: c_int = 16;
pub const DREQ_PIO2_TX1: c_int = 17;
pub const DREQ_PIO2_TX2: c_int = 18;
pub const DREQ_PIO2_TX3: c_int = 19;
pub const DREQ_PIO2_RX0: c_int = 20;
pub const DREQ_PIO2_RX1: c_int = 21;
pub const DREQ_PIO2_RX2: c_int = 22;
pub const DREQ_PIO2_RX3: c_int = 23;
pub const DREQ_SPI0_TX: c_int = 24;
pub const DREQ_SPI0_RX: c_int = 25;
pub const DREQ_SPI1_TX: c_int = 26;
pub const DREQ_SPI1_RX: c_int = 27;
pub const DREQ_UART0_TX: c_int = 28;
pub const DREQ_UART0_RX: c_int = 29;
pub const DREQ_UART1_TX: c_int = 30;
pub const DREQ_UART1_RX: c_int = 31;
pub const DREQ_PWM_WRAP0: c_int = 32;
pub const DREQ_PWM_WRAP1: c_int = 33;
pub const DREQ_PWM_WRAP2: c_int = 34;
pub const DREQ_PWM_WRAP3: c_int = 35;
pub const DREQ_PWM_WRAP4: c_int = 36;
pub const DREQ_PWM_WRAP5: c_int = 37;
pub const DREQ_PWM_WRAP6: c_int = 38;
pub const DREQ_PWM_WRAP7: c_int = 39;
pub const DREQ_PWM_WRAP8: c_int = 40;
pub const DREQ_PWM_WRAP9: c_int = 41;
pub const DREQ_PWM_WRAP10: c_int = 42;
pub const DREQ_PWM_WRAP11: c_int = 43;
pub const DREQ_I2C0_TX: c_int = 44;
pub const DREQ_I2C0_RX: c_int = 45;
pub const DREQ_I2C1_TX: c_int = 46;
pub const DREQ_I2C1_RX: c_int = 47;
pub const DREQ_ADC: c_int = 48;
pub const DREQ_XIP_STREAM: c_int = 49;
pub const DREQ_XIP_QMITX: c_int = 50;
pub const DREQ_XIP_QMIRX: c_int = 51;
pub const DREQ_HSTX: c_int = 52;
pub const DREQ_CORESIGHT: c_int = 53;
pub const DREQ_SHA256: c_int = 54;
pub const DREQ_DMA_TIMER0: c_int = 59;
pub const DREQ_DMA_TIMER1: c_int = 60;
pub const DREQ_DMA_TIMER2: c_int = 61;
pub const DREQ_DMA_TIMER3: c_int = 62;
pub const DREQ_FORCE: c_int = 63;
pub const DREQ_COUNT: c_int = 64;
pub const enum_dreq_num_rp2350 = c_uint;
pub const dreq_num_t = enum_dreq_num_rp2350;
comptime {
    if (!(DREQ_UART0_RX == (DREQ_UART0_TX + @as(c_int, 1)))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(DREQ_UART1_RX == (DREQ_UART1_TX + @as(c_int, 1)))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(DREQ_UART1_TX == (DREQ_UART0_TX + @as(c_int, 2)))) @compileError("static assertion failed \"\"");
}
comptime {
    if (!(UART1_IRQ == (UART0_IRQ + @as(c_int, 1)))) @compileError("static assertion failed \"\"");
}
pub const enum_reset_num_rp2350 = c_uint;
pub const reset_num_t = enum_reset_num_rp2350;
pub const resets_hw_t = extern struct {
    reset: io_rw_32 = 0,
    wdsel: io_rw_32 = 0,
    reset_done: io_ro_32 = 0,
};
comptime {
    if (!(@sizeOf(resets_hw_t) == @as(c_uint, 12))) @compileError("static assertion failed \"\"");
}
pub inline fn reset_block_reg_mask(arg_reset: [*c]volatile io_rw_32, arg_mask: u32) void {
    var reset = arg_reset;
    _ = &reset;
    var mask = arg_mask;
    _ = &mask;
    hw_set_bits(reset, mask);
}
pub inline fn unreset_block_reg_mask(arg_reset: [*c]volatile io_rw_32, arg_mask: u32) void {
    var reset = arg_reset;
    _ = &reset;
    var mask = arg_mask;
    _ = &mask;
    hw_clear_bits(reset, mask);
}
pub inline fn unreset_block_reg_mask_wait_blocking(arg_reset: [*c]volatile io_rw_32, arg_reset_done: [*c]const volatile io_ro_32, arg_mask: u32) void {
    var reset = arg_reset;
    _ = &reset;
    var reset_done = arg_reset_done;
    _ = &reset_done;
    var mask = arg_mask;
    _ = &mask;
    hw_clear_bits(reset, mask);
    while ((~reset_done.* & mask) != 0) {
        tight_loop_contents();
    }
}
pub inline fn reset_block_mask(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    reset_block_reg_mask(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, bits);
}
pub inline fn unreset_block_mask(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    unreset_block_reg_mask(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, bits);
}
pub inline fn unreset_block_mask_wait_blocking(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    unreset_block_reg_mask_wait_blocking(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, &@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset_done, bits);
}
pub inline fn reset_block(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    reset_block_mask(bits);
}
pub inline fn unreset_block(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    unreset_block_mask(bits);
}
pub inline fn unreset_block_wait(arg_bits: u32) void {
    var bits = arg_bits;
    _ = &bits;
    unreset_block_mask_wait_blocking(bits);
}
pub fn reset_block_num(arg_block_num: u32) callconv(.c) void {
    var block_num = arg_block_num;
    _ = &block_num;
    reset_block_reg_mask(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, @as(c_uint, 1) << @intCast(block_num));
}
pub fn unreset_block_num(arg_block_num: uint) callconv(.c) void {
    var block_num = arg_block_num;
    _ = &block_num;
    const static_local___func__ = struct {
        const __func__: [17:0]u8 = "unreset_block_num".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(block_num > @as(c_uint, 28))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_resets/include/hardware/resets.h", 187, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( block_num > 28u)");
        }
    }
    unreset_block_reg_mask(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, @as(c_uint, 1) << @intCast(block_num));
}
pub fn unreset_block_num_wait_blocking(arg_block_num: uint) callconv(.c) void {
    var block_num = arg_block_num;
    _ = &block_num;
    const static_local___func__ = struct {
        const __func__: [31:0]u8 = "unreset_block_num_wait_blocking".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(block_num > @as(c_uint, 28))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_resets/include/hardware/resets.h", 197, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( block_num > 28u)");
        }
    }
    unreset_block_reg_mask_wait_blocking(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, &@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset_done, @as(c_uint, 1) << @intCast(block_num));
}
pub fn reset_unreset_block_num_wait_blocking(arg_block_num: uint) callconv(.c) void {
    var block_num = arg_block_num;
    _ = &block_num;
    const static_local___func__ = struct {
        const __func__: [37:0]u8 = "reset_unreset_block_num_wait_blocking".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(block_num > @as(c_uint, 28))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_resets/include/hardware/resets.h", 207, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( block_num > 28u)");
        }
    }
    reset_block_reg_mask(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, @as(c_uint, 1) << @intCast(block_num));
    unreset_block_reg_mask_wait_blocking(&@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset, &@as([*c]resets_hw_t, @ptrFromInt(@as(c_uint, 1073872896))).*.reset_done, @as(c_uint, 1) << @intCast(block_num));
}
pub fn uart_get_index(arg_uart: ?*uart_inst_t) callconv(.c) uint {
    var uart = arg_uart;
    _ = &uart;
    const static_local___func__ = struct {
        const __func__: [14:0]u8 = "uart_get_index".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!((uart != @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074200576))))))) and (uart != @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074233344))))))))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h", 219, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( uart != ((uart_inst_t *)((uart_hw_t *)0x40070000u)) && uart != ((uart_inst_t *)((uart_hw_t *)0x40078000u)))");
        }
    }
    return @bitCast(@as(c_int, @intFromBool(uart == @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074233344)))))))));
}
pub fn uart_get_instance(arg_num: uint) callconv(.c) ?*uart_inst_t {
    var num = arg_num;
    _ = &num;
    const static_local___func__ = struct {
        const __func__: [17:0]u8 = "uart_get_instance".*;
    };
    _ = &static_local___func__;
    {
        if ((false or false) and !false) {
            if (!(num >= @as(c_uint, 2))) _ = @as(c_int, 0) else __assert_func("/home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h", 230, @ptrCast(@alignCast(&static_local___func__.__func__)), "!( num >= 2u)");
        }
    }
    return if (num != 0) @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074233344)))))) else @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074200576))))));
}
pub fn uart_get_hw(arg_uart: ?*uart_inst_t) callconv(.c) [*c]uart_hw_t {
    var uart = arg_uart;
    _ = &uart;
    _ = uart_get_index(uart);
    return @ptrCast(@alignCast(uart));
}
pub const uart_parity_t = c_uint;
pub extern fn uart_init(uart: ?*uart_inst_t, baudrate: uint) uint;
pub extern fn uart_deinit(uart: ?*uart_inst_t) void;
pub extern fn uart_set_baudrate(uart: ?*uart_inst_t, baudrate: uint) uint;
pub fn uart_set_hw_flow(arg_uart: ?*uart_inst_t, arg_cts: bool, arg_rts: bool) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var cts = arg_cts;
    _ = &cts;
    var rts = arg_rts;
    _ = &rts;
    hw_write_masked(&uart_get_hw(uart).*.cr, (@as(uint, @bitCast(@as(c_int, @intFromBool(!!cts)))) << @intCast(15)) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!rts)))) << @intCast(14)), @as(c_uint, 16384) | @as(c_uint, 32768));
}
pub extern fn uart_set_format(uart: ?*uart_inst_t, data_bits: uint, stop_bits: uint, parity: uart_parity_t) void;
pub fn uart_set_irqs_enabled(arg_uart: ?*uart_inst_t, arg_rx_has_data: bool, arg_tx_needs_data: bool) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var rx_has_data = arg_rx_has_data;
    _ = &rx_has_data;
    var tx_needs_data = arg_tx_needs_data;
    _ = &tx_needs_data;
    uart_get_hw(uart).*.imsc = ((@as(uint, @bitCast(@as(c_int, @intFromBool(!!tx_needs_data)))) << @intCast(5)) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!rx_has_data)))) << @intCast(4))) | (@as(uint, @bitCast(@as(c_int, @intFromBool(!!rx_has_data)))) << @intCast(6));
    if (rx_has_data) {
        hw_write_masked(&uart_get_hw(uart).*.ifls, @as(c_uint, 0) << @intCast(3), 56);
    }
    if (tx_needs_data) {
        hw_write_masked(&uart_get_hw(uart).*.ifls, @as(c_uint, 0) << @intCast(0), 7);
    }
}
pub fn uart_set_irq_enables(arg_uart: ?*uart_inst_t, arg_rx_has_data: bool, arg_tx_needs_data: bool) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var rx_has_data = arg_rx_has_data;
    _ = &rx_has_data;
    var tx_needs_data = arg_tx_needs_data;
    _ = &tx_needs_data;
    uart_set_irqs_enabled(uart, rx_has_data, tx_needs_data);
}
pub fn uart_is_enabled(arg_uart: ?*uart_inst_t) callconv(.c) bool {
    var uart = arg_uart;
    _ = &uart;
    return (uart_get_hw(uart).*.cr & @as(c_uint, 1)) != 0;
}
pub extern fn uart_set_fifo_enabled(uart: ?*uart_inst_t, enabled: bool) void;
pub fn uart_is_writable(arg_uart: ?*uart_inst_t) callconv(.c) bool {
    var uart = arg_uart;
    _ = &uart;
    return !((uart_get_hw(uart).*.fr & @as(c_uint, 32)) != 0);
}
pub fn uart_tx_wait_blocking(arg_uart: ?*uart_inst_t) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    while ((uart_get_hw(uart).*.fr & @as(c_uint, 8)) != 0) {
        tight_loop_contents();
    }
}
pub fn uart_is_readable(arg_uart: ?*uart_inst_t) callconv(.c) bool {
    var uart = arg_uart;
    _ = &uart;
    return !((uart_get_hw(uart).*.fr & @as(c_uint, 16)) != 0);
}
pub fn uart_write_blocking(arg_uart: ?*uart_inst_t, arg_src: [*c]const u8, arg_len: usize) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var src = arg_src;
    _ = &src;
    var len = arg_len;
    _ = &len;
    {
        var i: usize = 0;
        _ = &i;
        while (i < len) : (i +%= 1) {
            while (!uart_is_writable(uart)) {
                tight_loop_contents();
            }
            uart_get_hw(uart).*.dr = (blk: {
                const ref = &src;
                const tmp = ref.*;
                ref.* += 1;
                break :blk tmp;
            }).*;
        }
    }
}
pub fn uart_read_blocking(arg_uart: ?*uart_inst_t, arg_dst: [*c]u8, arg_len: usize) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var dst = arg_dst;
    _ = &dst;
    var len = arg_len;
    _ = &len;
    {
        var i: usize = 0;
        _ = &i;
        while (i < len) : (i +%= 1) {
            while (!uart_is_readable(uart)) {
                tight_loop_contents();
            }
            (blk: {
                const ref = &dst;
                const tmp = ref.*;
                ref.* += 1;
                break :blk tmp;
            }).* = @truncate(uart_get_hw(uart).*.dr);
        }
    }
}
pub fn uart_putc_raw(arg_uart: ?*uart_inst_t, arg_c: u8) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var c = arg_c;
    _ = &c;
    uart_write_blocking(uart, @ptrCast(@alignCast(&c)), 1);
}
pub fn uart_putc(arg_uart: ?*uart_inst_t, arg_c: u8) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var c = arg_c;
    _ = &c;
    const extern_local_uart_char_to_line_feed = struct {
        extern var uart_char_to_line_feed: [2]c_short;
    };
    _ = &extern_local_uart_char_to_line_feed;
    if (@as(c_int, extern_local_uart_char_to_line_feed.uart_char_to_line_feed[uart_get_index(uart)]) == @as(c_int, c)) {
        uart_putc_raw(uart, '\r');
    }
    uart_putc_raw(uart, c);
}
pub fn uart_puts(arg_uart: ?*uart_inst_t, arg_s: [*c]const u8) callconv(.c) void {
    var uart = arg_uart;
    _ = &uart;
    var s = arg_s;
    _ = &s;
    var last_was_cr: bool = @as(c_int, 0) != 0;
    _ = &last_was_cr;
    while (@as(c_int, s.*) != 0) {
        if (last_was_cr) {
            uart_putc_raw(uart, s.*);
        } else {
            uart_putc(uart, s.*);
        }
        last_was_cr = @as(c_int, (blk: {
            const ref = &s;
            const tmp = ref.*;
            ref.* += 1;
            break :blk tmp;
        }).*) == @as(c_int, '\r');
    }
}
pub fn uart_getc(arg_uart: ?*uart_inst_t) callconv(.c) u8 {
    var uart = arg_uart;
    _ = &uart;
    var c: u8 = undefined;
    _ = &c;
    uart_read_blocking(uart, @ptrCast(@alignCast(&c)), 1);
    return c;
}
pub extern fn uart_set_break(uart: ?*uart_inst_t, en: bool) void;
pub extern fn uart_set_translate_crlf(uart: ?*uart_inst_t, translate: bool) void;
pub fn uart_default_tx_wait_blocking() callconv(.c) void {
    uart_tx_wait_blocking(@as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074200576)))))));
}
pub extern fn uart_is_readable_within_us(uart: ?*uart_inst_t, us: u32) bool;
pub fn uart_get_dreq_num(arg_uart: ?*uart_inst_t, arg_is_tx: bool) callconv(.c) uint {
    var uart = arg_uart;
    _ = &uart;
    var is_tx = arg_is_tx;
    _ = &is_tx;
    return @bitCast(@as(c_int, blk: {
        break :blk (DREQ_UART0_TX + (@intFromBool(uart == @as(?*uart_inst_t, @ptrCast(@alignCast(@as([*c]uart_hw_t, @ptrFromInt(@as(c_uint, 1074233344))))))) * @as(c_int, 2))) + @intFromBool(!is_tx);
    }));
}
pub fn uart_get_reset_num(arg_uart: ?*uart_inst_t) callconv(.c) uint {
    var uart = arg_uart;
    _ = &uart;
    return @bitCast(@as(c_int, if (uart_get_index(uart) != 0) RESET_UART1 else RESET_UART0));
}
pub fn uart_get_dreq(arg_uart: ?*uart_inst_t, arg_is_tx: bool) callconv(.c) uint {
    var uart = arg_uart;
    _ = &uart;
    var is_tx = arg_is_tx;
    _ = &is_tx;
    return uart_get_dreq_num(uart, is_tx);
}
pub const stdio_usb = @compileError("local variable has opaque type"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include/pico/stdio_usb.h:169:23
pub extern fn stdio_usb_init() bool;
pub extern fn stdio_usb_deinit() bool;
pub extern fn stdio_usb_connected() bool;
pub extern fn stdio_usb_call_chars_available_callback() void;
pub extern fn setup_default_uart() void;

pub const __VERSION__ = "Aro aro-zig";
pub const __Aro__ = "";
pub const __STDC__ = @as(c_int, 1);
pub const __STDC_HOSTED__ = @as(c_int, 0);
pub const __STDC_UTF_16__ = @as(c_int, 1);
pub const __STDC_UTF_32__ = @as(c_int, 1);
pub const __STDC_EMBED_NOT_FOUND__ = @as(c_int, 0);
pub const __STDC_EMBED_FOUND__ = @as(c_int, 1);
pub const __STDC_EMBED_EMPTY__ = @as(c_int, 2);
pub const __STDC_VERSION__ = @as(c_long, 201710);
pub const __GNUC__ = @as(c_int, 4);
pub const __GNUC_MINOR__ = @as(c_int, 2);
pub const __GNUC_PATCHLEVEL__ = @as(c_int, 1);
pub const __ARO_EMULATE_CLANG__ = @as(c_int, 1);
pub const __ARO_EMULATE_GCC__ = @as(c_int, 2);
pub const __ARO_EMULATE_MSVC__ = @as(c_int, 3);
pub const __ARO_EMULATE__ = __ARO_EMULATE_CLANG__;
pub const __arm__ = @as(c_int, 1);
pub const __arm = @as(c_int, 1);
pub const __thumb__ = @as(c_int, 1);
pub const _ILP32 = @as(c_int, 1);
pub const __ILP32__ = @as(c_int, 1);
pub const __ORDER_LITTLE_ENDIAN__ = @as(c_int, 1234);
pub const __ORDER_BIG_ENDIAN__ = @as(c_int, 4321);
pub const __ORDER_PDP_ENDIAN__ = @as(c_int, 3412);
pub const __BYTE_ORDER__ = __ORDER_LITTLE_ENDIAN__;
pub const __LITTLE_ENDIAN__ = @as(c_int, 1);
pub const __ELF__ = @as(c_int, 1);
pub const __ATOMIC_RELAXED = @as(c_int, 0);
pub const __ATOMIC_CONSUME = @as(c_int, 1);
pub const __ATOMIC_ACQUIRE = @as(c_int, 2);
pub const __ATOMIC_RELEASE = @as(c_int, 3);
pub const __ATOMIC_ACQ_REL = @as(c_int, 4);
pub const __ATOMIC_SEQ_CST = @as(c_int, 5);
pub const __ATOMIC_BOOL_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_CHAR_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_CHAR16_T_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_CHAR32_T_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_WCHAR_T_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_SHORT_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_INT_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_LONG_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_LLONG_LOCK_FREE = @as(c_int, 1);
pub const __ATOMIC_POINTER_LOCK_FREE = @as(c_int, 1);
pub const __CHAR_UNSIGNED__ = @as(c_int, 1);
pub const __CHAR_BIT__ = @as(c_int, 8);
pub const __BOOL_WIDTH__ = @as(c_int, 8);
pub const __SCHAR_MAX__ = @as(c_int, 127);
pub const __SCHAR_WIDTH__ = @as(c_int, 8);
pub const __SHRT_MAX__ = @as(c_int, 32767);
pub const __SHRT_WIDTH__ = @as(c_int, 16);
pub const __INT_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __INT_WIDTH__ = @as(c_int, 32);
pub const __LONG_MAX__ = @as(c_long, 2147483647);
pub const __LONG_WIDTH__ = @as(c_int, 32);
pub const __LONG_LONG_MAX__ = @as(c_longlong, 9223372036854775807);
pub const __LONG_LONG_WIDTH__ = @as(c_int, 64);
pub const __WCHAR_MAX__ = __helpers.promoteIntLiteral(c_uint, 4294967295, .decimal);
pub const __WCHAR_WIDTH__ = @as(c_int, 32);
pub const __INTMAX_MAX__ = @as(c_longlong, 9223372036854775807);
pub const __INTMAX_WIDTH__ = @as(c_int, 64);
pub const __SIZE_MAX__ = __helpers.promoteIntLiteral(c_uint, 4294967295, .decimal);
pub const __SIZE_WIDTH__ = @as(c_int, 32);
pub const __UINTMAX_MAX__ = @as(c_ulonglong, 18446744073709551615);
pub const __UINTMAX_WIDTH__ = @as(c_int, 64);
pub const __PTRDIFF_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __PTRDIFF_WIDTH__ = @as(c_int, 32);
pub const __INTPTR_MAX__ = @as(c_long, 2147483647);
pub const __INTPTR_WIDTH__ = @as(c_int, 32);
pub const __UINTPTR_MAX__ = @as(c_ulong, 4294967295);
pub const __UINTPTR_WIDTH__ = @as(c_int, 32);
pub const __SIG_ATOMIC_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __SIG_ATOMIC_WIDTH__ = @as(c_int, 32);
pub const __BITINT_MAXWIDTH__ = __helpers.promoteIntLiteral(c_int, 65535, .decimal);
pub const __SIZEOF_FLOAT__ = @as(c_int, 4);
pub const __SIZEOF_DOUBLE__ = @as(c_int, 8);
pub const __SIZEOF_LONG_DOUBLE__ = @as(c_int, 8);
pub const __SIZEOF_SHORT__ = @as(c_int, 2);
pub const __SIZEOF_INT__ = @as(c_int, 4);
pub const __SIZEOF_LONG__ = @as(c_int, 4);
pub const __SIZEOF_LONG_LONG__ = @as(c_int, 8);
pub const __SIZEOF_POINTER__ = @as(c_int, 4);
pub const __SIZEOF_PTRDIFF_T__ = @as(c_int, 4);
pub const __SIZEOF_SIZE_T__ = @as(c_int, 4);
pub const __SIZEOF_WCHAR_T__ = @as(c_int, 4);
pub const __INTPTR_TYPE__ = c_long;
pub const __UINTPTR_TYPE__ = c_ulong;
pub const __INTMAX_TYPE__ = c_longlong;
pub const __INTMAX_C_SUFFIX__ = @compileError("unable to translate macro: undefined identifier `L`"); // <builtin>:89:9
pub const __UINTMAX_TYPE__ = c_ulonglong;
pub const __UINTMAX_C_SUFFIX__ = @compileError("unable to translate macro: undefined identifier `UL`"); // <builtin>:91:9
pub const __PTRDIFF_TYPE__ = c_int;
pub const __SIZE_TYPE__ = c_uint;
pub const __WCHAR_TYPE__ = c_uint;
pub const __CHAR16_TYPE__ = c_ushort;
pub const __CHAR32_TYPE__ = c_uint;
pub const __INT8_TYPE__ = i8;
pub const __INT8_FMTd__ = "hhd";
pub const __INT8_FMTi__ = "hhi";
pub const __INT8_C_SUFFIX__ = "";
pub const __INT16_TYPE__ = c_short;
pub const __INT16_FMTd__ = "hd";
pub const __INT16_FMTi__ = "hi";
pub const __INT16_C_SUFFIX__ = "";
pub const __INT32_TYPE__ = c_int;
pub const __INT32_FMTd__ = "d";
pub const __INT32_FMTi__ = "i";
pub const __INT32_C_SUFFIX__ = "";
pub const __INT64_TYPE__ = c_longlong;
pub const __INT64_FMTd__ = "lld";
pub const __INT64_FMTi__ = "lli";
pub const __INT64_C_SUFFIX__ = @compileError("unable to translate macro: undefined identifier `LL`"); // <builtin>:112:9
pub const __UINT8_TYPE__ = u8;
pub const __UINT8_FMTo__ = "hho";
pub const __UINT8_FMTu__ = "hhu";
pub const __UINT8_FMTx__ = "hhx";
pub const __UINT8_FMTX__ = "hhX";
pub const __UINT8_C_SUFFIX__ = "";
pub const __UINT8_MAX__ = @as(c_int, 255);
pub const __INT8_MAX__ = @as(c_int, 127);
pub const __UINT16_TYPE__ = c_ushort;
pub const __UINT16_FMTo__ = "ho";
pub const __UINT16_FMTu__ = "hu";
pub const __UINT16_FMTx__ = "hx";
pub const __UINT16_FMTX__ = "hX";
pub const __UINT16_C_SUFFIX__ = "";
pub const __UINT16_MAX__ = __helpers.promoteIntLiteral(c_int, 65535, .decimal);
pub const __INT16_MAX__ = @as(c_int, 32767);
pub const __UINT32_TYPE__ = c_uint;
pub const __UINT32_FMTo__ = "o";
pub const __UINT32_FMTu__ = "u";
pub const __UINT32_FMTx__ = "x";
pub const __UINT32_FMTX__ = "X";
pub const __UINT32_C_SUFFIX__ = @compileError("unable to translate macro: undefined identifier `U`"); // <builtin>:134:9
pub const __UINT32_MAX__ = __helpers.promoteIntLiteral(c_uint, 4294967295, .decimal);
pub const __INT32_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __UINT64_TYPE__ = c_ulonglong;
pub const __UINT64_FMTo__ = "llo";
pub const __UINT64_FMTu__ = "llu";
pub const __UINT64_FMTx__ = "llx";
pub const __UINT64_FMTX__ = "llX";
pub const __UINT64_C_SUFFIX__ = @compileError("unable to translate macro: undefined identifier `ULL`"); // <builtin>:142:9
pub const __UINT64_MAX__ = @as(c_ulonglong, 18446744073709551615);
pub const __INT64_MAX__ = @as(c_longlong, 9223372036854775807);
pub const __INT_LEAST8_TYPE__ = i8;
pub const __INT_LEAST8_MAX__ = @as(c_int, 127);
pub const __INT_LEAST8_WIDTH__ = @as(c_int, 8);
pub const INT_LEAST8_FMTd__ = "hhd";
pub const INT_LEAST8_FMTi__ = "hhi";
pub const __UINT_LEAST8_TYPE__ = u8;
pub const __UINT_LEAST8_MAX__ = @as(c_int, 255);
pub const UINT_LEAST8_FMTo__ = "hho";
pub const UINT_LEAST8_FMTu__ = "hhu";
pub const UINT_LEAST8_FMTx__ = "hhx";
pub const UINT_LEAST8_FMTX__ = "hhX";
pub const __INT_FAST8_TYPE__ = i8;
pub const __INT_FAST8_MAX__ = @as(c_int, 127);
pub const __INT_FAST8_WIDTH__ = @as(c_int, 8);
pub const INT_FAST8_FMTd__ = "hhd";
pub const INT_FAST8_FMTi__ = "hhi";
pub const __UINT_FAST8_TYPE__ = u8;
pub const __UINT_FAST8_MAX__ = @as(c_int, 255);
pub const UINT_FAST8_FMTo__ = "hho";
pub const UINT_FAST8_FMTu__ = "hhu";
pub const UINT_FAST8_FMTx__ = "hhx";
pub const UINT_FAST8_FMTX__ = "hhX";
pub const __INT_LEAST16_TYPE__ = c_short;
pub const __INT_LEAST16_MAX__ = @as(c_int, 32767);
pub const __INT_LEAST16_WIDTH__ = @as(c_int, 16);
pub const INT_LEAST16_FMTd__ = "hd";
pub const INT_LEAST16_FMTi__ = "hi";
pub const __UINT_LEAST16_TYPE__ = c_ushort;
pub const __UINT_LEAST16_MAX__ = __helpers.promoteIntLiteral(c_int, 65535, .decimal);
pub const UINT_LEAST16_FMTo__ = "ho";
pub const UINT_LEAST16_FMTu__ = "hu";
pub const UINT_LEAST16_FMTx__ = "hx";
pub const UINT_LEAST16_FMTX__ = "hX";
pub const __INT_FAST16_TYPE__ = c_short;
pub const __INT_FAST16_MAX__ = @as(c_int, 32767);
pub const __INT_FAST16_WIDTH__ = @as(c_int, 16);
pub const INT_FAST16_FMTd__ = "hd";
pub const INT_FAST16_FMTi__ = "hi";
pub const __UINT_FAST16_TYPE__ = c_ushort;
pub const __UINT_FAST16_MAX__ = __helpers.promoteIntLiteral(c_int, 65535, .decimal);
pub const UINT_FAST16_FMTo__ = "ho";
pub const UINT_FAST16_FMTu__ = "hu";
pub const UINT_FAST16_FMTx__ = "hx";
pub const UINT_FAST16_FMTX__ = "hX";
pub const __INT_LEAST32_TYPE__ = c_int;
pub const __INT_LEAST32_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __INT_LEAST32_WIDTH__ = @as(c_int, 32);
pub const INT_LEAST32_FMTd__ = "d";
pub const INT_LEAST32_FMTi__ = "i";
pub const __UINT_LEAST32_TYPE__ = c_uint;
pub const __UINT_LEAST32_MAX__ = __helpers.promoteIntLiteral(c_uint, 4294967295, .decimal);
pub const UINT_LEAST32_FMTo__ = "o";
pub const UINT_LEAST32_FMTu__ = "u";
pub const UINT_LEAST32_FMTx__ = "x";
pub const UINT_LEAST32_FMTX__ = "X";
pub const __INT_FAST32_TYPE__ = c_int;
pub const __INT_FAST32_MAX__ = __helpers.promoteIntLiteral(c_int, 2147483647, .decimal);
pub const __INT_FAST32_WIDTH__ = @as(c_int, 32);
pub const INT_FAST32_FMTd__ = "d";
pub const INT_FAST32_FMTi__ = "i";
pub const __UINT_FAST32_TYPE__ = c_uint;
pub const __UINT_FAST32_MAX__ = __helpers.promoteIntLiteral(c_uint, 4294967295, .decimal);
pub const UINT_FAST32_FMTo__ = "o";
pub const UINT_FAST32_FMTu__ = "u";
pub const UINT_FAST32_FMTx__ = "x";
pub const UINT_FAST32_FMTX__ = "X";
pub const __INT_LEAST64_TYPE__ = c_longlong;
pub const __INT_LEAST64_MAX__ = @as(c_longlong, 9223372036854775807);
pub const __INT_LEAST64_WIDTH__ = @as(c_int, 64);
pub const INT_LEAST64_FMTd__ = "lld";
pub const INT_LEAST64_FMTi__ = "lli";
pub const __UINT_LEAST64_TYPE__ = c_ulonglong;
pub const __UINT_LEAST64_MAX__ = @as(c_ulonglong, 18446744073709551615);
pub const UINT_LEAST64_FMTo__ = "llo";
pub const UINT_LEAST64_FMTu__ = "llu";
pub const UINT_LEAST64_FMTx__ = "llx";
pub const UINT_LEAST64_FMTX__ = "llX";
pub const __INT_FAST64_TYPE__ = c_longlong;
pub const __INT_FAST64_MAX__ = @as(c_longlong, 9223372036854775807);
pub const __INT_FAST64_WIDTH__ = @as(c_int, 64);
pub const INT_FAST64_FMTd__ = "lld";
pub const INT_FAST64_FMTi__ = "lli";
pub const __UINT_FAST64_TYPE__ = c_ulonglong;
pub const __UINT_FAST64_MAX__ = @as(c_ulonglong, 18446744073709551615);
pub const UINT_FAST64_FMTo__ = "llo";
pub const UINT_FAST64_FMTu__ = "llu";
pub const UINT_FAST64_FMTx__ = "llx";
pub const UINT_FAST64_FMTX__ = "llX";
pub const __FLT_DENORM_MIN__ = @as(f32, 1.40129846e-45);
pub const __FLT_HAS_DENORM__ = "";
pub const __FLT_DIG__ = @as(c_int, 6);
pub const __FLT_DECIMAL_DIG__ = @as(c_int, 9);
pub const __FLT_EPSILON__ = @as(f32, 1.19209290e-7);
pub const __FLT_HAS_INFINITY__ = "";
pub const __FLT_HAS_QUIET_NAN__ = "";
pub const __FLT_MANT_DIG__ = @as(c_int, 24);
pub const __FLT_MAX_10_EXP__ = @as(c_int, 38);
pub const __FLT_MAX_EXP__ = @as(c_int, 128);
pub const __FLT_MAX__ = @as(f32, 3.40282347e+38);
pub const __FLT_MIN_10_EXP__ = -@as(c_int, 37);
pub const __FLT_MIN_EXP__ = -@as(c_int, 125);
pub const __FLT_MIN__ = @as(f32, 1.17549435e-38);
pub const __DBL_DENORM_MIN__ = @as(f64, 4.9406564584124654e-324);
pub const __DBL_HAS_DENORM__ = "";
pub const __DBL_DIG__ = @as(c_int, 15);
pub const __DBL_DECIMAL_DIG__ = @as(c_int, 17);
pub const __DBL_EPSILON__ = @as(f64, 2.2204460492503131e-16);
pub const __DBL_HAS_INFINITY__ = "";
pub const __DBL_HAS_QUIET_NAN__ = "";
pub const __DBL_MANT_DIG__ = @as(c_int, 53);
pub const __DBL_MAX_10_EXP__ = @as(c_int, 308);
pub const __DBL_MAX_EXP__ = @as(c_int, 1024);
pub const __DBL_MAX__ = @as(f64, 1.7976931348623157e+308);
pub const __DBL_MIN_10_EXP__ = -@as(c_int, 307);
pub const __DBL_MIN_EXP__ = -@as(c_int, 1021);
pub const __DBL_MIN__ = @as(f64, 2.2250738585072014e-308);
pub const __LDBL_DENORM_MIN__ = @as(c_longdouble, 4.9406564584124654e-324);
pub const __LDBL_HAS_DENORM__ = "";
pub const __LDBL_DIG__ = @as(c_int, 15);
pub const __LDBL_DECIMAL_DIG__ = @as(c_int, 17);
pub const __LDBL_EPSILON__ = @as(c_longdouble, 2.2204460492503131e-16);
pub const __LDBL_HAS_INFINITY__ = "";
pub const __LDBL_HAS_QUIET_NAN__ = "";
pub const __LDBL_MANT_DIG__ = @as(c_int, 53);
pub const __LDBL_MAX_10_EXP__ = @as(c_int, 308);
pub const __LDBL_MAX_EXP__ = @as(c_int, 1024);
pub const __LDBL_MAX__ = @as(c_longdouble, 1.7976931348623157e+308);
pub const __LDBL_MIN_10_EXP__ = -@as(c_int, 307);
pub const __LDBL_MIN_EXP__ = -@as(c_int, 1021);
pub const __LDBL_MIN__ = @as(c_longdouble, 2.2250738585072014e-308);
pub const __FLT_EVAL_METHOD__ = @as(c_int, 0);
pub const __FLT_RADIX__ = @as(c_int, 2);
pub const __DECIMAL_DIG__ = __LDBL_DECIMAL_DIG__;
pub const LIB_PICO_STDIO_USB = @as(c_int, 1);
pub const PICO_RP2350 = @as(c_int, 1);
pub const PICO_32BIT = @as(c_int, 1);
pub const PICO_ARM = @as(c_int, 1);
pub const PICO_PIO_VERSION = @as(c_int, 1);
pub const NUM_DOORBELLS = _u(@as(c_int, 8));
pub const PICO_CMSIS_DEVICE = @compileError("unable to translate macro: undefined identifier `RP2350`"); // <command line>:7:9
pub const PICO_DEFAULT_FLASH_SIZE_BYTES = "4 * 1024 * 1024";
pub const __ARM_ARCH_8M_MAIN__ = @as(c_int, 1);
pub const PICO_CYW43_ARCH_THREADSAFE_BACKGROUND = @as(c_int, 1);
pub const _HARDWARE_ADC_H = "";
pub const _PICO_H = "";
pub const __PICO_STRING = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico.h:22:9
pub inline fn __PICO_XSTRING(x: anytype) @TypeOf(__PICO_STRING(x)) {
    _ = &x;
    return __PICO_STRING(x);
}
pub const __PICO_CONCAT1 = @compileError("unable to translate C expr: unexpected token '##'"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico.h:24:9
pub const _PICO_TYPES_H = "";
pub const _PICO_ASSERT_H = "";
pub const @"bool" = bool;
pub const @"true" = @as(c_int, 1);
pub const @"false" = @as(c_int, 0);
pub const __bool_true_false_are_defined = @as(c_int, 1);
pub const _ANSIDECL_H_ = "";
pub const __NEWLIB_H__ = @as(c_int, 1);
pub const _NEWLIB_VERSION_H__ = @as(c_int, 1);
pub const _NEWLIB_VERSION = "4.5.0";
pub const __NEWLIB__ = @as(c_int, 4);
pub const __NEWLIB_MINOR__ = @as(c_int, 5);
pub const __NEWLIB_PATCHLEVEL__ = @as(c_int, 0);
pub const _ATEXIT_DYNAMIC_ALLOC = @as(c_int, 1);
pub const _FSEEK_OPTIMIZATION = @as(c_int, 1);
pub const _FVWRITE_IN_STREAMIO = @as(c_int, 1);
pub const _HAVE_CC_INHIBIT_LOOP_TO_LIBCALL = @as(c_int, 1);
pub const _HAVE_INITFINI_ARRAY = @as(c_int, 1);
pub const _HAVE_LONG_DOUBLE = @as(c_int, 1);
pub const _LDBL_EQ_DBL = @as(c_int, 1);
pub const _MB_LEN_MAX = @as(c_int, 1);
pub const _REENT_CHECK_VERIFY = @as(c_int, 1);
pub const _RETARGETABLE_LOCKING = @as(c_int, 1);
pub const _UNBUF_STREAM_OPT = @as(c_int, 1);
pub const _WANT_IO_C99_FORMATS = @as(c_int, 1);
pub const _WANT_IO_LONG_LONG = @as(c_int, 1);
pub const _WANT_REGISTER_FINI = @as(c_int, 1);
pub const _WANT_USE_GDTOA = @as(c_int, 1);
pub const _WIDE_ORIENT = @as(c_int, 1);
pub const __SYS_CONFIG_H__ = "";
pub const __IEEE_BIG_ENDIAN = "";
pub const _SUPPORTS_ERREXCEPT = "";
pub const __DOUBLE_TYPE = f64;
pub const __FLOAT_TYPE = f32;
pub const __OBSOLETE_MATH_DEFAULT = @as(c_int, 1);
pub const __OBSOLETE_MATH = __OBSOLETE_MATH_DEFAULT;
pub const _SYS_FEATURES_H = "";
pub inline fn __GNUC_PREREQ(maj: anytype, min: anytype) @TypeOf(((__GNUC__ << @as(c_int, 16)) + __GNUC_MINOR__) >= ((maj << @as(c_int, 16)) + min)) {
    _ = &maj;
    _ = &min;
    return ((__GNUC__ << @as(c_int, 16)) + __GNUC_MINOR__) >= ((maj << @as(c_int, 16)) + min);
}
pub inline fn __GNUC_PREREQ__(ma: anytype, mi: anytype) @TypeOf(__GNUC_PREREQ(ma, mi)) {
    _ = &ma;
    _ = &mi;
    return __GNUC_PREREQ(ma, mi);
}
pub const _DEFAULT_SOURCE = @as(c_int, 1);
pub const _POSIX_SOURCE = @as(c_int, 1);
pub const _POSIX_C_SOURCE = @as(c_long, 202405);
pub const _ATFILE_SOURCE = @as(c_int, 1);
pub const __ATFILE_VISIBLE = @as(c_int, 1);
pub const __BSD_VISIBLE = @as(c_int, 1);
pub const __GNU_VISIBLE = @as(c_int, 0);
pub const __ISO_C_VISIBLE = @as(c_int, 2011);
pub const __LARGEFILE_VISIBLE = @as(c_int, 0);
pub const __MISC_VISIBLE = @as(c_int, 1);
pub const __POSIX_VISIBLE = __helpers.promoteIntLiteral(c_int, 202405, .decimal);
pub const __SVID_VISIBLE = @as(c_int, 1);
pub const __XSI_VISIBLE = @as(c_int, 0);
pub const __SSP_FORTIFY_LEVEL = @as(c_int, 0);
pub const _POINTER_INT = c_long;
pub const __RAND_MAX = __helpers.promoteIntLiteral(c_int, 0x7fffffff, .hex);
pub const __EXPORT = "";
pub const __IMPORT = "";
pub const _READ_WRITE_RETURN_TYPE = c_int;
pub const _READ_WRITE_BUFSIZE_TYPE = c_int;
pub const _USE_GDTOA = "";
pub const _BEGIN_STD_C = "";
pub const _END_STD_C = "";
pub const _NOTHROW = "";
pub const _LONG_DOUBLE = @compileError("unable to translate: TODO long double"); // /usr/arm-none-eabi/include/_ansi.h:37:9
pub const _ATTRIBUTE = @compileError("unable to translate C expr: unexpected token '__attribute__'"); // /usr/arm-none-eabi/include/_ansi.h:43:9
pub const _ELIDABLE_INLINE = @compileError("unable to translate macro: undefined identifier `__always_inline__`"); // /usr/arm-none-eabi/include/_ansi.h:65:9
pub const _NOINLINE = @compileError("unable to translate macro: undefined identifier `__noinline__`"); // /usr/arm-none-eabi/include/_ansi.h:73:9
pub const _NOINLINE_STATIC = @compileError("unable to translate C expr: unexpected token 'static'"); // /usr/arm-none-eabi/include/_ansi.h:74:9
pub const assert = @compileError("unable to translate macro: undefined identifier `__FILE__`"); // /usr/arm-none-eabi/include/assert.h:16:10
pub const __ASSERT_FUNC = @compileError("unable to translate C expr: unexpected token 'an identifier'"); // /usr/arm-none-eabi/include/assert.h:26:12
pub const static_assert = @compileError("unable to translate C expr: unexpected token '_Static_assert'"); // /usr/arm-none-eabi/include/assert.h:45:10
pub const PARAM_ASSERTIONS_ENABLE_ALL = @as(c_int, 0);
pub const PARAM_ASSERTIONS_DISABLE_ALL = @as(c_int, 0);
pub const PARAM_ASSERTIONS_ENABLED = @compileError("unable to translate macro: undefined identifier `PARAM_ASSERTIONS_ENABLED_`"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/assert.h:32:9
pub const invalid_params_if = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/assert.h:34:9
pub const valid_params_if = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/assert.h:35:9
pub const hard_assert_if = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/assert.h:36:9
pub const invalid_params_if_and_return = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/assert.h:37:9
pub const hard_assert = assert;
pub const __stdint_int_c_cat = @compileError("unable to translate C expr: unexpected token '##'"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdint.h:12:9
pub inline fn __stdint_int_c(V: anytype, SUFFIX: anytype) @TypeOf(__stdint_int_c_cat(V, SUFFIX)) {
    _ = &V;
    _ = &SUFFIX;
    return __stdint_int_c_cat(V, SUFFIX);
}
pub const __stdint_uint_c = @compileError("unable to translate macro: undefined identifier `U`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdint.h:14:9
pub const INTPTR_MIN = -__INTPTR_MAX__ - @as(c_int, 1);
pub const INTPTR_MAX = __INTPTR_MAX__;
pub const UINTPTR_MAX = __UINTPTR_MAX__;
pub const PTRDIFF_MIN = -__PTRDIFF_MAX__ - @as(c_int, 1);
pub const PTRDIFF_MAX = __PTRDIFF_MAX__;
pub const SIZE_MAX = __SIZE_MAX__;
pub const INTMAX_MIN = -__INTMAX_MAX__ - @as(c_int, 1);
pub const INTMAX_MAX = __INTMAX_MAX__;
pub const UINTMAX_MAX = __UINTMAX_MAX__;
pub const __intptr_t_defined = "";
pub const _INTPTR_T = "";
pub const _UINTPTR_T = "";
pub const __int64_c_suffix = __INT64_C_SUFFIX__;
pub inline fn INT64_C(v: anytype) @TypeOf(__stdint_int_c(v, __int64_c_suffix)) {
    _ = &v;
    return __stdint_int_c(v, __int64_c_suffix);
}
pub inline fn UINT64_C(v: anytype) @TypeOf(__stdint_uint_c(v, __int64_c_suffix)) {
    _ = &v;
    return __stdint_uint_c(v, __int64_c_suffix);
}
pub const INT64_MAX = INT64_C(__helpers.promoteIntLiteral(c_int, 9223372036854775807, .decimal));
pub const INT64_MIN = -INT64_C(__helpers.promoteIntLiteral(c_int, 9223372036854775807, .decimal)) - @as(c_int, 1);
pub const UINT64_MAX = UINT64_C(__helpers.promoteIntLiteral(c_int, 18446744073709551615, .decimal));
pub const __int32_c_suffix = "";
pub inline fn INT32_C(v: anytype) @TypeOf(__stdint_int_c(v, __int32_c_suffix)) {
    _ = &v;
    return __stdint_int_c(v, __int32_c_suffix);
}
pub inline fn UINT32_C(v: anytype) @TypeOf(__stdint_uint_c(v, __int32_c_suffix)) {
    _ = &v;
    return __stdint_uint_c(v, __int32_c_suffix);
}
pub const INT32_MAX = INT32_C(__helpers.promoteIntLiteral(c_int, 2147483647, .decimal));
pub const INT32_MIN = -INT32_C(__helpers.promoteIntLiteral(c_int, 2147483647, .decimal)) - @as(c_int, 1);
pub const UINT32_MAX = UINT32_C(__helpers.promoteIntLiteral(c_int, 4294967295, .decimal));
pub const __int16_c_suffix = "";
pub inline fn INT16_C(v: anytype) @TypeOf(__stdint_int_c(v, __int16_c_suffix)) {
    _ = &v;
    return __stdint_int_c(v, __int16_c_suffix);
}
pub inline fn UINT16_C(v: anytype) @TypeOf(__stdint_uint_c(v, __int16_c_suffix)) {
    _ = &v;
    return __stdint_uint_c(v, __int16_c_suffix);
}
pub const INT16_MAX = INT16_C(@as(c_int, 32767));
pub const INT16_MIN = -INT16_C(@as(c_int, 32767)) - @as(c_int, 1);
pub const UINT16_MAX = UINT16_C(__helpers.promoteIntLiteral(c_int, 65535, .decimal));
pub const __int8_c_suffix = "";
pub inline fn INT8_C(v: anytype) @TypeOf(__stdint_int_c(v, __int8_c_suffix)) {
    _ = &v;
    return __stdint_int_c(v, __int8_c_suffix);
}
pub inline fn UINT8_C(v: anytype) @TypeOf(__stdint_uint_c(v, __int8_c_suffix)) {
    _ = &v;
    return __stdint_uint_c(v, __int8_c_suffix);
}
pub const INT8_MAX = INT8_C(@as(c_int, 127));
pub const INT8_MIN = -INT8_C(@as(c_int, 127)) - @as(c_int, 1);
pub const UINT8_MAX = UINT8_C(@as(c_int, 255));
pub const INT_LEAST8_MAX = __INT_LEAST8_MAX__;
pub const INT_LEAST8_MIN = -__INT_LEAST8_MAX__ - @as(c_int, 1);
pub const UINT_LEAST8_MAX = __UINT_LEAST8_MAX__;
pub const INT_LEAST16_MAX = __INT_LEAST16_MAX__;
pub const INT_LEAST16_MIN = -__INT_LEAST16_MAX__ - @as(c_int, 1);
pub const UINT_LEAST16_MAX = __UINT_LEAST16_MAX__;
pub const INT_LEAST32_MAX = __INT_LEAST32_MAX__;
pub const INT_LEAST32_MIN = -__INT_LEAST32_MAX__ - @as(c_int, 1);
pub const UINT_LEAST32_MAX = __UINT_LEAST32_MAX__;
pub const INT_LEAST64_MAX = __INT_LEAST64_MAX__;
pub const INT_LEAST64_MIN = -__INT_LEAST64_MAX__ - @as(c_int, 1);
pub const UINT_LEAST64_MAX = __UINT_LEAST64_MAX__;
pub const INT_FAST8_MAX = __INT_FAST8_MAX__;
pub const INT_FAST8_MIN = -__INT_FAST8_MAX__ - @as(c_int, 1);
pub const UINT_FAST8_MAX = __UINT_FAST8_MAX__;
pub const INT_FAST16_MAX = __INT_FAST16_MAX__;
pub const INT_FAST16_MIN = -__INT_FAST16_MAX__ - @as(c_int, 1);
pub const UINT_FAST16_MAX = __UINT_FAST16_MAX__;
pub const INT_FAST32_MAX = __INT_FAST32_MAX__;
pub const INT_FAST32_MIN = -__INT_FAST32_MAX__ - @as(c_int, 1);
pub const UINT_FAST32_MAX = __UINT_FAST32_MAX__;
pub const INT_FAST64_MAX = __INT_FAST64_MAX__;
pub const INT_FAST64_MIN = -__INT_FAST64_MAX__ - @as(c_int, 1);
pub const UINT_FAST64_MAX = __UINT_FAST64_MAX__;
pub const __STDC_VERSION_STDDEF_H__ = @as(c_long, 202311);
pub const NULL = __helpers.cast(?*anyopaque, @as(c_int, 0));
pub const offsetof = @compileError("unable to translate macro: undefined identifier `__builtin_offsetof`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stddef.h:18:9
pub const PICO_OPAQUE_ABSOLUTE_TIME_T = @as(c_int, 0);
pub const ABSOLUTE_TIME_INITIALIZED_VAR = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/types.h:91:9
pub const PICO_INCLUDE_RTC_DATETIME = @compileError("unable to translate macro: undefined identifier `PICO_RP2040`"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico/types.h:96:9
pub inline fn bool_to_bit(x: anytype) uint {
    _ = &x;
    return __helpers.cast(uint, !!(x != 0));
}
pub const _PICO_VERSION_H = "";
pub const PICO_SDK_VERSION_MAJOR = @as(c_int, 2);
pub const PICO_SDK_VERSION_MINOR = @as(c_int, 2);
pub const PICO_SDK_VERSION_REVISION = @as(c_int, 0);
pub const PICO_SDK_VERSION_STRING = "2.2.0";
pub const pico_board_cmake_set = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico.h:41:9
pub const pico_board_cmake_set_default = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/common/pico_base_headers/include/pico.h:56:9
pub const _PICO_CONFIG_H = "";
pub const _BOARDS_PICO2_W_H = "";
pub const RASPBERRYPI_PICO2_W = "";
pub const PICO_RP2350A = @as(c_int, 1);
pub const PICO_DEFAULT_UART = @as(c_int, 0);
pub const PICO_DEFAULT_UART_TX_PIN = @as(c_int, 0);
pub const PICO_DEFAULT_UART_RX_PIN = @as(c_int, 1);
pub const PICO_DEFAULT_I2C = @as(c_int, 0);
pub const PICO_DEFAULT_I2C_SDA_PIN = @as(c_int, 4);
pub const PICO_DEFAULT_I2C_SCL_PIN = @as(c_int, 5);
pub const PICO_DEFAULT_SPI = @as(c_int, 0);
pub const PICO_DEFAULT_SPI_SCK_PIN = @as(c_int, 18);
pub const PICO_DEFAULT_SPI_TX_PIN = @as(c_int, 19);
pub const PICO_DEFAULT_SPI_RX_PIN = @as(c_int, 16);
pub const PICO_DEFAULT_SPI_CSN_PIN = @as(c_int, 17);
pub const PICO_BOOT_STAGE2_CHOOSE_W25Q080 = @as(c_int, 1);
pub const PICO_FLASH_SPI_CLKDIV = @as(c_int, 2);
pub const PICO_FLASH_SIZE_BYTES = (@as(c_int, 4) * @as(c_int, 1024)) * @as(c_int, 1024);
pub const CYW43_WL_GPIO_COUNT = @as(c_int, 3);
pub const CYW43_WL_GPIO_LED_PIN = @as(c_int, 0);
pub const CYW43_WL_GPIO_SMPS_PIN = @as(c_int, 1);
pub const CYW43_WL_GPIO_VBUS_PIN = @as(c_int, 2);
pub const CYW43_USES_VSYS_PIN = @as(c_int, 1);
pub const PICO_VSYS_PIN = @as(c_int, 29);
pub const PICO_RP2350_A2_SUPPORTED = @as(c_int, 1);
pub const CYW43_PIN_WL_DYNAMIC = @as(c_int, 0);
pub const CYW43_DEFAULT_PIN_WL_REG_ON = @as(c_uint, 23);
pub const CYW43_DEFAULT_PIN_WL_DATA_OUT = @as(c_uint, 24);
pub const CYW43_DEFAULT_PIN_WL_DATA_IN = @as(c_uint, 24);
pub const CYW43_DEFAULT_PIN_WL_HOST_WAKE = @as(c_uint, 24);
pub const CYW43_DEFAULT_PIN_WL_CLOCK = @as(c_uint, 29);
pub const CYW43_DEFAULT_PIN_WL_CS = @as(c_uint, 25);
pub const _PICO_PLATFORM_H = "";
pub const _PICO_PLATFORM_COMPILER_H = "";
pub const _HARDWARE_PLATFORM_DEFS_H = "";
pub const _u = __helpers.U_SUFFIX;
pub const NUM_CORES = _u(@as(c_int, 2));
pub const NUM_DMA_CHANNELS = _u(@as(c_int, 16));
pub const NUM_DMA_TIMERS = _u(@as(c_int, 4));
pub const NUM_DMA_MPU_REGIONS = _u(@as(c_int, 8));
pub const NUM_DMA_IRQS = _u(@as(c_int, 4));
pub const NUM_IRQS = _u(@as(c_int, 52));
pub const NUM_USER_IRQS = _u(@as(c_int, 6));
pub const NUM_PIOS = _u(@as(c_int, 3));
pub const NUM_PIO_STATE_MACHINES = _u(@as(c_int, 4));
pub const NUM_PIO_IRQS = _u(@as(c_int, 2));
pub const NUM_PWM_SLICES = _u(@as(c_int, 12));
pub const NUM_PWM_IRQS = _u(@as(c_int, 2));
pub const NUM_SPIN_LOCKS = _u(@as(c_int, 32));
pub const NUM_UARTS = _u(@as(c_int, 2));
pub const NUM_I2CS = _u(@as(c_int, 2));
pub const NUM_SPIS = _u(@as(c_int, 2));
pub const NUM_GENERIC_TIMERS = _u(@as(c_int, 2));
pub const NUM_ALARMS = _u(@as(c_int, 4));
pub const NUM_ADC_CHANNELS = _u(@as(c_int, 5));
pub const ADC_BASE_PIN = _u(@as(c_int, 26));
pub const NUM_RESETS = _u(@as(c_int, 28));
pub const NUM_BANK0_GPIOS = _u(@as(c_int, 30));
pub const NUM_QSPI_GPIOS = _u(@as(c_int, 6));
pub const NUM_OTP_PAGES = _u(@as(c_int, 64));
pub const NUM_OTP_PAGE_ROWS = _u(@as(c_int, 64));
pub const NUM_OTP_ROWS = NUM_OTP_PAGES * NUM_OTP_PAGE_ROWS;
pub const PIO_INSTRUCTION_COUNT = _u(@as(c_int, 32));
pub const NUM_MPU_REGIONS = _u(@as(c_int, 8));
pub const NUM_SAU_REGIONS = _u(@as(c_int, 8));
pub const NUM_BOOT_LOCKS = _u(@as(c_int, 8));
pub const BOOTRAM_SIZE = _u(@as(c_int, 0x400));
pub const USBCTRL_DPRAM_SIZE = _u(@as(c_int, 4096));
pub const HAS_GPIO_COPROCESSOR = @as(c_int, 1);
pub const HAS_DOUBLE_COPROCESSOR = @as(c_int, 1);
pub const HAS_REDUNDANCY_COPROCESSOR = @as(c_int, 1);
pub const HAS_POWMAN_TIMER = @as(c_int, 1);
pub const HAS_RP2350_TRNG = @as(c_int, 1);
pub const HAS_HSTX = @as(c_int, 1);
pub const HAS_PADS_BANK0_ISOLATION = @as(c_int, 1);
pub const __RISCV_PMP_CHECKED = @as(c_int, 1);
pub const FPGA_CLK_SYS_HZ = @compileError("unable to translate macro: undefined identifier `MHZ`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/platform_defs.h:80:9
pub const FPGA_CLK_REF_HZ = @compileError("unable to translate macro: undefined identifier `MHZ`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/platform_defs.h:84:9
pub const XOSC_HZ = _u(__helpers.promoteIntLiteral(c_int, 12000000, .decimal));
pub const PICO_USE_FASTEST_SUPPORTED_CLOCK = @as(c_int, 0);
pub const SYS_CLK_HZ = _u(__helpers.promoteIntLiteral(c_int, 150000000, .decimal));
pub const USB_CLK_HZ = _u(__helpers.promoteIntLiteral(c_int, 48000000, .decimal));
pub const XOSC_KHZ = __helpers.div(XOSC_HZ, @as(c_int, 1000));
pub const XOSC_MHZ = __helpers.div(XOSC_KHZ, @as(c_int, 1000));
pub const SYS_CLK_KHZ = __helpers.div(SYS_CLK_HZ, @as(c_int, 1000));
pub const SYS_CLK_MHZ = __helpers.div(SYS_CLK_KHZ, @as(c_int, 1000));
pub const USB_CLK_KHZ = __helpers.div(USB_CLK_HZ, @as(c_int, 1000));
pub const USB_CLK_MHZ = __helpers.div(USB_CLK_KHZ, @as(c_int, 1000));
pub const POWMAN_PASSWORD_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x5afe0000, .hex));
pub const VTABLE_FIRST_IRQ = @as(c_int, 16);
pub const FIRST_USER_IRQ = NUM_IRQS - NUM_USER_IRQS;
pub const REG_FIELD_WIDTH = @compileError("unable to translate macro: undefined identifier `_MSB`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/platform_defs.h:178:9
pub const _SYS_CDEFS_H_ = "";
pub const _MACHINE__DEFAULT_TYPES_H = "";
pub const __have_longlong64 = @as(c_int, 1);
pub const __have_long32 = @as(c_int, 1);
pub const ___int8_t_defined = @as(c_int, 1);
pub const ___int16_t_defined = @as(c_int, 1);
pub const ___int32_t_defined = @as(c_int, 1);
pub const ___int64_t_defined = @as(c_int, 1);
pub const ___int_least8_t_defined = @as(c_int, 1);
pub const ___int_least16_t_defined = @as(c_int, 1);
pub const ___int_least32_t_defined = @as(c_int, 1);
pub const ___int_least64_t_defined = @as(c_int, 1);
pub inline fn __PMT(args: anytype) @TypeOf(args) {
    _ = &args;
    return args;
}
pub const __DOTS = @compileError("unable to translate C expr: unexpected token ','"); // /usr/arm-none-eabi/include/sys/cdefs.h:50:9
pub const __THROW = "";
pub const __ASMNAME = @compileError("unable to translate macro: undefined identifier `__USER_LABEL_PREFIX__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:54:10
pub const __ptr_t = ?*anyopaque;
pub const __long_double_t = @compileError("unable to translate: TODO long double"); // /usr/arm-none-eabi/include/sys/cdefs.h:58:9
pub const __attribute_malloc__ = "";
pub const __attribute_pure__ = "";
pub const __attribute_format_strfmon__ = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:62:9
pub const __flexarr = @compileError("unable to translate C expr: unexpected token '['"); // /usr/arm-none-eabi/include/sys/cdefs.h:63:9
pub const __bounded = "";
pub const __unbounded = "";
pub const __ptrvalue = "";
pub const __BEGIN_DECLS = "";
pub const __END_DECLS = "";
pub const __GNUCLIKE_ASM = @as(c_int, 3);
pub const __GNUCLIKE_MATH_BUILTIN_CONSTANTS = "";
pub const __GNUCLIKE___TYPEOF = @as(c_int, 1);
pub const __GNUCLIKE___SECTION = @as(c_int, 1);
pub const __GNUCLIKE_CTOR_SECTION_HANDLING = @as(c_int, 1);
pub const __GNUCLIKE_BUILTIN_CONSTANT_P = @as(c_int, 1);
pub const __GNUCLIKE_BUILTIN_VARARGS = @as(c_int, 1);
pub const __GNUCLIKE_BUILTIN_STDARG = @as(c_int, 1);
pub const __GNUCLIKE_BUILTIN_VAALIST = @as(c_int, 1);
pub const __GNUC_VA_LIST_COMPATIBILITY = @as(c_int, 1);
pub const __compiler_membar = @compileError("unable to translate C expr: unexpected token '__asm'"); // /usr/arm-none-eabi/include/sys/cdefs.h:130:9
pub const __GNUCLIKE_BUILTIN_NEXT_ARG = @as(c_int, 1);
pub const __GNUCLIKE_MATH_BUILTIN_RELOPS = "";
pub const __GNUCLIKE_BUILTIN_MEMCPY = @as(c_int, 1);
pub const __CC_SUPPORTS_INLINE = @as(c_int, 1);
pub const __CC_SUPPORTS___INLINE = @as(c_int, 1);
pub const __CC_SUPPORTS___INLINE__ = @as(c_int, 1);
pub const __CC_SUPPORTS___FUNC__ = @as(c_int, 1);
pub const __CC_SUPPORTS_WARNING = @as(c_int, 1);
pub const __CC_SUPPORTS_VARADIC_XXX = @as(c_int, 1);
pub const __CC_SUPPORTS_DYNAMIC_ARRAY_INIT = @as(c_int, 1);
pub inline fn __P(protos: anytype) @TypeOf(protos) {
    _ = &protos;
    return protos;
}
pub const __CONCAT1 = @compileError("unable to translate C expr: unexpected token '##'"); // /usr/arm-none-eabi/include/sys/cdefs.h:165:9
pub inline fn __CONCAT(x: anytype, y: anytype) @TypeOf(__CONCAT1(x, y)) {
    _ = &x;
    _ = &y;
    return __CONCAT1(x, y);
}
pub const __STRING = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:167:9
pub inline fn __XSTRING(x: anytype) @TypeOf(__STRING(x)) {
    _ = &x;
    return __STRING(x);
}
pub const __const = @compileError("unable to translate C expr: unexpected token 'const'"); // /usr/arm-none-eabi/include/sys/cdefs.h:170:9
pub const __signed = c_int;
pub const __volatile = @compileError("unable to translate C expr: unexpected token 'volatile'"); // /usr/arm-none-eabi/include/sys/cdefs.h:172:9
pub const __weak_symbol = @compileError("unable to translate macro: undefined identifier `__weak__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:217:9
pub const __dead2 = @compileError("unable to translate macro: undefined identifier `__noreturn__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:230:9
pub const __pure2 = @compileError("unable to translate C expr: unexpected token '__attribute__'"); // /usr/arm-none-eabi/include/sys/cdefs.h:231:9
pub const __unused = @compileError("unable to translate macro: undefined identifier `__unused__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:232:9
pub const __used = @compileError("unable to translate macro: undefined identifier `__used__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:233:9
pub const __packed = @compileError("unable to translate macro: undefined identifier `__packed__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:234:9
pub const __aligned = @compileError("unable to translate macro: undefined identifier `__aligned__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:235:9
pub const __section = @compileError("unable to translate macro: undefined identifier `__section__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:236:9
pub const __alloc_size = @compileError("unable to translate macro: undefined identifier `__alloc_size__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:239:9
pub const __alloc_size2 = @compileError("unable to translate macro: undefined identifier `__alloc_size__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:240:9
pub const __alloc_align = @compileError("unable to translate macro: undefined identifier `__alloc_align__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:246:9
pub const __generic = @compileError("unable to translate C expr: unexpected token '_Generic'"); // /usr/arm-none-eabi/include/sys/cdefs.h:338:9
pub inline fn __min_size(x: anytype) @TypeOf(x) {
    _ = &x;
    return x;
}
pub const __malloc_like = @compileError("unable to translate macro: undefined identifier `__malloc__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:362:9
pub const __pure = @compileError("unable to translate macro: undefined identifier `__pure__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:363:9
pub const __always_inline = @compileError("unable to translate macro: undefined identifier `__always_inline__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:370:9
pub const __noinline = @compileError("unable to translate macro: undefined identifier `__noinline__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:376:9
pub const __nonnull = @compileError("unable to translate macro: undefined identifier `__nonnull__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:382:9
pub const __nonnull_all = @compileError("unable to translate macro: undefined identifier `__nonnull__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:383:9
pub const __fastcall = @compileError("unable to translate macro: undefined identifier `__fastcall__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:390:9
pub const __result_use_check = @compileError("unable to translate macro: undefined identifier `__warn_unused_result__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:391:9
pub const __returns_twice = @compileError("unable to translate macro: undefined identifier `__returns_twice__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:398:9
pub inline fn __unreachable() @TypeOf(__builtin.@"unreachable"()) {
    return __builtin.@"unreachable"();
}
pub const __restrict = @compileError("unable to translate C expr: unexpected token 'restrict'"); // /usr/arm-none-eabi/include/sys/cdefs.h:421:9
pub const __restrict_arr = @compileError("unable to translate C expr: unexpected token 'restrict'"); // /usr/arm-none-eabi/include/sys/cdefs.h:434:9
pub inline fn __predict_true(exp: anytype) @TypeOf(__builtin.expect(exp, @as(c_int, 1))) {
    _ = &exp;
    return __builtin.expect(exp, @as(c_int, 1));
}
pub inline fn __predict_false(exp: anytype) @TypeOf(__builtin.expect(exp, @as(c_int, 0))) {
    _ = &exp;
    return __builtin.expect(exp, @as(c_int, 0));
}
pub const __null_sentinel = @compileError("unable to translate macro: undefined identifier `__sentinel__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:476:9
pub const __exported = @compileError("unable to translate macro: undefined identifier `__visibility__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:477:9
pub const __hidden = @compileError("unable to translate macro: undefined identifier `__visibility__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:480:9
pub inline fn __offsetof(@"type": anytype, field: anytype) @TypeOf(offsetof(@"type", field)) {
    _ = &@"type";
    _ = &field;
    return offsetof(@"type", field);
}
pub inline fn __rangeof(@"type": anytype, start: anytype, end: anytype) @TypeOf(__offsetof(@"type", end) - __offsetof(@"type", start)) {
    _ = &@"type";
    _ = &start;
    _ = &end;
    return __offsetof(@"type", end) - __offsetof(@"type", start);
}
pub const __containerof = @compileError("unable to translate macro: undefined identifier `__x`"); // /usr/arm-none-eabi/include/sys/cdefs.h:501:9
pub const __printflike = @compileError("unable to translate macro: undefined identifier `__format__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:523:9
pub const __scanflike = @compileError("unable to translate macro: undefined identifier `__format__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:525:9
pub const __format_arg = @compileError("unable to translate macro: undefined identifier `__format_arg__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:527:9
pub const __strfmonlike = @compileError("unable to translate macro: undefined identifier `__format__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:528:9
pub const __strftimelike = @compileError("unable to translate macro: undefined identifier `__format__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:530:9
pub const __printf0like = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:540:9
pub const __strong_reference = @compileError("unable to translate macro: undefined identifier `__alias__`"); // /usr/arm-none-eabi/include/sys/cdefs.h:544:9
pub const __weak_reference = @compileError("unable to translate C expr: unexpected token '__asm__'"); // /usr/arm-none-eabi/include/sys/cdefs.h:548:9
pub const __warn_references = @compileError("unable to translate C expr: unexpected token '__asm__'"); // /usr/arm-none-eabi/include/sys/cdefs.h:551:9
pub const __sym_compat = @compileError("unable to translate C expr: unexpected token '__asm__'"); // /usr/arm-none-eabi/include/sys/cdefs.h:555:9
pub const __sym_default = @compileError("unable to translate C expr: unexpected token '__asm__'"); // /usr/arm-none-eabi/include/sys/cdefs.h:557:9
pub const __FBSDID = @compileError("unable to translate macro: undefined identifier `__hack`"); // /usr/arm-none-eabi/include/sys/cdefs.h:592:9
pub const __RCSID = @compileError("unable to translate macro: undefined identifier `__hack`"); // /usr/arm-none-eabi/include/sys/cdefs.h:596:9
pub const __RCSID_SOURCE = @compileError("unable to translate macro: undefined identifier `__hack`"); // /usr/arm-none-eabi/include/sys/cdefs.h:600:9
pub const __SCCSID = @compileError("unable to translate macro: undefined identifier `__hack`"); // /usr/arm-none-eabi/include/sys/cdefs.h:604:9
pub const __COPYRIGHT = @compileError("unable to translate macro: undefined identifier `__hack`"); // /usr/arm-none-eabi/include/sys/cdefs.h:608:9
pub const __DECONST = @compileError("unable to translate C expr: unexpected token 'const'"); // /usr/arm-none-eabi/include/sys/cdefs.h:612:9
pub const __DEVOLATILE = @compileError("unable to translate C expr: unexpected token 'volatile'"); // /usr/arm-none-eabi/include/sys/cdefs.h:616:9
pub const __DEQUALIFY = @compileError("unable to translate C expr: unexpected token 'const'"); // /usr/arm-none-eabi/include/sys/cdefs.h:620:9
pub const _Nonnull = "";
pub const _Nullable = "";
pub const _Null_unspecified = "";
pub const __NULLABILITY_PRAGMA_PUSH = "";
pub const __NULLABILITY_PRAGMA_POP = "";
pub const __arg_type_tag = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:652:9
pub const __datatype_type_tag = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:653:9
pub const __lock_annotate = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/cdefs.h:671:9
pub const __lockable = @compileError("unable to translate macro: undefined identifier `lockable`"); // /usr/arm-none-eabi/include/sys/cdefs.h:677:9
pub const __locks_exclusive = @compileError("unable to translate macro: undefined identifier `exclusive_lock_function`"); // /usr/arm-none-eabi/include/sys/cdefs.h:680:9
pub const __locks_shared = @compileError("unable to translate macro: undefined identifier `shared_lock_function`"); // /usr/arm-none-eabi/include/sys/cdefs.h:682:9
pub const __trylocks_exclusive = @compileError("unable to translate macro: undefined identifier `exclusive_trylock_function`"); // /usr/arm-none-eabi/include/sys/cdefs.h:686:9
pub const __trylocks_shared = @compileError("unable to translate macro: undefined identifier `shared_trylock_function`"); // /usr/arm-none-eabi/include/sys/cdefs.h:688:9
pub const __unlocks = @compileError("unable to translate macro: undefined identifier `unlock_function`"); // /usr/arm-none-eabi/include/sys/cdefs.h:692:9
pub const __asserts_exclusive = @compileError("unable to translate macro: undefined identifier `assert_exclusive_lock`"); // /usr/arm-none-eabi/include/sys/cdefs.h:695:9
pub const __asserts_shared = @compileError("unable to translate macro: undefined identifier `assert_shared_lock`"); // /usr/arm-none-eabi/include/sys/cdefs.h:697:9
pub const __requires_exclusive = @compileError("unable to translate macro: undefined identifier `exclusive_locks_required`"); // /usr/arm-none-eabi/include/sys/cdefs.h:701:9
pub const __requires_shared = @compileError("unable to translate macro: undefined identifier `shared_locks_required`"); // /usr/arm-none-eabi/include/sys/cdefs.h:703:9
pub const __requires_unlocked = @compileError("unable to translate macro: undefined identifier `locks_excluded`"); // /usr/arm-none-eabi/include/sys/cdefs.h:705:9
pub const __no_lock_analysis = @compileError("unable to translate macro: undefined identifier `no_thread_safety_analysis`"); // /usr/arm-none-eabi/include/sys/cdefs.h:709:9
pub const __nosanitizeaddress = "";
pub const __nosanitizememory = "";
pub const __nosanitizethread = "";
pub const __guarded_by = @compileError("unable to translate macro: undefined identifier `guarded_by`"); // /usr/arm-none-eabi/include/sys/cdefs.h:732:9
pub const __pt_guarded_by = @compileError("unable to translate macro: undefined identifier `pt_guarded_by`"); // /usr/arm-none-eabi/include/sys/cdefs.h:733:9
pub const __align_up = @compileError("unable to translate macro: undefined identifier `__builtin_align_up`"); // /usr/arm-none-eabi/include/sys/cdefs.h:750:9
pub const __align_down = @compileError("unable to translate macro: undefined identifier `__builtin_align_down`"); // /usr/arm-none-eabi/include/sys/cdefs.h:751:9
pub const __is_aligned = @compileError("unable to translate macro: undefined identifier `__builtin_is_aligned`"); // /usr/arm-none-eabi/include/sys/cdefs.h:752:9
pub const PICO_C_COMPILER_IS_GNU = @as(c_int, 1);
pub const __weak = @compileError("unable to translate macro: undefined identifier `weak`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:73:9
pub const GCC_Like_Pragma = @compileError("unable to translate macro: undefined identifier `_Pragma`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:79:9
pub const Clang_Pragma = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:88:9
pub const GCC_Pragma = @compileError("unable to translate macro: undefined identifier `_Pragma`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:93:9
pub const __isr = "";
pub const __packed_aligned = __packed ++ __aligned(@as(c_int, 4));
pub const __force_inline = @compileError("unable to translate C expr: unexpected token 'inline'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:123:9
pub const count_of = @compileError("unable to translate C expr: unexpected token '('"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:132:9
pub inline fn MAX(a: anytype, b: anytype) @TypeOf(if (a > b) a else b) {
    _ = &a;
    _ = &b;
    return if (a > b) a else b;
}
pub inline fn MIN(a: anytype, b: anytype) @TypeOf(if (b > a) a else b) {
    _ = &a;
    _ = &b;
    return if (b > a) a else b;
}
pub const pico_default_asm = @compileError("unable to translate C expr: unexpected token '__asm'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:155:9
pub const pico_default_asm_volatile = @compileError("unable to translate C expr: unexpected token '__asm'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:156:9
pub const pico_default_asm_goto = @compileError("unable to translate C expr: unexpected token '__asm'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:157:9
pub const pico_default_asm_volatile_goto = @compileError("unable to translate C expr: unexpected token '__asm'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:158:9
pub const __check_type_compatible = @compileError("unable to translate macro: undefined identifier `__builtin_types_compatible_p`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:183:9
pub const WRAPPER_FUNC = @compileError("unable to translate macro: undefined identifier `__wrap_`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:185:9
pub const REAL_FUNC = @compileError("unable to translate macro: undefined identifier `__real_`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_compiler/include/pico/platform/compiler.h:186:9
pub const _PICO_PLATFORM_SECTION_MACROS_H = "";
pub const __after_data = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:25:9
pub const __not_in_flash = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:41:9
pub const __scratch_x = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:60:9
pub const __scratch_y = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:79:9
pub const __uninitialized_ram = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:98:9
pub const __in_flash = @compileError("unable to translate macro: undefined identifier `section`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:114:9
pub const __not_in_flash_func = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:132:9
pub inline fn __time_critical_func(func_name: anytype) @TypeOf(__not_in_flash_func(func_name)) {
    _ = &func_name;
    return __not_in_flash_func(func_name);
}
pub inline fn __no_inline_not_in_flash_func(func_name: anytype) @TypeOf(__noinline ++ __not_in_flash_func(func_name)) {
    _ = &func_name;
    return __noinline ++ __not_in_flash_func(func_name);
}
pub const _PICO_PLATFORM_PANIC_H = "";
pub inline fn panic_compact() @TypeOf(panic("")) {
    return panic("");
}
pub const _PICO_PLATFORM_COMMON_H = "";
pub const PICO_MINIMAL_STORED_VECTOR_TABLE = @as(c_int, 0);
pub const PICO_NUM_VTABLE_IRQS = NUM_IRQS;
pub const PICO_NO_FPGA_CHECK = @as(c_int, 1);
pub const PICO_NO_SIM_CHECK = @as(c_int, 1);
pub inline fn host_safe_hw_ptr(x: anytype) usize {
    _ = &x;
    return __helpers.cast(usize, x);
}
pub inline fn native_safe_hw_ptr(x: anytype) @TypeOf(host_safe_hw_ptr(x)) {
    _ = &x;
    return host_safe_hw_ptr(x);
}
pub const _ADDRESSMAP_H = "";
pub const REG_ALIAS_RW_BITS = _u(@as(c_int, 0x0)) << _u(@as(c_int, 12));
pub const REG_ALIAS_XOR_BITS = _u(@as(c_int, 0x1)) << _u(@as(c_int, 12));
pub const REG_ALIAS_SET_BITS = _u(@as(c_int, 0x2)) << _u(@as(c_int, 12));
pub const REG_ALIAS_CLR_BITS = _u(@as(c_int, 0x3)) << _u(@as(c_int, 12));
pub const ROM_BASE = _u(@as(c_int, 0x00000000));
pub const XIP_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x10000000, .hex));
pub const XIP_SRAM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x13ffc000, .hex));
pub const XIP_END = _u(__helpers.promoteIntLiteral(c_int, 0x14000000, .hex));
pub const XIP_NOCACHE_NOALLOC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x14000000, .hex));
pub const XIP_SRAM_END = _u(__helpers.promoteIntLiteral(c_int, 0x14000000, .hex));
pub const XIP_NOCACHE_NOALLOC_END = _u(__helpers.promoteIntLiteral(c_int, 0x18000000, .hex));
pub const XIP_MAINTENANCE_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x18000000, .hex));
pub const XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x1c000000, .hex));
pub const SRAM0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20000000, .hex));
pub const XIP_NOCACHE_NOALLOC_NOTRANSLATE_END = _u(__helpers.promoteIntLiteral(c_int, 0x20000000, .hex));
pub const SRAM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20000000, .hex));
pub const SRAM_STRIPED_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20000000, .hex));
pub const SRAM4_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20040000, .hex));
pub const SRAM8_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20080000, .hex));
pub const SRAM_STRIPED_END = _u(__helpers.promoteIntLiteral(c_int, 0x20080000, .hex));
pub const SRAM_SCRATCH_X_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20080000, .hex));
pub const SRAM9_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20081000, .hex));
pub const SRAM_SCRATCH_Y_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x20081000, .hex));
pub const SRAM_END = _u(__helpers.promoteIntLiteral(c_int, 0x20082000, .hex));
pub const SYSINFO_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40000000, .hex));
pub const SYSCFG_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40008000, .hex));
pub const CLOCKS_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40010000, .hex));
pub const PSM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40018000, .hex));
pub const IO_QSPI_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40030000, .hex));
pub const XOSC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40048000, .hex));
pub const PLL_SYS_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40050000, .hex));
pub const PLL_USB_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40058000, .hex));
pub const BUSCTRL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40068000, .hex));
pub const UART0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40070000, .hex));
pub const UART1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40078000, .hex));
pub const SPI0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40080000, .hex));
pub const SPI1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40088000, .hex));
pub const I2C0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40090000, .hex));
pub const I2C1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40098000, .hex));
pub const ADC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400a0000, .hex));
pub const PWM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400a8000, .hex));
pub const TIMER0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400b0000, .hex));
pub const TIMER1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400b8000, .hex));
pub const HSTX_CTRL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400c0000, .hex));
pub const XIP_CTRL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400c8000, .hex));
pub const XIP_QMI_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400d0000, .hex));
pub const WATCHDOG_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400d8000, .hex));
pub const BOOTRAM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400e0000, .hex));
pub const BOOTRAM_END = _u(__helpers.promoteIntLiteral(c_int, 0x400e0400, .hex));
pub const ROSC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400e8000, .hex));
pub const TRNG_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400f0000, .hex));
pub const SHA256_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x400f8000, .hex));
pub const POWMAN_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40100000, .hex));
pub const TICKS_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40108000, .hex));
pub const OTP_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40120000, .hex));
pub const OTP_DATA_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40130000, .hex));
pub const OTP_DATA_RAW_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40134000, .hex));
pub const OTP_DATA_GUARDED_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40138000, .hex));
pub const OTP_DATA_RAW_GUARDED_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x4013c000, .hex));
pub const CORESIGHT_PERIPH_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40140000, .hex));
pub const CORESIGHT_ROMTABLE_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40140000, .hex));
pub const CORESIGHT_AHB_AP_CORE0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40142000, .hex));
pub const CORESIGHT_AHB_AP_CORE1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40144000, .hex));
pub const CORESIGHT_TIMESTAMP_GEN_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40146000, .hex));
pub const CORESIGHT_ATB_FUNNEL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40147000, .hex));
pub const CORESIGHT_TPIU_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40148000, .hex));
pub const CORESIGHT_CTI_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40149000, .hex));
pub const CORESIGHT_APB_AP_RISCV_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x4014a000, .hex));
pub const DFT_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40150000, .hex));
pub const GLITCH_DETECTOR_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40158000, .hex));
pub const TBMAN_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40160000, .hex));
pub const DMA_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50000000, .hex));
pub const USBCTRL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50100000, .hex));
pub const USBCTRL_DPRAM_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50100000, .hex));
pub const USBCTRL_REGS_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50110000, .hex));
pub const PIO0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50200000, .hex));
pub const PIO1_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50300000, .hex));
pub const PIO2_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50400000, .hex));
pub const XIP_AUX_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50500000, .hex));
pub const HSTX_FIFO_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50600000, .hex));
pub const CORESIGHT_TRACE_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x50700000, .hex));
pub const PPB_BASE = _u(__helpers.promoteIntLiteral(c_int, 0xe0000000, .hex));
pub const PPB_NONSEC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0xe0020000, .hex));
pub const EPPB_BASE = _u(__helpers.promoteIntLiteral(c_int, 0xe0080000, .hex));
pub const _HARDWARE_REGS_SIO_H = "";
pub const PICO_STACK_SIZE = _u(@as(c_int, 0x800));
pub const PICO_HEAP_SIZE = _u(@as(c_int, 0x800));
pub const PICO_NO_RAM_VECTOR_TABLE = @as(c_int, 0);
pub const PICO_USE_STACK_GUARDS = @as(c_int, 0);
pub const PICO_CLKDIV_ROUND_NEAREST = @as(c_int, 1);
pub const __fast_mul = @compileError("unable to translate macro: undefined identifier `__builtin_choose_expr`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/pico_platform/include/pico/platform.h:247:9
pub const _PICO_ERROR_H = "";
pub const _HARDWARE_STRUCTS_ADC_H = "";
pub const _HARDWARE_ADDRESS_MAPPED_H = "";
pub inline fn check_hw_layout(@"type": anytype, member: anytype, offset: anytype) @TypeOf(static_assert(offsetof(@"type", member) == offset, "hw offset mismatch")) {
    _ = &@"type";
    _ = &member;
    _ = &offset;
    return static_assert(offsetof(@"type", member) == offset, "hw offset mismatch");
}
pub inline fn check_hw_size(@"type": anytype, size: anytype) @TypeOf(static_assert(__helpers.sizeof(@"type") == size, "hw size mismatch")) {
    _ = &@"type";
    _ = &size;
    return static_assert(__helpers.sizeof(@"type") == size, "hw size mismatch");
}
pub const PARAM_ASSERTIONS_ENABLED_ADDRESS_ALIAS = @as(c_int, 0);
pub const _REG_ = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:84:9
pub inline fn hw_alias_check_addr(addr: anytype) usize {
    _ = &addr;
    return __helpers.cast(usize, addr);
}
pub inline fn hw_set_alias_untyped(addr: anytype) ?*anyopaque {
    _ = &addr;
    return __helpers.cast(?*anyopaque, REG_ALIAS_SET_BITS + hw_alias_check_addr(addr));
}
pub inline fn hw_clear_alias_untyped(addr: anytype) ?*anyopaque {
    _ = &addr;
    return __helpers.cast(?*anyopaque, REG_ALIAS_CLR_BITS + hw_alias_check_addr(addr));
}
pub inline fn hw_xor_alias_untyped(addr: anytype) ?*anyopaque {
    _ = &addr;
    return __helpers.cast(?*anyopaque, REG_ALIAS_XOR_BITS + hw_alias_check_addr(addr));
}
pub const hw_set_alias = @compileError("unable to translate C expr: unexpected token 'typeof'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:122:9
pub const hw_clear_alias = @compileError("unable to translate C expr: unexpected token 'typeof'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:123:9
pub const hw_xor_alias = @compileError("unable to translate C expr: unexpected token 'typeof'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:124:9
pub const xip_noalloc_alias = @compileError("unable to translate macro: undefined identifier `xip_noalloc_alias_untyped`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:125:9
pub const xip_nocache_alias = @compileError("unable to translate macro: undefined identifier `xip_nocache_alias_untyped`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:126:9
pub const xip_nocache_noalloc_alias = @compileError("unable to translate macro: undefined identifier `xip_nocache_noalloc_alias_untyped`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_base/include/hardware/address_mapped.h:127:9
pub const _HARDWARE_STRUCTS_ACCESSCTRL_H = "";
pub const _HARDWARE_REGS_ACCESSCTRL_H = "";
pub const accessctrl_hw = __helpers.cast([*c]accessctrl_hw_t, ACCESSCTRL_BASE);
pub const _HARDWARE_REGS_ADC_H = "";
pub const ADC_CS_OFFSET = _u(@as(c_int, 0x00000000));
pub const ADC_CS_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x01fff70f, .hex));
pub const ADC_CS_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_CS_RROBIN_RESET = _u(@as(c_int, 0x000));
pub const ADC_CS_RROBIN_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x01ff0000, .hex));
pub const ADC_CS_RROBIN_MSB = _u(@as(c_int, 24));
pub const ADC_CS_RROBIN_LSB = _u(@as(c_int, 16));
pub const ADC_CS_RROBIN_ACCESS = "RW";
pub const ADC_CS_AINSEL_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_AINSEL_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x0000f000, .hex));
pub const ADC_CS_AINSEL_MSB = _u(@as(c_int, 15));
pub const ADC_CS_AINSEL_LSB = _u(@as(c_int, 12));
pub const ADC_CS_AINSEL_ACCESS = "RW";
pub const ADC_CS_ERR_STICKY_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_ERR_STICKY_BITS = _u(@as(c_int, 0x00000400));
pub const ADC_CS_ERR_STICKY_MSB = _u(@as(c_int, 10));
pub const ADC_CS_ERR_STICKY_LSB = _u(@as(c_int, 10));
pub const ADC_CS_ERR_STICKY_ACCESS = "WC";
pub const ADC_CS_ERR_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_ERR_BITS = _u(@as(c_int, 0x00000200));
pub const ADC_CS_ERR_MSB = _u(@as(c_int, 9));
pub const ADC_CS_ERR_LSB = _u(@as(c_int, 9));
pub const ADC_CS_ERR_ACCESS = "RO";
pub const ADC_CS_READY_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_READY_BITS = _u(@as(c_int, 0x00000100));
pub const ADC_CS_READY_MSB = _u(@as(c_int, 8));
pub const ADC_CS_READY_LSB = _u(@as(c_int, 8));
pub const ADC_CS_READY_ACCESS = "RO";
pub const ADC_CS_START_MANY_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_START_MANY_BITS = _u(@as(c_int, 0x00000008));
pub const ADC_CS_START_MANY_MSB = _u(@as(c_int, 3));
pub const ADC_CS_START_MANY_LSB = _u(@as(c_int, 3));
pub const ADC_CS_START_MANY_ACCESS = "RW";
pub const ADC_CS_START_ONCE_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_START_ONCE_BITS = _u(@as(c_int, 0x00000004));
pub const ADC_CS_START_ONCE_MSB = _u(@as(c_int, 2));
pub const ADC_CS_START_ONCE_LSB = _u(@as(c_int, 2));
pub const ADC_CS_START_ONCE_ACCESS = "SC";
pub const ADC_CS_TS_EN_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_TS_EN_BITS = _u(@as(c_int, 0x00000002));
pub const ADC_CS_TS_EN_MSB = _u(@as(c_int, 1));
pub const ADC_CS_TS_EN_LSB = _u(@as(c_int, 1));
pub const ADC_CS_TS_EN_ACCESS = "RW";
pub const ADC_CS_EN_RESET = _u(@as(c_int, 0x0));
pub const ADC_CS_EN_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_CS_EN_MSB = _u(@as(c_int, 0));
pub const ADC_CS_EN_LSB = _u(@as(c_int, 0));
pub const ADC_CS_EN_ACCESS = "RW";
pub const ADC_RESULT_OFFSET = _u(@as(c_int, 0x00000004));
pub const ADC_RESULT_BITS = _u(@as(c_int, 0x00000fff));
pub const ADC_RESULT_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_RESULT_MSB = _u(@as(c_int, 11));
pub const ADC_RESULT_LSB = _u(@as(c_int, 0));
pub const ADC_RESULT_ACCESS = "RO";
pub const ADC_FCS_OFFSET = _u(@as(c_int, 0x00000008));
pub const ADC_FCS_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x0f0f0f0f, .hex));
pub const ADC_FCS_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_FCS_THRESH_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_THRESH_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x0f000000, .hex));
pub const ADC_FCS_THRESH_MSB = _u(@as(c_int, 27));
pub const ADC_FCS_THRESH_LSB = _u(@as(c_int, 24));
pub const ADC_FCS_THRESH_ACCESS = "RW";
pub const ADC_FCS_LEVEL_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_LEVEL_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x000f0000, .hex));
pub const ADC_FCS_LEVEL_MSB = _u(@as(c_int, 19));
pub const ADC_FCS_LEVEL_LSB = _u(@as(c_int, 16));
pub const ADC_FCS_LEVEL_ACCESS = "RO";
pub const ADC_FCS_OVER_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_OVER_BITS = _u(@as(c_int, 0x00000800));
pub const ADC_FCS_OVER_MSB = _u(@as(c_int, 11));
pub const ADC_FCS_OVER_LSB = _u(@as(c_int, 11));
pub const ADC_FCS_OVER_ACCESS = "WC";
pub const ADC_FCS_UNDER_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_UNDER_BITS = _u(@as(c_int, 0x00000400));
pub const ADC_FCS_UNDER_MSB = _u(@as(c_int, 10));
pub const ADC_FCS_UNDER_LSB = _u(@as(c_int, 10));
pub const ADC_FCS_UNDER_ACCESS = "WC";
pub const ADC_FCS_FULL_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_FULL_BITS = _u(@as(c_int, 0x00000200));
pub const ADC_FCS_FULL_MSB = _u(@as(c_int, 9));
pub const ADC_FCS_FULL_LSB = _u(@as(c_int, 9));
pub const ADC_FCS_FULL_ACCESS = "RO";
pub const ADC_FCS_EMPTY_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_EMPTY_BITS = _u(@as(c_int, 0x00000100));
pub const ADC_FCS_EMPTY_MSB = _u(@as(c_int, 8));
pub const ADC_FCS_EMPTY_LSB = _u(@as(c_int, 8));
pub const ADC_FCS_EMPTY_ACCESS = "RO";
pub const ADC_FCS_DREQ_EN_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_DREQ_EN_BITS = _u(@as(c_int, 0x00000008));
pub const ADC_FCS_DREQ_EN_MSB = _u(@as(c_int, 3));
pub const ADC_FCS_DREQ_EN_LSB = _u(@as(c_int, 3));
pub const ADC_FCS_DREQ_EN_ACCESS = "RW";
pub const ADC_FCS_ERR_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_ERR_BITS = _u(@as(c_int, 0x00000004));
pub const ADC_FCS_ERR_MSB = _u(@as(c_int, 2));
pub const ADC_FCS_ERR_LSB = _u(@as(c_int, 2));
pub const ADC_FCS_ERR_ACCESS = "RW";
pub const ADC_FCS_SHIFT_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_SHIFT_BITS = _u(@as(c_int, 0x00000002));
pub const ADC_FCS_SHIFT_MSB = _u(@as(c_int, 1));
pub const ADC_FCS_SHIFT_LSB = _u(@as(c_int, 1));
pub const ADC_FCS_SHIFT_ACCESS = "RW";
pub const ADC_FCS_EN_RESET = _u(@as(c_int, 0x0));
pub const ADC_FCS_EN_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_FCS_EN_MSB = _u(@as(c_int, 0));
pub const ADC_FCS_EN_LSB = _u(@as(c_int, 0));
pub const ADC_FCS_EN_ACCESS = "RW";
pub const ADC_FIFO_OFFSET = _u(@as(c_int, 0x0000000c));
pub const ADC_FIFO_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x00008fff, .hex));
pub const ADC_FIFO_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_FIFO_ERR_RESET = "-";
pub const ADC_FIFO_ERR_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x00008000, .hex));
pub const ADC_FIFO_ERR_MSB = _u(@as(c_int, 15));
pub const ADC_FIFO_ERR_LSB = _u(@as(c_int, 15));
pub const ADC_FIFO_ERR_ACCESS = "RF";
pub const ADC_FIFO_VAL_RESET = "-";
pub const ADC_FIFO_VAL_BITS = _u(@as(c_int, 0x00000fff));
pub const ADC_FIFO_VAL_MSB = _u(@as(c_int, 11));
pub const ADC_FIFO_VAL_LSB = _u(@as(c_int, 0));
pub const ADC_FIFO_VAL_ACCESS = "RF";
pub const ADC_DIV_OFFSET = _u(@as(c_int, 0x00000010));
pub const ADC_DIV_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x00ffffff, .hex));
pub const ADC_DIV_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_DIV_INT_RESET = _u(@as(c_int, 0x0000));
pub const ADC_DIV_INT_BITS = _u(__helpers.promoteIntLiteral(c_int, 0x00ffff00, .hex));
pub const ADC_DIV_INT_MSB = _u(@as(c_int, 23));
pub const ADC_DIV_INT_LSB = _u(@as(c_int, 8));
pub const ADC_DIV_INT_ACCESS = "RW";
pub const ADC_DIV_FRAC_RESET = _u(@as(c_int, 0x00));
pub const ADC_DIV_FRAC_BITS = _u(@as(c_int, 0x000000ff));
pub const ADC_DIV_FRAC_MSB = _u(@as(c_int, 7));
pub const ADC_DIV_FRAC_LSB = _u(@as(c_int, 0));
pub const ADC_DIV_FRAC_ACCESS = "RW";
pub const ADC_INTR_OFFSET = _u(@as(c_int, 0x00000014));
pub const ADC_INTR_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTR_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_INTR_FIFO_RESET = _u(@as(c_int, 0x0));
pub const ADC_INTR_FIFO_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTR_FIFO_MSB = _u(@as(c_int, 0));
pub const ADC_INTR_FIFO_LSB = _u(@as(c_int, 0));
pub const ADC_INTR_FIFO_ACCESS = "RO";
pub const ADC_INTE_OFFSET = _u(@as(c_int, 0x00000018));
pub const ADC_INTE_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTE_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_INTE_FIFO_RESET = _u(@as(c_int, 0x0));
pub const ADC_INTE_FIFO_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTE_FIFO_MSB = _u(@as(c_int, 0));
pub const ADC_INTE_FIFO_LSB = _u(@as(c_int, 0));
pub const ADC_INTE_FIFO_ACCESS = "RW";
pub const ADC_INTF_OFFSET = _u(@as(c_int, 0x0000001c));
pub const ADC_INTF_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTF_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_INTF_FIFO_RESET = _u(@as(c_int, 0x0));
pub const ADC_INTF_FIFO_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTF_FIFO_MSB = _u(@as(c_int, 0));
pub const ADC_INTF_FIFO_LSB = _u(@as(c_int, 0));
pub const ADC_INTF_FIFO_ACCESS = "RW";
pub const ADC_INTS_OFFSET = _u(@as(c_int, 0x00000020));
pub const ADC_INTS_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTS_RESET = _u(@as(c_int, 0x00000000));
pub const ADC_INTS_FIFO_RESET = _u(@as(c_int, 0x0));
pub const ADC_INTS_FIFO_BITS = _u(@as(c_int, 0x00000001));
pub const ADC_INTS_FIFO_MSB = _u(@as(c_int, 0));
pub const ADC_INTS_FIFO_LSB = _u(@as(c_int, 0));
pub const ADC_INTS_FIFO_ACCESS = "RO";
pub const adc_hw = __helpers.cast([*c]adc_hw_t, ADC_BASE);
pub const _HARDWARE_GPIO_H = "";
pub const _HARDWARE_STRUCTS_SIO_H = "";
pub const _HARDWARE_STRUCTS_INTERP_H = "";
pub const interp_hw_array = __helpers.cast([*c]interp_hw_t, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET);
pub const interp_hw_array_ns = __helpers.cast([*c]interp_hw_t, SIO_NONSEC_BASE + SIO_INTERP0_ACCUM0_OFFSET);
pub const interp0_hw = &interp_hw_array[@as(usize, @intCast(@as(c_int, 0)))];
pub const interp1_hw = &interp_hw_array[@as(usize, @intCast(@as(c_int, 1)))];
pub const sio_hw = __helpers.cast([*c]sio_hw_t, SIO_BASE);
pub const sio_ns_hw = __helpers.cast([*c]sio_hw_t, SIO_NONSEC_BASE);
pub const _HARDWARE_STRUCTS_PADS_BANK0_H = "";
pub const _HARDWARE_REGS_PADS_BANK0_H = "";
pub const pads_bank0_hw = __helpers.cast([*c]pads_bank0_hw_t, PADS_BANK0_BASE);
pub const _HARDWARE_IRQ_H = "";
pub const PICO_MAX_SHARED_IRQ_HANDLERS = @as(c_int, 4);
pub const PICO_DISABLE_SHARED_IRQ_HANDLERS = @as(c_int, 0);
pub const PICO_VTABLE_PER_CORE = @as(c_int, 0);
pub const _INTCTRL_H = "";
pub const isr_timer0_0 = @compileError("unable to translate macro: undefined identifier `isr_irq0`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:130:9
pub const isr_timer0_1 = @compileError("unable to translate macro: undefined identifier `isr_irq1`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:131:9
pub const isr_timer0_2 = @compileError("unable to translate macro: undefined identifier `isr_irq2`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:132:9
pub const isr_timer0_3 = @compileError("unable to translate macro: undefined identifier `isr_irq3`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:133:9
pub const isr_timer1_0 = @compileError("unable to translate macro: undefined identifier `isr_irq4`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:134:9
pub const isr_timer1_1 = @compileError("unable to translate macro: undefined identifier `isr_irq5`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:135:9
pub const isr_timer1_2 = @compileError("unable to translate macro: undefined identifier `isr_irq6`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:136:9
pub const isr_timer1_3 = @compileError("unable to translate macro: undefined identifier `isr_irq7`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:137:9
pub const isr_pwm_wrap_0 = @compileError("unable to translate macro: undefined identifier `isr_irq8`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:138:9
pub const isr_pwm_wrap_1 = @compileError("unable to translate macro: undefined identifier `isr_irq9`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:139:9
pub const isr_dma_0 = @compileError("unable to translate macro: undefined identifier `isr_irq10`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:140:9
pub const isr_dma_1 = @compileError("unable to translate macro: undefined identifier `isr_irq11`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:141:9
pub const isr_dma_2 = @compileError("unable to translate macro: undefined identifier `isr_irq12`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:142:9
pub const isr_dma_3 = @compileError("unable to translate macro: undefined identifier `isr_irq13`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:143:9
pub const isr_usbctrl = @compileError("unable to translate macro: undefined identifier `isr_irq14`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:144:9
pub const isr_pio0_0 = @compileError("unable to translate macro: undefined identifier `isr_irq15`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:145:9
pub const isr_pio0_1 = @compileError("unable to translate macro: undefined identifier `isr_irq16`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:146:9
pub const isr_pio1_0 = @compileError("unable to translate macro: undefined identifier `isr_irq17`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:147:9
pub const isr_pio1_1 = @compileError("unable to translate macro: undefined identifier `isr_irq18`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:148:9
pub const isr_pio2_0 = @compileError("unable to translate macro: undefined identifier `isr_irq19`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:149:9
pub const isr_pio2_1 = @compileError("unable to translate macro: undefined identifier `isr_irq20`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:150:9
pub const isr_io_bank0 = @compileError("unable to translate macro: undefined identifier `isr_irq21`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:151:9
pub const isr_io_bank0_ns = @compileError("unable to translate macro: undefined identifier `isr_irq22`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:152:9
pub const isr_io_qspi = @compileError("unable to translate macro: undefined identifier `isr_irq23`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:153:9
pub const isr_io_qspi_ns = @compileError("unable to translate macro: undefined identifier `isr_irq24`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:154:9
pub const isr_sio_fifo = @compileError("unable to translate macro: undefined identifier `isr_irq25`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:155:9
pub const isr_sio_bell = @compileError("unable to translate macro: undefined identifier `isr_irq26`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:156:9
pub const isr_sio_fifo_ns = @compileError("unable to translate macro: undefined identifier `isr_irq27`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:157:9
pub const isr_sio_bell_ns = @compileError("unable to translate macro: undefined identifier `isr_irq28`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:158:9
pub const isr_sio_mtimecmp = @compileError("unable to translate macro: undefined identifier `isr_irq29`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:159:9
pub const isr_clocks = @compileError("unable to translate macro: undefined identifier `isr_irq30`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:160:9
pub const isr_spi0 = @compileError("unable to translate macro: undefined identifier `isr_irq31`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:161:9
pub const isr_spi1 = @compileError("unable to translate macro: undefined identifier `isr_irq32`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:162:9
pub const isr_uart0 = @compileError("unable to translate macro: undefined identifier `isr_irq33`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:163:9
pub const isr_uart1 = @compileError("unable to translate macro: undefined identifier `isr_irq34`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:164:9
pub const isr_adc_fifo = @compileError("unable to translate macro: undefined identifier `isr_irq35`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:165:9
pub const isr_i2c0 = @compileError("unable to translate macro: undefined identifier `isr_irq36`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:166:9
pub const isr_i2c1 = @compileError("unable to translate macro: undefined identifier `isr_irq37`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:167:9
pub const isr_otp = @compileError("unable to translate macro: undefined identifier `isr_irq38`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:168:9
pub const isr_trng = @compileError("unable to translate macro: undefined identifier `isr_irq39`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:169:9
pub const isr_proc0_cti = @compileError("unable to translate macro: undefined identifier `isr_irq40`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:170:9
pub const isr_proc1_cti = @compileError("unable to translate macro: undefined identifier `isr_irq41`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:171:9
pub const isr_pll_sys = @compileError("unable to translate macro: undefined identifier `isr_irq42`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:172:9
pub const isr_pll_usb = @compileError("unable to translate macro: undefined identifier `isr_irq43`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:173:9
pub const isr_powman_pow = @compileError("unable to translate macro: undefined identifier `isr_irq44`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:174:9
pub const isr_powman_timer = @compileError("unable to translate macro: undefined identifier `isr_irq45`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:175:9
pub const isr_spare_0 = @compileError("unable to translate macro: undefined identifier `isr_irq46`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:176:9
pub const isr_spare_1 = @compileError("unable to translate macro: undefined identifier `isr_irq47`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:177:9
pub const isr_spare_2 = @compileError("unable to translate macro: undefined identifier `isr_irq48`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:178:9
pub const isr_spare_3 = @compileError("unable to translate macro: undefined identifier `isr_irq49`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:179:9
pub const isr_spare_4 = @compileError("unable to translate macro: undefined identifier `isr_irq50`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:180:9
pub const isr_spare_5 = @compileError("unable to translate macro: undefined identifier `isr_irq51`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/intctrl.h:181:9
pub const _PICO_PLATFORM_CPU_REGS_H = "";
pub const _HARDWARE_REGS_M33_H = "";
pub const ARM_CPU_PREFIXED = @compileError("unable to translate macro: undefined identifier `M33_`"); // /home/ianic/Code/pico/pico-sdk/src/rp2350/pico_platform/include/pico/platform/cpu_regs.h:22:9
pub const _HARDWARE_STRUCTS_M33_H = "";
pub const m33_hw = __helpers.cast([*c]m33_hw_t, PPB_BASE);
pub const m33_ns_hw = __helpers.cast([*c]m33_hw_t, PPB_NONSEC_BASE);
pub const arm_cpu_hw = m33_hw;
pub const _HARDWARE_STRUCTS_NVIC_H = "";
pub const nvic_hw = __helpers.cast([*c]nvic_hw_t, PPB_BASE + M33_NVIC_ISER0_OFFSET);
pub const nvic_ns_hw = __helpers.cast([*c]nvic_hw_t, PPB_NONSEC_BASE + M33_NVIC_ISER0_OFFSET);
pub const _HARDWARE_STRUCTS_SCB_H = "";
pub const scb_hw = __helpers.cast([*c]armv8m_scb_hw_t, PPB_BASE + M33_CPUID_OFFSET);
pub const scb_ns_hw = __helpers.cast([*c]armv8m_scb_hw_t, PPB_NONSEC_BASE + M33_CPUID_OFFSET);
pub const PICO_DEFAULT_IRQ_PRIORITY = @as(c_int, 0x80);
pub const PICO_LOWEST_IRQ_PRIORITY = @as(c_int, 0xff);
pub const PICO_HIGHEST_IRQ_PRIORITY = @as(c_int, 0x00);
pub const PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY = @as(c_int, 0x80);
pub const PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY = @as(c_int, 0xff);
pub const PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY = @as(c_int, 0x00);
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_IRQ = @as(c_int, 0);
pub const PICO_USE_GPIO_COPROCESSOR = @as(c_int, 1);
pub const _HARDWARE_GPIO_COPROC_H = "";
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_GPIO = @as(c_int, 0);
pub const GPIO_IRQ_CALLBACK_ORDER_PRIORITY = PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY;
pub const GPIO_RAW_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY = PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY;
pub const PICO_DEBUG_PIN_BASE = @as(c_uint, 19);
pub const PICO_DEBUG_PIN_COUNT = @as(c_uint, 3);
pub const CU_REGISTER_DEBUG_PINS = @compileError("unable to translate macro: undefined identifier `DEBUG_PIN_TYPE`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1428:9
pub const CU_SELECT_DEBUG_PINS = @compileError("unable to translate macro: undefined identifier `DEBUG_PIN_TYPE`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1429:9
pub const DEBUG_PINS_ENABLED = @compileError("unable to translate macro: undefined identifier `__selected_debug_pins`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1430:9
pub const DEBUG_PINS_SET = @compileError("unable to translate C expr: unexpected token 'if'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1441:9
pub const DEBUG_PINS_CLR = @compileError("unable to translate C expr: unexpected token 'if'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1442:9
pub const DEBUG_PINS_XOR = @compileError("unable to translate C expr: unexpected token 'if'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h:1443:9
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_ADC = @as(c_int, 0);
pub const ADC_TEMPERATURE_CHANNEL_NUM = NUM_ADC_CHANNELS - @as(c_int, 1);
pub const PICO_ADC_CLKDIV_ROUND_NEAREST = PICO_CLKDIV_ROUND_NEAREST;
pub const _PICO_CYW43_ARCH_H = "";
pub const CYW43_INCLUDED_CYW43_H = "";
pub const _CYW43_CONFIGPORT_H = "";
pub const _PICO_TIME_H = "";
pub const _HARDWARE_TIMER_H = "";
pub const _HARDWARE_STRUCTS_TIMER_H = "";
pub const _HARDWARE_REGS_TIMER_H = "";
pub const TIMER_TIMEHW_OFFSET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMEHW_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMEHW_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMEHW_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMEHW_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMEHW_ACCESS = "WF";
pub const TIMER_TIMELW_OFFSET = _u(@as(c_int, 0x00000004));
pub const TIMER_TIMELW_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMELW_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMELW_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMELW_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMELW_ACCESS = "WF";
pub const TIMER_TIMEHR_OFFSET = _u(@as(c_int, 0x00000008));
pub const TIMER_TIMEHR_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMEHR_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMEHR_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMEHR_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMEHR_ACCESS = "RO";
pub const TIMER_TIMELR_OFFSET = _u(@as(c_int, 0x0000000c));
pub const TIMER_TIMELR_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMELR_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMELR_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMELR_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMELR_ACCESS = "RO";
pub const TIMER_ALARM0_OFFSET = _u(@as(c_int, 0x00000010));
pub const TIMER_ALARM0_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_ALARM0_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_ALARM0_MSB = _u(@as(c_int, 31));
pub const TIMER_ALARM0_LSB = _u(@as(c_int, 0));
pub const TIMER_ALARM0_ACCESS = "RW";
pub const TIMER_ALARM1_OFFSET = _u(@as(c_int, 0x00000014));
pub const TIMER_ALARM1_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_ALARM1_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_ALARM1_MSB = _u(@as(c_int, 31));
pub const TIMER_ALARM1_LSB = _u(@as(c_int, 0));
pub const TIMER_ALARM1_ACCESS = "RW";
pub const TIMER_ALARM2_OFFSET = _u(@as(c_int, 0x00000018));
pub const TIMER_ALARM2_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_ALARM2_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_ALARM2_MSB = _u(@as(c_int, 31));
pub const TIMER_ALARM2_LSB = _u(@as(c_int, 0));
pub const TIMER_ALARM2_ACCESS = "RW";
pub const TIMER_ALARM3_OFFSET = _u(@as(c_int, 0x0000001c));
pub const TIMER_ALARM3_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_ALARM3_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_ALARM3_MSB = _u(@as(c_int, 31));
pub const TIMER_ALARM3_LSB = _u(@as(c_int, 0));
pub const TIMER_ALARM3_ACCESS = "RW";
pub const TIMER_ARMED_OFFSET = _u(@as(c_int, 0x00000020));
pub const TIMER_ARMED_BITS = _u(@as(c_int, 0x0000000f));
pub const TIMER_ARMED_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_ARMED_MSB = _u(@as(c_int, 3));
pub const TIMER_ARMED_LSB = _u(@as(c_int, 0));
pub const TIMER_ARMED_ACCESS = "WC";
pub const TIMER_TIMERAWH_OFFSET = _u(@as(c_int, 0x00000024));
pub const TIMER_TIMERAWH_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMERAWH_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMERAWH_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMERAWH_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMERAWH_ACCESS = "RO";
pub const TIMER_TIMERAWL_OFFSET = _u(@as(c_int, 0x00000028));
pub const TIMER_TIMERAWL_BITS = _u(__helpers.promoteIntLiteral(c_int, 0xffffffff, .hex));
pub const TIMER_TIMERAWL_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_TIMERAWL_MSB = _u(@as(c_int, 31));
pub const TIMER_TIMERAWL_LSB = _u(@as(c_int, 0));
pub const TIMER_TIMERAWL_ACCESS = "RO";
pub const TIMER_DBGPAUSE_OFFSET = _u(@as(c_int, 0x0000002c));
pub const TIMER_DBGPAUSE_BITS = _u(@as(c_int, 0x00000006));
pub const TIMER_DBGPAUSE_RESET = _u(@as(c_int, 0x00000007));
pub const TIMER_DBGPAUSE_DBG1_RESET = _u(@as(c_int, 0x1));
pub const TIMER_DBGPAUSE_DBG1_BITS = _u(@as(c_int, 0x00000004));
pub const TIMER_DBGPAUSE_DBG1_MSB = _u(@as(c_int, 2));
pub const TIMER_DBGPAUSE_DBG1_LSB = _u(@as(c_int, 2));
pub const TIMER_DBGPAUSE_DBG1_ACCESS = "RW";
pub const TIMER_DBGPAUSE_DBG0_RESET = _u(@as(c_int, 0x1));
pub const TIMER_DBGPAUSE_DBG0_BITS = _u(@as(c_int, 0x00000002));
pub const TIMER_DBGPAUSE_DBG0_MSB = _u(@as(c_int, 1));
pub const TIMER_DBGPAUSE_DBG0_LSB = _u(@as(c_int, 1));
pub const TIMER_DBGPAUSE_DBG0_ACCESS = "RW";
pub const TIMER_PAUSE_OFFSET = _u(@as(c_int, 0x00000030));
pub const TIMER_PAUSE_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_PAUSE_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_PAUSE_MSB = _u(@as(c_int, 0));
pub const TIMER_PAUSE_LSB = _u(@as(c_int, 0));
pub const TIMER_PAUSE_ACCESS = "RW";
pub const TIMER_LOCKED_OFFSET = _u(@as(c_int, 0x00000034));
pub const TIMER_LOCKED_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_LOCKED_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_LOCKED_MSB = _u(@as(c_int, 0));
pub const TIMER_LOCKED_LSB = _u(@as(c_int, 0));
pub const TIMER_LOCKED_ACCESS = "RW";
pub const TIMER_SOURCE_OFFSET = _u(@as(c_int, 0x00000038));
pub const TIMER_SOURCE_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_SOURCE_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_SOURCE_CLK_SYS_RESET = _u(@as(c_int, 0x0));
pub const TIMER_SOURCE_CLK_SYS_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_SOURCE_CLK_SYS_MSB = _u(@as(c_int, 0));
pub const TIMER_SOURCE_CLK_SYS_LSB = _u(@as(c_int, 0));
pub const TIMER_SOURCE_CLK_SYS_ACCESS = "RW";
pub const TIMER_SOURCE_CLK_SYS_VALUE_TICK = _u(@as(c_int, 0x0));
pub const TIMER_SOURCE_CLK_SYS_VALUE_CLK_SYS = _u(@as(c_int, 0x1));
pub const TIMER_INTR_OFFSET = _u(@as(c_int, 0x0000003c));
pub const TIMER_INTR_BITS = _u(@as(c_int, 0x0000000f));
pub const TIMER_INTR_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_INTR_ALARM_3_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTR_ALARM_3_BITS = _u(@as(c_int, 0x00000008));
pub const TIMER_INTR_ALARM_3_MSB = _u(@as(c_int, 3));
pub const TIMER_INTR_ALARM_3_LSB = _u(@as(c_int, 3));
pub const TIMER_INTR_ALARM_3_ACCESS = "WC";
pub const TIMER_INTR_ALARM_2_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTR_ALARM_2_BITS = _u(@as(c_int, 0x00000004));
pub const TIMER_INTR_ALARM_2_MSB = _u(@as(c_int, 2));
pub const TIMER_INTR_ALARM_2_LSB = _u(@as(c_int, 2));
pub const TIMER_INTR_ALARM_2_ACCESS = "WC";
pub const TIMER_INTR_ALARM_1_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTR_ALARM_1_BITS = _u(@as(c_int, 0x00000002));
pub const TIMER_INTR_ALARM_1_MSB = _u(@as(c_int, 1));
pub const TIMER_INTR_ALARM_1_LSB = _u(@as(c_int, 1));
pub const TIMER_INTR_ALARM_1_ACCESS = "WC";
pub const TIMER_INTR_ALARM_0_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTR_ALARM_0_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_INTR_ALARM_0_MSB = _u(@as(c_int, 0));
pub const TIMER_INTR_ALARM_0_LSB = _u(@as(c_int, 0));
pub const TIMER_INTR_ALARM_0_ACCESS = "WC";
pub const TIMER_INTE_OFFSET = _u(@as(c_int, 0x00000040));
pub const TIMER_INTE_BITS = _u(@as(c_int, 0x0000000f));
pub const TIMER_INTE_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_INTE_ALARM_3_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTE_ALARM_3_BITS = _u(@as(c_int, 0x00000008));
pub const TIMER_INTE_ALARM_3_MSB = _u(@as(c_int, 3));
pub const TIMER_INTE_ALARM_3_LSB = _u(@as(c_int, 3));
pub const TIMER_INTE_ALARM_3_ACCESS = "RW";
pub const TIMER_INTE_ALARM_2_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTE_ALARM_2_BITS = _u(@as(c_int, 0x00000004));
pub const TIMER_INTE_ALARM_2_MSB = _u(@as(c_int, 2));
pub const TIMER_INTE_ALARM_2_LSB = _u(@as(c_int, 2));
pub const TIMER_INTE_ALARM_2_ACCESS = "RW";
pub const TIMER_INTE_ALARM_1_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTE_ALARM_1_BITS = _u(@as(c_int, 0x00000002));
pub const TIMER_INTE_ALARM_1_MSB = _u(@as(c_int, 1));
pub const TIMER_INTE_ALARM_1_LSB = _u(@as(c_int, 1));
pub const TIMER_INTE_ALARM_1_ACCESS = "RW";
pub const TIMER_INTE_ALARM_0_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTE_ALARM_0_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_INTE_ALARM_0_MSB = _u(@as(c_int, 0));
pub const TIMER_INTE_ALARM_0_LSB = _u(@as(c_int, 0));
pub const TIMER_INTE_ALARM_0_ACCESS = "RW";
pub const TIMER_INTF_OFFSET = _u(@as(c_int, 0x00000044));
pub const TIMER_INTF_BITS = _u(@as(c_int, 0x0000000f));
pub const TIMER_INTF_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_INTF_ALARM_3_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTF_ALARM_3_BITS = _u(@as(c_int, 0x00000008));
pub const TIMER_INTF_ALARM_3_MSB = _u(@as(c_int, 3));
pub const TIMER_INTF_ALARM_3_LSB = _u(@as(c_int, 3));
pub const TIMER_INTF_ALARM_3_ACCESS = "RW";
pub const TIMER_INTF_ALARM_2_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTF_ALARM_2_BITS = _u(@as(c_int, 0x00000004));
pub const TIMER_INTF_ALARM_2_MSB = _u(@as(c_int, 2));
pub const TIMER_INTF_ALARM_2_LSB = _u(@as(c_int, 2));
pub const TIMER_INTF_ALARM_2_ACCESS = "RW";
pub const TIMER_INTF_ALARM_1_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTF_ALARM_1_BITS = _u(@as(c_int, 0x00000002));
pub const TIMER_INTF_ALARM_1_MSB = _u(@as(c_int, 1));
pub const TIMER_INTF_ALARM_1_LSB = _u(@as(c_int, 1));
pub const TIMER_INTF_ALARM_1_ACCESS = "RW";
pub const TIMER_INTF_ALARM_0_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTF_ALARM_0_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_INTF_ALARM_0_MSB = _u(@as(c_int, 0));
pub const TIMER_INTF_ALARM_0_LSB = _u(@as(c_int, 0));
pub const TIMER_INTF_ALARM_0_ACCESS = "RW";
pub const TIMER_INTS_OFFSET = _u(@as(c_int, 0x00000048));
pub const TIMER_INTS_BITS = _u(@as(c_int, 0x0000000f));
pub const TIMER_INTS_RESET = _u(@as(c_int, 0x00000000));
pub const TIMER_INTS_ALARM_3_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTS_ALARM_3_BITS = _u(@as(c_int, 0x00000008));
pub const TIMER_INTS_ALARM_3_MSB = _u(@as(c_int, 3));
pub const TIMER_INTS_ALARM_3_LSB = _u(@as(c_int, 3));
pub const TIMER_INTS_ALARM_3_ACCESS = "RO";
pub const TIMER_INTS_ALARM_2_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTS_ALARM_2_BITS = _u(@as(c_int, 0x00000004));
pub const TIMER_INTS_ALARM_2_MSB = _u(@as(c_int, 2));
pub const TIMER_INTS_ALARM_2_LSB = _u(@as(c_int, 2));
pub const TIMER_INTS_ALARM_2_ACCESS = "RO";
pub const TIMER_INTS_ALARM_1_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTS_ALARM_1_BITS = _u(@as(c_int, 0x00000002));
pub const TIMER_INTS_ALARM_1_MSB = _u(@as(c_int, 1));
pub const TIMER_INTS_ALARM_1_LSB = _u(@as(c_int, 1));
pub const TIMER_INTS_ALARM_1_ACCESS = "RO";
pub const TIMER_INTS_ALARM_0_RESET = _u(@as(c_int, 0x0));
pub const TIMER_INTS_ALARM_0_BITS = _u(@as(c_int, 0x00000001));
pub const TIMER_INTS_ALARM_0_MSB = _u(@as(c_int, 0));
pub const TIMER_INTS_ALARM_0_LSB = _u(@as(c_int, 0));
pub const TIMER_INTS_ALARM_0_ACCESS = "RO";
pub const timer0_hw = __helpers.cast([*c]timer_hw_t, TIMER0_BASE);
pub const timer1_hw = __helpers.cast([*c]timer_hw_t, TIMER1_BASE);
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER = @as(c_int, 0);
pub inline fn TIMER_NUM(timer: anytype) @TypeOf(timer == timer1_hw) {
    _ = &timer;
    return timer == timer1_hw;
}
pub inline fn TIMER_INSTANCE(num: anytype) @TypeOf(if (num) timer1_hw else timer0_hw) {
    _ = &num;
    return if (num) timer1_hw else timer0_hw;
}
pub inline fn TIMER_ALARM_IRQ_NUM(timer: anytype, alarm_num: anytype) @TypeOf((TIMER0_IRQ_0 + (TIMER_NUM(timer) * NUM_ALARMS)) + alarm_num) {
    _ = &timer;
    _ = &alarm_num;
    return (TIMER0_IRQ_0 + (TIMER_NUM(timer) * NUM_ALARMS)) + alarm_num;
}
pub inline fn TIMER_ALARM_NUM_FROM_IRQ(irq_num: anytype) @TypeOf((irq_num - TIMER0_IRQ_0) & @as(c_uint, 3)) {
    _ = &irq_num;
    return (irq_num - TIMER0_IRQ_0) & @as(c_uint, 3);
}
pub inline fn TIMER_NUM_FROM_IRQ(irq_num: anytype) @TypeOf((irq_num - TIMER0_IRQ_0) >> @as(c_int, 2)) {
    _ = &irq_num;
    return (irq_num - TIMER0_IRQ_0) >> @as(c_int, 2);
}
pub const PICO_DEFAULT_TIMER = @as(c_int, 0);
pub const PICO_DEFAULT_TIMER_INSTANCE = @compileError("unable to translate macro: undefined identifier `timer`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_timer/include/hardware/timer.h:182:9
pub const timer_hw = PICO_DEFAULT_TIMER_INSTANCE();
pub const PARAM_ASSERTIONS_ENABLED_PICO_TIME = @as(c_int, 0);
pub const PICO_TIME_SLEEP_OVERHEAD_ADJUST_US = @as(c_int, 6);
pub const PICO_TIME_DEFAULT_ALARM_POOL_DISABLED = @as(c_int, 0);
pub const PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM = @as(c_int, 3);
pub const PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS = @as(c_int, 16);
pub const CYW43_HOST_NAME = "PicoW";
pub const CYW43_GPIO = @as(c_int, 1);
pub const CYW43_LOGIC_DEBUG = @as(c_int, 0);
pub const CYW43_USE_OTP_MAC = @as(c_int, 1);
pub const CYW43_NO_NETUTILS = @as(c_int, 1);
pub const CYW43_IOCTL_TIMEOUT_US = __helpers.promoteIntLiteral(c_int, 1000000, .decimal);
pub const CYW43_USE_STATS = @as(c_int, 0);
pub const CYW43_HAL_MAC_WLAN0 = @as(c_int, 0);
pub const STATIC = @compileError("unable to translate C expr: unexpected token 'static'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:54:9
pub const CYW43_USE_SPI = @as(c_int, 1);
pub const CYW43_SPI_PIO = @as(c_int, 1);
pub const CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE = "w43439A0_7_95_49_00_combined.h";
pub const CYW43_WIFI_NVRAM_INCLUDE_FILE = "wifi_nvram_43439.h";
pub const CYW43_EPERM = -PICO_ERROR_NOT_PERMITTED;
pub const CYW43_EIO = -PICO_ERROR_IO;
pub const CYW43_EINVAL = -PICO_ERROR_INVALID_ARG;
pub const CYW43_ETIMEDOUT = -PICO_ERROR_TIMEOUT;
pub const CYW43_NUM_GPIOS = CYW43_WL_GPIO_COUNT;
pub const cyw43_hal_pin_obj_t = uint;
pub inline fn CYW43_ARRAY_SIZE(a: anytype) @TypeOf(count_of(a)) {
    _ = &a;
    return count_of(a);
}
pub const CYW43_PIN_WL_REG_ON = CYW43_DEFAULT_PIN_WL_REG_ON;
pub const CYW43_PIN_WL_DATA_OUT = CYW43_DEFAULT_PIN_WL_DATA_OUT;
pub const CYW43_PIN_WL_DATA_IN = CYW43_DEFAULT_PIN_WL_DATA_IN;
pub const CYW43_PIN_WL_HOST_WAKE = CYW43_DEFAULT_PIN_WL_HOST_WAKE;
pub const CYW43_PIN_WL_CLOCK = CYW43_DEFAULT_PIN_WL_CLOCK;
pub const CYW43_PIN_WL_CS = CYW43_DEFAULT_PIN_WL_CS;
pub const CYW43_HAL_PIN_MODE_INPUT = GPIO_IN;
pub const CYW43_HAL_PIN_MODE_OUTPUT = GPIO_OUT;
pub const CYW43_HAL_PIN_PULL_NONE = @as(c_int, 0);
pub const CYW43_HAL_PIN_PULL_UP = @as(c_int, 1);
pub const CYW43_HAL_PIN_PULL_DOWN = @as(c_int, 2);
pub const CYW43_THREAD_ENTER = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:169:9
pub const CYW43_THREAD_EXIT = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:170:9
pub inline fn cyw43_arch_lwip_check() @TypeOf(cyw43_thread_lock_check()) {
    return cyw43_thread_lock_check();
}
pub const CYW43_THREAD_LOCK_CHECK = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:176:9
pub const CYW43_SDPCM_SEND_COMMON_WAIT = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:184:9
pub const CYW43_DO_IOCTL_WAIT = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:185:9
pub const CYW43_POST_POLL_HOOK = @compileError("unable to translate C expr: unexpected token ';'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h:195:9
pub const cyw43_malloc = malloc;
pub const cyw43_free = free;
pub const PICO_CYW43_LOGGING_ENABLED = @as(c_int, 1);
pub const CYW43_CLEAR_SDIO_INT = @as(c_int, 0);
pub const CYW43_INCLUDE_LEGACY_F1_OVERFLOW_WORKAROUND_VARIABLES = @as(c_int, 0);
pub const CYW43_ENABLE_BLUETOOTH = @as(c_int, 0);
pub const CYW43_ENABLE_BLUETOOTH_OVER_UART = @as(c_int, 0);
pub const CYW43_RESOURCE_ATTRIBUTE = @compileError("unable to translate macro: undefined identifier `aligned`"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:111:9
pub const CYW43_RESOURCE_VERIFY_DOWNLOAD = @as(c_int, 0);
pub const CYW43_SLEEP_MAX = @as(c_int, 50);
pub const CYW43_HAL_UART_READCHAR_BLOCKING_WAIT = cyw43_delay_us(@as(c_int, 10));
pub const CYW43_LWIP = @as(c_int, 1);
pub const CYW43_NETUTILS = @as(c_int, 0);
pub const _STDIO_H_ = "";
pub const _FSTDIO = "";
pub const __need_size_t = "";
pub const __need_NULL = "";
pub const __need___va_list = "";
pub const __STDC_VERSION_STDARG_H__ = @as(c_int, 0);
pub const va_start = @compileError("unable to translate macro: undefined identifier `__builtin_va_start`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdarg.h:12:9
pub const va_end = @compileError("unable to translate macro: undefined identifier `__builtin_va_end`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdarg.h:14:9
pub const va_arg = @compileError("unable to translate macro: undefined identifier `__builtin_va_arg`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdarg.h:15:9
pub const __va_copy = @compileError("unable to translate macro: undefined identifier `__builtin_va_copy`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdarg.h:18:9
pub const va_copy = @compileError("unable to translate macro: undefined identifier `__builtin_va_copy`"); // /home/ianic/.cache/zig/p/N-V-__8AANdznhSRfEOCrJA7VmtMVvEylt-TYEMgwuQZIagI/lib/compiler/aro/include/stdarg.h:22:9
pub const __GNUC_VA_LIST = @as(c_int, 1);
pub const _VA_LIST_DEFINED = "";
pub const _SYS_REENT_H_ = "";
pub const _SYS__TYPES_H = "";
pub const __need_wint_t = "";
pub const _MACHINE__TYPES_H = "";
pub const _CLOCK_T_ = c_ulong;
pub const _TIME_T_ = __int_least64_t;
pub const _CLOCKID_T_ = c_ulong;
pub const _TIMER_T_ = c_ulong;
pub const _NULL = @as(c_int, 0);
pub const __Long = c_long;
pub const __SYS_LOCK_H__ = "";
pub const _LOCK_RECURSIVE_T = _LOCK_T;
pub const __LOCK_INIT = @compileError("unable to translate macro: undefined identifier `__lock_`"); // /usr/arm-none-eabi/include/sys/lock.h:37:9
pub inline fn __LOCK_INIT_RECURSIVE(class: anytype, lock: anytype) @TypeOf(__LOCK_INIT(class, lock)) {
    _ = &class;
    _ = &lock;
    return __LOCK_INIT(class, lock);
}
pub inline fn __lock_init(lock: anytype) @TypeOf(__retarget_lock_init(&lock)) {
    _ = &lock;
    return __retarget_lock_init(&lock);
}
pub inline fn __lock_init_recursive(lock: anytype) @TypeOf(__retarget_lock_init_recursive(&lock)) {
    _ = &lock;
    return __retarget_lock_init_recursive(&lock);
}
pub inline fn __lock_close(lock: anytype) @TypeOf(__retarget_lock_close(lock)) {
    _ = &lock;
    return __retarget_lock_close(lock);
}
pub inline fn __lock_close_recursive(lock: anytype) @TypeOf(__retarget_lock_close_recursive(lock)) {
    _ = &lock;
    return __retarget_lock_close_recursive(lock);
}
pub inline fn __lock_acquire(lock: anytype) @TypeOf(__retarget_lock_acquire(lock)) {
    _ = &lock;
    return __retarget_lock_acquire(lock);
}
pub inline fn __lock_acquire_recursive(lock: anytype) @TypeOf(__retarget_lock_acquire_recursive(lock)) {
    _ = &lock;
    return __retarget_lock_acquire_recursive(lock);
}
pub inline fn __lock_try_acquire(lock: anytype) @TypeOf(__retarget_lock_try_acquire(lock)) {
    _ = &lock;
    return __retarget_lock_try_acquire(lock);
}
pub inline fn __lock_try_acquire_recursive(lock: anytype) @TypeOf(__retarget_lock_try_acquire_recursive(lock)) {
    _ = &lock;
    return __retarget_lock_try_acquire_recursive(lock);
}
pub inline fn __lock_release(lock: anytype) @TypeOf(__retarget_lock_release(lock)) {
    _ = &lock;
    return __retarget_lock_release(lock);
}
pub inline fn __lock_release_recursive(lock: anytype) @TypeOf(__retarget_lock_release_recursive(lock)) {
    _ = &lock;
    return __retarget_lock_release_recursive(lock);
}
pub const _ATEXIT_SIZE = @as(c_int, 32);
pub const _ATEXIT_INIT = @compileError("unable to translate C expr: unexpected token '{'"); // /usr/arm-none-eabi/include/sys/reent.h:106:10
pub const _REENT_SMALL_CHECK_INIT = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:146:9
pub const _RAND48_SEED_0 = @as(c_int, 0x330e);
pub const _RAND48_SEED_1 = __helpers.promoteIntLiteral(c_int, 0xabcd, .hex);
pub const _RAND48_SEED_2 = @as(c_int, 0x1234);
pub const _RAND48_MULT_0 = __helpers.promoteIntLiteral(c_int, 0xe66d, .hex);
pub const _RAND48_MULT_1 = __helpers.promoteIntLiteral(c_int, 0xdeec, .hex);
pub const _RAND48_MULT_2 = @as(c_int, 0x0005);
pub const _RAND48_ADD = @as(c_int, 0x000b);
pub const _REENT_EMERGENCY_SIZE = @as(c_int, 25);
pub const _REENT_ASCTIME_SIZE = @as(c_int, 26);
pub const _REENT_SIGNAL_SIZE = @as(c_int, 24);
pub const _REENT_INIT_RESERVED_0 = "";
pub const _REENT_INIT_RESERVED_1 = "";
pub const _REENT_INIT_RESERVED_2 = "";
pub const _REENT_INIT_RESERVED_6_7 = "";
pub const _REENT_INIT_RESERVED_8 = "";
pub const _REENT_INIT = @compileError("unable to translate C expr: unexpected token '{'"); // /usr/arm-none-eabi/include/sys/reent.h:667:9
pub const _REENT_INIT_PTR_ZEROED = @compileError("unable to translate C expr: unexpected token '{'"); // /usr/arm-none-eabi/include/sys/reent.h:714:9
pub const _REENT_CHECK_RAND48 = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:728:9
pub const _REENT_CHECK_MP = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:729:9
pub const _REENT_CHECK_TM = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:730:9
pub const _REENT_CHECK_ASCTIME_BUF = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:731:9
pub const _REENT_CHECK_EMERGENCY = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:732:9
pub const _REENT_CHECK_MISC = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:733:9
pub const _REENT_CHECK_SIGNAL_BUF = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/sys/reent.h:734:9
pub inline fn _REENT_SIGNGAM(ptr: anytype) @TypeOf(ptr.*._new._reent._gamma_signgam) {
    _ = &ptr;
    return ptr.*._new._reent._gamma_signgam;
}
pub inline fn _REENT_RAND_NEXT(ptr: anytype) @TypeOf(ptr.*._new._reent._rand_next) {
    _ = &ptr;
    return ptr.*._new._reent._rand_next;
}
pub inline fn _REENT_RAND48_SEED(ptr: anytype) @TypeOf(ptr.*._new._reent._r48._seed) {
    _ = &ptr;
    return ptr.*._new._reent._r48._seed;
}
pub inline fn _REENT_RAND48_MULT(ptr: anytype) @TypeOf(ptr.*._new._reent._r48._mult) {
    _ = &ptr;
    return ptr.*._new._reent._r48._mult;
}
pub inline fn _REENT_RAND48_ADD(ptr: anytype) @TypeOf(ptr.*._new._reent._r48._add) {
    _ = &ptr;
    return ptr.*._new._reent._r48._add;
}
pub inline fn _REENT_MP_RESULT(ptr: anytype) @TypeOf(ptr.*._result) {
    _ = &ptr;
    return ptr.*._result;
}
pub inline fn _REENT_MP_RESULT_K(ptr: anytype) @TypeOf(ptr.*._result_k) {
    _ = &ptr;
    return ptr.*._result_k;
}
pub inline fn _REENT_MP_P5S(ptr: anytype) @TypeOf(ptr.*._p5s) {
    _ = &ptr;
    return ptr.*._p5s;
}
pub inline fn _REENT_MP_FREELIST(ptr: anytype) @TypeOf(ptr.*._freelist) {
    _ = &ptr;
    return ptr.*._freelist;
}
pub inline fn _REENT_ASCTIME_BUF(ptr: anytype) @TypeOf(ptr.*._new._reent._asctime_buf) {
    _ = &ptr;
    return ptr.*._new._reent._asctime_buf;
}
pub inline fn _REENT_TM(ptr: anytype) @TypeOf(&ptr.*._new._reent._localtime_buf) {
    _ = &ptr;
    return &ptr.*._new._reent._localtime_buf;
}
pub inline fn _REENT_STRTOK_LAST(ptr: anytype) @TypeOf(ptr.*._new._reent._strtok_last) {
    _ = &ptr;
    return ptr.*._new._reent._strtok_last;
}
pub inline fn _REENT_MBLEN_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._mblen_state) {
    _ = &ptr;
    return ptr.*._new._reent._mblen_state;
}
pub inline fn _REENT_MBTOWC_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._mbtowc_state) {
    _ = &ptr;
    return ptr.*._new._reent._mbtowc_state;
}
pub inline fn _REENT_WCTOMB_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._wctomb_state) {
    _ = &ptr;
    return ptr.*._new._reent._wctomb_state;
}
pub inline fn _REENT_MBRLEN_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._mbrlen_state) {
    _ = &ptr;
    return ptr.*._new._reent._mbrlen_state;
}
pub inline fn _REENT_MBRTOWC_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._mbrtowc_state) {
    _ = &ptr;
    return ptr.*._new._reent._mbrtowc_state;
}
pub inline fn _REENT_MBSRTOWCS_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._mbsrtowcs_state) {
    _ = &ptr;
    return ptr.*._new._reent._mbsrtowcs_state;
}
pub inline fn _REENT_WCRTOMB_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._wcrtomb_state) {
    _ = &ptr;
    return ptr.*._new._reent._wcrtomb_state;
}
pub inline fn _REENT_WCSRTOMBS_STATE(ptr: anytype) @TypeOf(ptr.*._new._reent._wcsrtombs_state) {
    _ = &ptr;
    return ptr.*._new._reent._wcsrtombs_state;
}
pub inline fn _REENT_L64A_BUF(ptr: anytype) @TypeOf(ptr.*._new._reent._l64a_buf) {
    _ = &ptr;
    return ptr.*._new._reent._l64a_buf;
}
pub inline fn _REENT_SIGNAL_BUF(ptr: anytype) @TypeOf(ptr.*._new._reent._signal_buf) {
    _ = &ptr;
    return ptr.*._new._reent._signal_buf;
}
pub inline fn _REENT_GETDATE_ERR_P(ptr: anytype) @TypeOf(&ptr.*._new._reent._getdate_err) {
    _ = &ptr;
    return &ptr.*._new._reent._getdate_err;
}
pub inline fn _REENT_GETLOCALENAME_L_BUF(ptr: anytype) @TypeOf(ptr.*._new._reent._getlocalename_l_buf) {
    _ = &ptr;
    return ptr.*._new._reent._getlocalename_l_buf;
}
pub inline fn _REENT_CLEANUP(_ptr: anytype) @TypeOf(_ptr.*.__cleanup) {
    _ = &_ptr;
    return _ptr.*.__cleanup;
}
pub inline fn _REENT_CVTBUF(_ptr: anytype) @TypeOf(_ptr.*._cvtbuf) {
    _ = &_ptr;
    return _ptr.*._cvtbuf;
}
pub inline fn _REENT_CVTLEN(_ptr: anytype) @TypeOf(_ptr.*._cvtlen) {
    _ = &_ptr;
    return _ptr.*._cvtlen;
}
pub inline fn _REENT_EMERGENCY(_ptr: anytype) @TypeOf(_ptr.*._emergency) {
    _ = &_ptr;
    return _ptr.*._emergency;
}
pub inline fn _REENT_ERRNO(_ptr: anytype) @TypeOf(_ptr.*._errno) {
    _ = &_ptr;
    return _ptr.*._errno;
}
pub inline fn _REENT_INC(_ptr: anytype) @TypeOf(_ptr.*._inc) {
    _ = &_ptr;
    return _ptr.*._inc;
}
pub inline fn _REENT_LOCALE(_ptr: anytype) @TypeOf(_ptr.*._locale) {
    _ = &_ptr;
    return _ptr.*._locale;
}
pub inline fn _REENT_SIG_FUNC(_ptr: anytype) @TypeOf(_ptr.*._sig_func) {
    _ = &_ptr;
    return _ptr.*._sig_func;
}
pub inline fn _REENT_STDIN(_ptr: anytype) @TypeOf(_ptr.*._stdin) {
    _ = &_ptr;
    return _ptr.*._stdin;
}
pub inline fn _REENT_STDOUT(_ptr: anytype) @TypeOf(_ptr.*._stdout) {
    _ = &_ptr;
    return _ptr.*._stdout;
}
pub inline fn _REENT_STDERR(_ptr: anytype) @TypeOf(_ptr.*._stderr) {
    _ = &_ptr;
    return _ptr.*._stderr;
}
pub const _REENT_INIT_PTR = @compileError("unable to translate C expr: unexpected token '{'"); // /usr/arm-none-eabi/include/sys/reent.h:783:9
pub const __ATTRIBUTE_IMPURE_PTR__ = "";
pub const __ATTRIBUTE_IMPURE_DATA__ = "";
// /usr/arm-none-eabi/include/sys/reent.h:813:10: warning: macro '_REENT' contains a runtime value, translated to function
pub inline fn _REENT() @TypeOf(_impure_ptr) {
    return _impure_ptr;
}
pub inline fn _REENT_IS_NULL(_ptr: anytype) @TypeOf(_ptr == NULL) {
    _ = &_ptr;
    return _ptr == NULL;
}
// /usr/arm-none-eabi/include/sys/reent.h:818:9: warning: macro '_GLOBAL_REENT' contains a runtime value, translated to function
pub inline fn _GLOBAL_REENT() @TypeOf(&_impure_data) {
    return &_impure_data;
}
pub const _Kmax = __helpers.sizeof(usize) << @as(c_int, 3);
pub const __FILE_defined = "";
pub const _OFF_T_DECLARED = "";
pub const _SSIZE_T_DECLARED = "";
pub const _NEWLIB_STDIO_H = "";
pub inline fn _flockfile(fp: anytype) @TypeOf(if (fp.*._flags & __SSTR) @as(c_int, 0) else __lock_acquire_recursive(fp.*._lock)) {
    _ = &fp;
    return if (fp.*._flags & __SSTR) @as(c_int, 0) else __lock_acquire_recursive(fp.*._lock);
}
pub inline fn _funlockfile(fp: anytype) @TypeOf(if (fp.*._flags & __SSTR) @as(c_int, 0) else __lock_release_recursive(fp.*._lock)) {
    _ = &fp;
    return if (fp.*._flags & __SSTR) @as(c_int, 0) else __lock_release_recursive(fp.*._lock);
}
pub const __SLBF = @as(c_int, 0x0001);
pub const __SNBF = @as(c_int, 0x0002);
pub const __SRD = @as(c_int, 0x0004);
pub const __SWR = @as(c_int, 0x0008);
pub const __SRW = @as(c_int, 0x0010);
pub const __SEOF = @as(c_int, 0x0020);
pub const __SERR = @as(c_int, 0x0040);
pub const __SMBF = @as(c_int, 0x0080);
pub const __SAPP = @as(c_int, 0x0100);
pub const __SSTR = @as(c_int, 0x0200);
pub const __SOPT = @as(c_int, 0x0400);
pub const __SNPT = @as(c_int, 0x0800);
pub const __SOFF = @as(c_int, 0x1000);
pub const __SORD = @as(c_int, 0x2000);
pub const __SL64 = __helpers.promoteIntLiteral(c_int, 0x8000, .hex);
pub const __SNLK = @as(c_int, 0x0001);
pub const __SWID = @as(c_int, 0x2000);
pub const _IOFBF = @as(c_int, 0);
pub const _IOLBF = @as(c_int, 1);
pub const _IONBF = @as(c_int, 2);
pub const EOF = -@as(c_int, 1);
pub const BUFSIZ = @as(c_int, 1024);
pub const FOPEN_MAX = @as(c_int, 20);
pub const FILENAME_MAX = @as(c_int, 1024);
pub const L_tmpnam = FILENAME_MAX;
pub const P_tmpdir = "/tmp";
pub const SEEK_SET = @as(c_int, 0);
pub const SEEK_CUR = @as(c_int, 1);
pub const SEEK_END = @as(c_int, 2);
pub const TMP_MAX = @as(c_int, 26);
pub const stdin = _REENT_STDIN(_REENT);
pub const stdout = _REENT_STDOUT(_REENT);
pub const stderr = _REENT_STDERR(_REENT);
pub inline fn _stdin_r(x: anytype) @TypeOf(_REENT_STDIN(x)) {
    _ = &x;
    return _REENT_STDIN(x);
}
pub inline fn _stdout_r(x: anytype) @TypeOf(_REENT_STDOUT(x)) {
    _ = &x;
    return _REENT_STDOUT(x);
}
pub inline fn _stderr_r(x: anytype) @TypeOf(_REENT_STDERR(x)) {
    _ = &x;
    return _REENT_STDERR(x);
}
pub const __VALIST = __gnuc_va_list;
pub inline fn fropen(__cookie: anytype, __fn: anytype) @TypeOf(funopen(__cookie, __fn, NULL, NULL, NULL)) {
    _ = &__cookie;
    _ = &__fn;
    return funopen(__cookie, __fn, NULL, NULL, NULL);
}
pub inline fn fwopen(__cookie: anytype, __fn: anytype) @TypeOf(funopen(__cookie, NULL, __fn, NULL, NULL)) {
    _ = &__cookie;
    _ = &__fn;
    return funopen(__cookie, NULL, __fn, NULL, NULL);
}
pub const __sgetc_raw_r = @compileError("TODO unary inc/dec expr"); // /usr/arm-none-eabi/include/stdio.h:658:15
pub inline fn __sgetc_r(__ptr: anytype, __p: anytype) @TypeOf(__sgetc_raw_r(__ptr, __p)) {
    _ = &__ptr;
    _ = &__p;
    return __sgetc_raw_r(__ptr, __p);
}
pub inline fn __sfeof(p: anytype) c_int {
    _ = &p;
    return __helpers.cast(c_int, (p.*._flags & __SEOF) != @as(c_int, 0));
}
pub inline fn __sferror(p: anytype) c_int {
    _ = &p;
    return __helpers.cast(c_int, (p.*._flags & __SERR) != @as(c_int, 0));
}
pub const __sclearerr = @compileError("unable to translate C expr: expected ')' instead got '&='"); // /usr/arm-none-eabi/include/stdio.h:725:9
pub inline fn __sfileno(p: anytype) @TypeOf(p.*._file) {
    _ = &p;
    return p.*._file;
}
pub const fast_putc = @compileError("TODO unary inc/dec expr"); // /usr/arm-none-eabi/include/stdio.h:778:9
pub const L_ctermid = @as(c_int, 16);
pub const CYW43_PRINTF = @compileError("unable to translate C expr: unexpected token '__VA_ARGS__'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:155:9
pub inline fn CYW43_VDEBUG() anyopaque {
    return __helpers.cast(anyopaque, @as(c_int, 0));
}
pub const CYW43_VERBOSE_DEBUG = @as(c_int, 0);
pub const CYW43_DEBUG = @compileError("unable to translate C expr: unexpected token '__VA_ARGS__'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:167:9
pub const CYW43_INFO = @compileError("unable to translate C expr: unexpected token '__VA_ARGS__'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:172:9
pub const CYW43_WARN = @compileError("unable to translate C expr: expected ',' or ')' instead got '__VA_ARGS__'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:176:9
pub inline fn CYW43_FAIL_FAST_CHECK(res: anytype) @TypeOf(res) {
    _ = &res;
    return res;
}
pub const CYW43_EVENT_POLL_HOOK = "";
pub const CYW43_DEFAULT_IP_STA_ADDRESS = LWIP_MAKEU32(@as(c_int, 0), @as(c_int, 0), @as(c_int, 0), @as(c_int, 0));
pub const CYW43_DEFAULT_IP_AP_ADDRESS = LWIP_MAKEU32(@as(c_int, 192), @as(c_int, 168), @as(c_int, 4), @as(c_int, 1));
pub const CYW43_DEFAULT_IP_MASK = LWIP_MAKEU32(@as(c_int, 255), @as(c_int, 255), @as(c_int, 255), @as(c_int, 0));
pub const CYW43_DEFAULT_IP_STA_GATEWAY = LWIP_MAKEU32(@as(c_int, 192), @as(c_int, 168), @as(c_int, 0), @as(c_int, 1));
pub const CYW43_DEFAULT_IP_AP_GATEWAY = LWIP_MAKEU32(@as(c_int, 192), @as(c_int, 168), @as(c_int, 4), @as(c_int, 1));
pub const CYW43_DEFAULT_IP_DNS = LWIP_MAKEU32(@as(c_int, 8), @as(c_int, 8), @as(c_int, 8), @as(c_int, 8));
pub const CYW43_CB_TCPIP_INIT_EXTRA = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:216:9
pub const CYW43_CB_TCPIP_DEINIT_EXTRA = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/cyw43-driver/src/cyw43_config.h:222:9
pub const LWIP_HDR_NETIF_H = "";
pub const LWIP_HDR_OPT_H = "";
pub const __LWIPOPTS_H__ = "";
pub const NO_SYS = @as(c_int, 1);
pub const LWIP_SOCKET = @as(c_int, 0);
pub const MEM_LIBC_MALLOC = @as(c_int, 0);
pub const MEM_ALIGNMENT = @as(c_int, 4);
pub const MEM_SIZE = @as(c_int, 4000);
pub const MEMP_NUM_TCP_SEG = @as(c_int, 32);
pub const MEMP_NUM_ARP_QUEUE = @as(c_int, 10);
pub const PBUF_POOL_SIZE = @as(c_int, 24);
pub const LWIP_ARP = @as(c_int, 1);
pub const LWIP_ETHERNET = @as(c_int, 1);
pub const LWIP_ICMP = @as(c_int, 1);
pub const LWIP_RAW = @as(c_int, 1);
pub const TCP_WND = @as(c_int, 8) * TCP_MSS;
pub const TCP_MSS = @as(c_int, 1460);
pub const TCP_SND_BUF = @as(c_int, 8) * TCP_MSS;
pub const TCP_SND_QUEUELEN = __helpers.div((@as(c_int, 4) * TCP_SND_BUF) + (TCP_MSS - @as(c_int, 1)), TCP_MSS);
pub const LWIP_NETIF_STATUS_CALLBACK = @as(c_int, 1);
pub const LWIP_NETIF_LINK_CALLBACK = @as(c_int, 1);
pub const LWIP_NETIF_HOSTNAME = @as(c_int, 1);
pub const LWIP_NETCONN = @as(c_int, 0);
pub const MEM_STATS = @as(c_int, 0);
pub const SYS_STATS = @as(c_int, 0);
pub const MEMP_STATS = @as(c_int, 0);
pub const LINK_STATS = @as(c_int, 0);
pub const LWIP_CHKSUM_ALGORITHM = @as(c_int, 3);
pub const LWIP_DHCP = @as(c_int, 1);
pub const LWIP_IPV4 = @as(c_int, 1);
pub const LWIP_TCP = @as(c_int, 1);
pub const LWIP_UDP = @as(c_int, 1);
pub const LWIP_DNS = @as(c_int, 1);
pub const LWIP_TCP_KEEPALIVE = @as(c_int, 1);
pub const LWIP_NETIF_TX_SINGLE_PBUF = @as(c_int, 1);
pub const DHCP_DOES_ARP_CHECK = @as(c_int, 0);
pub const LWIP_DHCP_DOES_ACD_CHECK = @as(c_int, 0);
pub const LWIP_DEBUG = @as(c_int, 1);
pub const LWIP_STATS = @as(c_int, 1);
pub const LWIP_STATS_DISPLAY = @as(c_int, 1);
pub const ETHARP_DEBUG = LWIP_DBG_OFF;
pub const NETIF_DEBUG = LWIP_DBG_OFF;
pub const PBUF_DEBUG = LWIP_DBG_OFF;
pub const API_LIB_DEBUG = LWIP_DBG_OFF;
pub const API_MSG_DEBUG = LWIP_DBG_OFF;
pub const SOCKETS_DEBUG = LWIP_DBG_OFF;
pub const ICMP_DEBUG = LWIP_DBG_OFF;
pub const INET_DEBUG = LWIP_DBG_OFF;
pub const IP_DEBUG = LWIP_DBG_OFF;
pub const IP_REASS_DEBUG = LWIP_DBG_OFF;
pub const RAW_DEBUG = LWIP_DBG_OFF;
pub const MEM_DEBUG = LWIP_DBG_OFF;
pub const MEMP_DEBUG = LWIP_DBG_OFF;
pub const SYS_DEBUG = LWIP_DBG_OFF;
pub const TCP_DEBUG = LWIP_DBG_OFF;
pub const TCP_INPUT_DEBUG = LWIP_DBG_OFF;
pub const TCP_OUTPUT_DEBUG = LWIP_DBG_OFF;
pub const TCP_RTO_DEBUG = LWIP_DBG_OFF;
pub const TCP_CWND_DEBUG = LWIP_DBG_OFF;
pub const TCP_WND_DEBUG = LWIP_DBG_OFF;
pub const TCP_FR_DEBUG = LWIP_DBG_OFF;
pub const TCP_QLEN_DEBUG = LWIP_DBG_OFF;
pub const TCP_RST_DEBUG = LWIP_DBG_OFF;
pub const UDP_DEBUG = LWIP_DBG_OFF;
pub const TCPIP_DEBUG = LWIP_DBG_OFF;
pub const PPP_DEBUG = LWIP_DBG_OFF;
pub const SLIP_DEBUG = LWIP_DBG_OFF;
pub const DHCP_DEBUG = LWIP_DBG_OFF;
pub const LWIP_HDR_DEBUG_H = "";
pub const LWIP_HDR_ARCH_H = "";
pub const LITTLE_ENDIAN = _LITTLE_ENDIAN;
pub const BIG_ENDIAN = _BIG_ENDIAN;
pub const __CC_H__ = "";
pub const _SYS_TIME_H_ = "";
pub const _SYS__TIMEVAL_H_ = "";
pub const _SUSECONDS_T_DECLARED = "";
pub const __time_t_defined = "";
pub const _TIME_T_DECLARED = "";
pub const _TIMEVAL_DEFINED = "";
pub const __BIT_TYPES_DEFINED__ = @as(c_int, 1);
pub const _SYS_TYPES_H = "";
pub const _SYS__STDINT_H = "";
pub const _INT8_T_DECLARED = "";
pub const _UINT8_T_DECLARED = "";
pub const __int8_t_defined = @as(c_int, 1);
pub const _INT16_T_DECLARED = "";
pub const _UINT16_T_DECLARED = "";
pub const __int16_t_defined = @as(c_int, 1);
pub const _INT32_T_DECLARED = "";
pub const _UINT32_T_DECLARED = "";
pub const __int32_t_defined = @as(c_int, 1);
pub const _INT64_T_DECLARED = "";
pub const _UINT64_T_DECLARED = "";
pub const __int64_t_defined = @as(c_int, 1);
pub const _INTMAX_T_DECLARED = "";
pub const _UINTMAX_T_DECLARED = "";
pub const _INTPTR_T_DECLARED = "";
pub const _UINTPTR_T_DECLARED = "";
pub const __MACHINE_ENDIAN_H__ = "";
pub const _LITTLE_ENDIAN = @as(c_int, 1234);
pub const _BIG_ENDIAN = @as(c_int, 4321);
pub const _PDP_ENDIAN = @as(c_int, 3412);
pub const _BYTE_ORDER = _LITTLE_ENDIAN;
pub const _QUAD_HIGHWORD = @as(c_int, 1);
pub const _QUAD_LOWWORD = @as(c_int, 0);
pub const PDP_ENDIAN = _PDP_ENDIAN;
pub const BYTE_ORDER = _BYTE_ORDER;
pub inline fn __bswap16(_x: anytype) @TypeOf(__builtin.bswap16(_x)) {
    _ = &_x;
    return __builtin.bswap16(_x);
}
pub inline fn __bswap32(_x: anytype) @TypeOf(__builtin.bswap32(_x)) {
    _ = &_x;
    return __builtin.bswap32(_x);
}
pub inline fn __bswap64(_x: anytype) @TypeOf(__builtin.bswap64(_x)) {
    _ = &_x;
    return __builtin.bswap64(_x);
}
pub inline fn __htonl(_x: anytype) @TypeOf(__bswap32(_x)) {
    _ = &_x;
    return __bswap32(_x);
}
pub inline fn __htons(_x: anytype) @TypeOf(__bswap16(_x)) {
    _ = &_x;
    return __bswap16(_x);
}
pub inline fn __ntohl(_x: anytype) @TypeOf(__bswap32(_x)) {
    _ = &_x;
    return __bswap32(_x);
}
pub inline fn __ntohs(_x: anytype) @TypeOf(__bswap16(_x)) {
    _ = &_x;
    return __bswap16(_x);
}
pub const _SYS_SELECT_H = "";
pub const _SYS__SIGSET_H_ = "";
pub const _SYS_TIMESPEC_H_ = "";
pub const _SYS__TIMESPEC_H_ = "";
pub const TIMEVAL_TO_TIMESPEC = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/timespec.h:41:9
pub const TIMESPEC_TO_TIMEVAL = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/timespec.h:46:9
pub const _SIGSET_T_DECLARED = "";
pub const _SYS_TYPES_FD_SET = "";
pub const FD_SETSIZE = @as(c_int, 64);
pub const _NFDBITS = __helpers.cast(c_int, __helpers.sizeof(__fd_mask)) * @as(c_int, 8);
pub const NFDBITS = _NFDBITS;
pub inline fn _howmany(x: anytype, y: anytype) @TypeOf(__helpers.div(x + (y - @as(c_int, 1)), y)) {
    _ = &x;
    _ = &y;
    return __helpers.div(x + (y - @as(c_int, 1)), y);
}
pub const fds_bits = @compileError("unable to translate macro: undefined identifier `__fds_bits`"); // /usr/arm-none-eabi/include/sys/select.h:58:9
pub inline fn __fdset_mask(n: anytype) @TypeOf(__helpers.cast(__fd_mask, @as(c_int, 1)) << __helpers.rem(n, _NFDBITS)) {
    _ = &n;
    return __helpers.cast(__fd_mask, @as(c_int, 1)) << __helpers.rem(n, _NFDBITS);
}
pub const FD_CLR = @compileError("unable to translate C expr: expected ')' instead got '&='"); // /usr/arm-none-eabi/include/sys/select.h:62:9
pub const FD_COPY = @compileError("unable to translate C expr: expected ')' instead got '='"); // /usr/arm-none-eabi/include/sys/select.h:64:9
pub inline fn FD_ISSET(n: anytype, p: anytype) @TypeOf((p.*.__fds_bits[@as(usize, @intCast(__helpers.div(n, _NFDBITS)))] & __fdset_mask(n)) != @as(c_int, 0)) {
    _ = &n;
    _ = &p;
    return (p.*.__fds_bits[@as(usize, @intCast(__helpers.div(n, _NFDBITS)))] & __fdset_mask(n)) != @as(c_int, 0);
}
pub const FD_SET = @compileError("unable to translate C expr: expected ')' instead got '|='"); // /usr/arm-none-eabi/include/sys/select.h:67:9
pub const FD_ZERO = @compileError("unable to translate macro: undefined identifier `_p`"); // /usr/arm-none-eabi/include/sys/select.h:68:9
pub const physadr = @compileError("unable to translate macro: undefined identifier `physadr_t`"); // /usr/arm-none-eabi/include/sys/types.h:51:11
pub const quad = @compileError("unable to translate macro: undefined identifier `quad_t`"); // /usr/arm-none-eabi/include/sys/types.h:52:11
pub const _IN_ADDR_T_DECLARED = "";
pub const _IN_PORT_T_DECLARED = "";
pub const __u_char_defined = "";
pub const __u_short_defined = "";
pub const __u_int_defined = "";
pub const __u_long_defined = "";
pub const _BSDTYPES_DEFINED = "";
pub const _BLKCNT_T_DECLARED = "";
pub const _BLKSIZE_T_DECLARED = "";
pub const __clock_t_defined = "";
pub const _CLOCK_T_DECLARED = "";
pub const __caddr_t_defined = "";
pub const _FSBLKCNT_T_DECLARED = "";
pub const _ID_T_DECLARED = "";
pub const _INO_T_DECLARED = "";
pub const _DEV_T_DECLARED = "";
pub const _UID_T_DECLARED = "";
pub const _GID_T_DECLARED = "";
pub const _PID_T_DECLARED = "";
pub const _KEY_T_DECLARED = "";
pub const _MODE_T_DECLARED = "";
pub const _NLINK_T_DECLARED = "";
pub const __clockid_t_defined = "";
pub const _CLOCKID_T_DECLARED = "";
pub const __timer_t_defined = "";
pub const _TIMER_T_DECLARED = "";
pub const _USECONDS_T_DECLARED = "";
pub const _SYS__PTHREADTYPES_H_ = "";
pub const _SYS_SCHED_H_ = "";
pub const SCHED_OTHER = @as(c_int, 0);
pub const SCHED_FIFO = @as(c_int, 1);
pub const SCHED_RR = @as(c_int, 2);
pub const PTHREAD_SCOPE_PROCESS = @as(c_int, 0);
pub const PTHREAD_SCOPE_SYSTEM = @as(c_int, 1);
pub const PTHREAD_INHERIT_SCHED = @as(c_int, 1);
pub const PTHREAD_EXPLICIT_SCHED = @as(c_int, 2);
pub const PTHREAD_CREATE_DETACHED = @as(c_int, 0);
pub const PTHREAD_CREATE_JOINABLE = @as(c_int, 1);
pub const _PTHREAD_MUTEX_INITIALIZER = __helpers.cast(pthread_mutex_t, __helpers.promoteIntLiteral(c_int, 0xFFFFFFFF, .hex));
pub const _PTHREAD_COND_INITIALIZER = __helpers.cast(pthread_cond_t, __helpers.promoteIntLiteral(c_int, 0xFFFFFFFF, .hex));
pub const _PTHREAD_ONCE_INIT = @compileError("unable to translate C expr: unexpected token '{'"); // /usr/arm-none-eabi/include/sys/_pthreadtypes.h:197:9
pub const DST_NONE = @as(c_int, 0);
pub const DST_USA = @as(c_int, 1);
pub const DST_AUST = @as(c_int, 2);
pub const DST_WET = @as(c_int, 3);
pub const DST_MET = @as(c_int, 4);
pub const DST_EET = @as(c_int, 5);
pub const DST_CAN = @as(c_int, 6);
pub const bintime_clear = @compileError("unable to translate C expr: expected ')' instead got '='"); // /usr/arm-none-eabi/include/sys/time.h:132:9
pub inline fn bintime_isset(a: anytype) @TypeOf((a.*.sec != 0) or (a.*.frac != 0)) {
    _ = &a;
    return (a.*.sec != 0) or (a.*.frac != 0);
}
pub const bintime_cmp = @compileError("unable to translate C expr: expected ')' instead got ''"); // /usr/arm-none-eabi/include/sys/time.h:134:9
pub const SBT_1S = __helpers.cast(sbintime_t, @as(c_int, 1)) << @as(c_int, 32);
pub const SBT_1M = SBT_1S * @as(c_int, 60);
pub const SBT_1MS = __helpers.div(SBT_1S, @as(c_int, 1000));
pub const SBT_1US = __helpers.div(SBT_1S, __helpers.promoteIntLiteral(c_int, 1000000, .decimal));
pub const SBT_1NS = __helpers.div(SBT_1S, __helpers.promoteIntLiteral(c_int, 1000000000, .decimal));
pub const SBT_MAX = @as(c_longlong, 0x7fffffffffffffff);
pub const timespecclear = @compileError("unable to translate C expr: expected ')' instead got '='"); // /usr/arm-none-eabi/include/sys/time.h:345:9
pub inline fn timespecisset(tvp: anytype) @TypeOf((tvp.*.tv_sec != 0) or (tvp.*.tv_nsec != 0)) {
    _ = &tvp;
    return (tvp.*.tv_sec != 0) or (tvp.*.tv_nsec != 0);
}
pub const timespeccmp = @compileError("unable to translate C expr: expected ')' instead got ''"); // /usr/arm-none-eabi/include/sys/time.h:347:9
pub const timespecadd = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/time.h:352:9
pub const timespecsub = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/time.h:361:9
pub const timerclear = @compileError("unable to translate C expr: expected ')' instead got '='"); // /usr/arm-none-eabi/include/sys/time.h:373:9
pub inline fn timerisset(tvp: anytype) @TypeOf((tvp.*.tv_sec != 0) or (tvp.*.tv_usec != 0)) {
    _ = &tvp;
    return (tvp.*.tv_sec != 0) or (tvp.*.tv_usec != 0);
}
pub const timercmp = @compileError("unable to translate C expr: expected ')' instead got ''"); // /usr/arm-none-eabi/include/sys/time.h:375:9
pub const timeradd = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/time.h:379:9
pub const timersub = @compileError("unable to translate C expr: unexpected token 'do'"); // /usr/arm-none-eabi/include/sys/time.h:388:9
pub const ITIMER_REAL = @as(c_int, 0);
pub const ITIMER_VIRTUAL = @as(c_int, 1);
pub const ITIMER_PROF = @as(c_int, 2);
pub const _TIME_H_ = "";
pub const _MACHTIME_H_ = "";
pub const _CLOCKS_PER_SEC_ = @as(c_int, 100);
pub const CLOCKS_PER_SEC = _CLOCKS_PER_SEC_;
pub const CLK_TCK = CLOCKS_PER_SEC;
pub const _SYS__LOCALE_H = "";
// /usr/arm-none-eabi/include/time.h:142:9: warning: macro 'tzname' contains a runtime value, translated to function
pub inline fn tzname() @TypeOf(_tzname) {
    return _tzname;
}
pub const CLOCK_ENABLED = @as(c_int, 1);
pub const CLOCK_DISABLED = @as(c_int, 0);
pub const CLOCK_ALLOWED = @as(c_int, 1);
pub const CLOCK_DISALLOWED = @as(c_int, 0);
pub const TIMER_ABSTIME = @as(c_int, 4);
pub const CLOCK_REALTIME = @as(c_int, 1);
pub const PICO_LWIP_CUSTOM_LOCK_TCPIP_CORE = @as(c_int, 1);
pub const PACK_STRUCT_BEGIN = "";
pub const PACK_STRUCT_STRUCT = @compileError("unable to translate macro: undefined identifier `__packed__`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_lwip/include/arch/cc.h:74:9
pub const PACK_STRUCT_END = "";
pub inline fn PACK_STRUCT_FIELD(x: anytype) @TypeOf(x) {
    _ = &x;
    return x;
}
pub inline fn LWIP_PLATFORM_ASSERT(x: anytype) @TypeOf(panic(x)) {
    _ = &x;
    return panic(x);
}
pub const _PICO_RAND_H = "";
pub const PICO_RAND_ENTROPY_SRC_TRNG = @as(c_int, 1);
pub const PICO_RAND_ENTROPY_SRC_TIME = @as(c_int, 1);
pub const PICO_RAND_SEED_ENTROPY_SRC_ROSC = @compileError("unable to translate macro: undefined identifier `PICO_RAND_ENTROPY_SRC_ROSC`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_rand/include/pico/rand.h:98:9
pub const PICO_RAND_SEED_ENTROPY_SRC_TRNG = PICO_RAND_ENTROPY_SRC_TRNG;
pub const PICO_RAND_SEED_ENTROPY_SRC_TIME = PICO_RAND_ENTROPY_SRC_TIME;
pub const PICO_RAND_SEED_ENTROPY_SRC_BUS_PERF_COUNTER = @compileError("unable to translate macro: undefined identifier `PICO_RAND_ENTROPY_SRC_BUS_PERF_COUNTER`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_rand/include/pico/rand.h:113:9
pub const PICO_RAND_SEED_ENTROPY_SRC_BOOT_RANDOM = @as(c_int, 1);
pub const PICO_RAND_SEED_ENTROPY_SRC_BOARD_ID = !(PICO_RAND_SEED_ENTROPY_SRC_BOOT_RANDOM != 0);
pub const PICO_RAND_ROSC_BIT_SAMPLE_COUNT = @as(c_int, 1);
pub const PICO_RAND_MIN_ROSC_BIT_SAMPLE_TIME_US = @as(c_uint, 10);
pub const PICO_RAND_BUS_PERF_COUNTER_EVENT = @compileError("unable to translate macro: undefined identifier `arbiter_sram5_perf_event_access`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/pico_rand/include/pico/rand.h:162:9
pub const PICO_RAND_RAM_HASH_END = SRAM_END;
pub const PICO_RAND_RAM_HASH_START = PICO_RAND_RAM_HASH_END - @as(c_uint, 1024);
pub inline fn LWIP_RAND() @TypeOf(get_rand_32()) {
    return get_rand_32();
}
pub const LWIP_PLATFORM_DIAG = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/arch.h:81:9
pub const _STDLIB_H_ = "";
pub const __need_wchar_t = "";
pub const _MACHSTDLIB_H_ = "";
pub const _NEWLIB_ALLOCA_H = "";
pub const alloca = @compileError("unable to translate macro: undefined identifier `__builtin_alloca`"); // /usr/arm-none-eabi/include/alloca.h:16:9
pub const __compar_fn_t_defined = "";
pub const EXIT_FAILURE = @as(c_int, 1);
pub const EXIT_SUCCESS = @as(c_int, 0);
pub const RAND_MAX = __RAND_MAX;
pub const MB_CUR_MAX = __locale_mb_cur_max();
pub const strtodf = strtof;
pub const LWIP_NO_STDDEF_H = @as(c_int, 0);
pub const LWIP_NO_STDINT_H = @as(c_int, 0);
pub const LWIP_HAVE_INT64 = @as(c_int, 1);
pub const LWIP_NO_INTTYPES_H = @as(c_int, 0);
pub const __CLANG_INTTYPES_H = "";
pub const _INTTYPES_H = "";
pub const _SYS__INTSUP_H = "";
pub const __STDINT_EXP = @compileError("unable to translate macro: undefined identifier `__`"); // /usr/arm-none-eabi/include/sys/_intsup.h:16:9
pub const __int20 = @as(c_int, 2);
pub const __int20__ = @as(c_int, 2);
pub const _INTPTR_EQ_LONG = "";
pub const __INT8 = "hh";
pub const __INT16 = "h";
pub const __INT32 = "";
pub const __INT64 = "ll";
pub const __FAST8 = "hh";
pub const __FAST16 = "h";
pub const __FAST32 = "";
pub const __FAST64 = "ll";
pub const __LEAST8 = "hh";
pub const __LEAST16 = "h";
pub const __LEAST32 = "";
pub const __LEAST64 = "ll";
pub const __STRINGIFY = @compileError("unable to translate C expr: unexpected token ''"); // /usr/arm-none-eabi/include/inttypes.h:28:9
pub inline fn __PRI8(x: anytype) @TypeOf(__INT8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT8 ++ __STRINGIFY(x);
}
pub inline fn __PRI8LEAST(x: anytype) @TypeOf(__LEAST8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST8 ++ __STRINGIFY(x);
}
pub inline fn __PRI8FAST(x: anytype) @TypeOf(__FAST8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST8 ++ __STRINGIFY(x);
}
pub inline fn __SCN8(x: anytype) @TypeOf(__INT8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT8 ++ __STRINGIFY(x);
}
pub inline fn __SCN8LEAST(x: anytype) @TypeOf(__LEAST8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST8 ++ __STRINGIFY(x);
}
pub inline fn __SCN8FAST(x: anytype) @TypeOf(__FAST8 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST8 ++ __STRINGIFY(x);
}
pub const PRId8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:52:9
pub const PRIi8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:53:9
pub const PRIo8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:54:9
pub const PRIu8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:55:9
pub const PRIx8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:56:9
pub const PRIX8 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:57:9
pub const SCNd8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:62:9
pub const SCNi8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:63:9
pub const SCNo8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:64:9
pub const SCNu8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:65:9
pub const SCNx8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:66:9
pub const PRIdLEAST8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:71:9
pub const PRIiLEAST8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:72:9
pub const PRIoLEAST8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:73:9
pub const PRIuLEAST8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:74:9
pub const PRIxLEAST8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:75:9
pub const PRIXLEAST8 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:76:9
pub const SCNdLEAST8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:81:11
pub const SCNiLEAST8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:82:11
pub const SCNoLEAST8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:83:11
pub const SCNuLEAST8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:84:11
pub const SCNxLEAST8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:85:11
pub const PRIdFAST8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:89:9
pub const PRIiFAST8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:90:9
pub const PRIoFAST8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:91:9
pub const PRIuFAST8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:92:9
pub const PRIxFAST8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:93:9
pub const PRIXFAST8 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:94:9
pub const SCNdFAST8 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:99:11
pub const SCNiFAST8 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:100:11
pub const SCNoFAST8 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:101:11
pub const SCNuFAST8 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:102:11
pub const SCNxFAST8 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:103:11
pub inline fn __PRI16(x: anytype) @TypeOf(__INT16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT16 ++ __STRINGIFY(x);
}
pub inline fn __PRI16LEAST(x: anytype) @TypeOf(__LEAST16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST16 ++ __STRINGIFY(x);
}
pub inline fn __PRI16FAST(x: anytype) @TypeOf(__FAST16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST16 ++ __STRINGIFY(x);
}
pub inline fn __SCN16(x: anytype) @TypeOf(__INT16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT16 ++ __STRINGIFY(x);
}
pub inline fn __SCN16LEAST(x: anytype) @TypeOf(__LEAST16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST16 ++ __STRINGIFY(x);
}
pub inline fn __SCN16FAST(x: anytype) @TypeOf(__FAST16 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST16 ++ __STRINGIFY(x);
}
pub const PRId16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:116:9
pub const PRIi16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:117:9
pub const PRIo16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:118:9
pub const PRIu16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:119:9
pub const PRIx16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:120:9
pub const PRIX16 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:121:9
pub const SCNd16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:123:9
pub const SCNi16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:124:9
pub const SCNo16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:125:9
pub const SCNu16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:126:9
pub const SCNx16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:127:9
pub const PRIdLEAST16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:130:9
pub const PRIiLEAST16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:131:9
pub const PRIoLEAST16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:132:9
pub const PRIuLEAST16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:133:9
pub const PRIxLEAST16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:134:9
pub const PRIXLEAST16 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:135:9
pub const SCNdLEAST16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:137:9
pub const SCNiLEAST16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:138:9
pub const SCNoLEAST16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:139:9
pub const SCNuLEAST16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:140:9
pub const SCNxLEAST16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:141:9
pub const PRIdFAST16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:144:9
pub const PRIiFAST16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:145:9
pub const PRIoFAST16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:146:9
pub const PRIuFAST16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:147:9
pub const PRIxFAST16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:148:9
pub const PRIXFAST16 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:149:9
pub const SCNdFAST16 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:151:9
pub const SCNiFAST16 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:152:9
pub const SCNoFAST16 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:153:9
pub const SCNuFAST16 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:154:9
pub const SCNxFAST16 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:155:9
pub inline fn __PRI32(x: anytype) @TypeOf(__INT32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT32 ++ __STRINGIFY(x);
}
pub inline fn __SCN32(x: anytype) @TypeOf(__INT32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT32 ++ __STRINGIFY(x);
}
pub inline fn __PRI32LEAST(x: anytype) @TypeOf(__LEAST32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST32 ++ __STRINGIFY(x);
}
pub inline fn __SCN32LEAST(x: anytype) @TypeOf(__LEAST32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST32 ++ __STRINGIFY(x);
}
pub inline fn __PRI32FAST(x: anytype) @TypeOf(__FAST32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST32 ++ __STRINGIFY(x);
}
pub inline fn __SCN32FAST(x: anytype) @TypeOf(__FAST32 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST32 ++ __STRINGIFY(x);
}
pub const PRId32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:165:9
pub const PRIi32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:166:9
pub const PRIo32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:167:9
pub const PRIu32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:168:9
pub const PRIx32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:169:9
pub const PRIX32 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:170:9
pub const SCNd32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:172:9
pub const SCNi32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:173:9
pub const SCNo32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:174:9
pub const SCNu32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:175:9
pub const SCNx32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:176:9
pub const PRIdLEAST32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:179:9
pub const PRIiLEAST32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:180:9
pub const PRIoLEAST32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:181:9
pub const PRIuLEAST32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:182:9
pub const PRIxLEAST32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:183:9
pub const PRIXLEAST32 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:184:9
pub const SCNdLEAST32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:186:9
pub const SCNiLEAST32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:187:9
pub const SCNoLEAST32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:188:9
pub const SCNuLEAST32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:189:9
pub const SCNxLEAST32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:190:9
pub const PRIdFAST32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:193:9
pub const PRIiFAST32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:194:9
pub const PRIoFAST32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:195:9
pub const PRIuFAST32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:196:9
pub const PRIxFAST32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:197:9
pub const PRIXFAST32 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:198:9
pub const SCNdFAST32 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:200:9
pub const SCNiFAST32 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:201:9
pub const SCNoFAST32 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:202:9
pub const SCNuFAST32 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:203:9
pub const SCNxFAST32 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:204:9
pub inline fn __PRI64(x: anytype) @TypeOf(__INT64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT64 ++ __STRINGIFY(x);
}
pub inline fn __SCN64(x: anytype) @TypeOf(__INT64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __INT64 ++ __STRINGIFY(x);
}
pub inline fn __PRI64LEAST(x: anytype) @TypeOf(__LEAST64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST64 ++ __STRINGIFY(x);
}
pub inline fn __SCN64LEAST(x: anytype) @TypeOf(__LEAST64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __LEAST64 ++ __STRINGIFY(x);
}
pub inline fn __PRI64FAST(x: anytype) @TypeOf(__FAST64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST64 ++ __STRINGIFY(x);
}
pub inline fn __SCN64FAST(x: anytype) @TypeOf(__FAST64 ++ __STRINGIFY(x)) {
    _ = &x;
    return __FAST64 ++ __STRINGIFY(x);
}
pub const PRId64 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:217:9
pub const PRIi64 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:218:9
pub const PRIo64 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:219:9
pub const PRIu64 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:220:9
pub const PRIx64 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:221:9
pub const PRIX64 = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:222:9
pub const SCNd64 = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:224:9
pub const SCNi64 = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:225:9
pub const SCNo64 = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:226:9
pub const SCNu64 = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:227:9
pub const SCNx64 = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:228:9
pub const __PRIMAX = @compileError("unable to translate macro: undefined identifier `ll`"); // /usr/arm-none-eabi/include/inttypes.h:266:9
pub const __SCNMAX = @compileError("unable to translate macro: undefined identifier `ll`"); // /usr/arm-none-eabi/include/inttypes.h:267:9
pub const PRIdMAX = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:273:9
pub const PRIiMAX = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:274:9
pub const PRIoMAX = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:275:9
pub const PRIuMAX = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:276:9
pub const PRIxMAX = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:277:9
pub const PRIXMAX = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:278:9
pub const SCNdMAX = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:280:9
pub const SCNiMAX = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:281:9
pub const SCNoMAX = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:282:9
pub const SCNuMAX = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:283:9
pub const SCNxMAX = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:284:9
pub const __PRIPTR = @compileError("unable to translate macro: undefined identifier `l`"); // /usr/arm-none-eabi/include/inttypes.h:291:10
pub const __SCNPTR = @compileError("unable to translate macro: undefined identifier `l`"); // /usr/arm-none-eabi/include/inttypes.h:292:10
pub const PRIdPTR = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:298:9
pub const PRIiPTR = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:299:9
pub const PRIoPTR = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:300:9
pub const PRIuPTR = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:301:9
pub const PRIxPTR = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:302:9
pub const PRIXPTR = @compileError("unable to translate macro: undefined identifier `X`"); // /usr/arm-none-eabi/include/inttypes.h:303:9
pub const SCNdPTR = @compileError("unable to translate macro: undefined identifier `d`"); // /usr/arm-none-eabi/include/inttypes.h:305:9
pub const SCNiPTR = @compileError("unable to translate macro: undefined identifier `i`"); // /usr/arm-none-eabi/include/inttypes.h:306:9
pub const SCNoPTR = @compileError("unable to translate macro: undefined identifier `o`"); // /usr/arm-none-eabi/include/inttypes.h:307:9
pub const SCNuPTR = @compileError("unable to translate macro: undefined identifier `u`"); // /usr/arm-none-eabi/include/inttypes.h:308:9
pub const SCNxPTR = @compileError("unable to translate macro: undefined identifier `x`"); // /usr/arm-none-eabi/include/inttypes.h:309:9
pub const X8_F = "02" ++ PRIx8;
pub const U16_F = PRIu16;
pub const S16_F = PRId16;
pub const X16_F = PRIx16;
pub const U32_F = PRIu32;
pub const S32_F = PRId32;
pub const X32_F = PRIx32;
pub const SZT_F = PRIuPTR;
pub const LWIP_NO_LIMITS_H = @as(c_int, 0);
pub const _GCC_LIMITS_H_ = "";
pub const SCHAR_MAX = __SCHAR_MAX__;
pub const SHRT_MAX = __SHRT_MAX__;
pub const INT_MAX = __INT_MAX__;
pub const LONG_MAX = __LONG_MAX__;
pub const SCHAR_MIN = -__SCHAR_MAX__ - @as(c_int, 1);
pub const SHRT_MIN = -__SHRT_MAX__ - @as(c_int, 1);
pub const INT_MIN = -__INT_MAX__ - @as(c_int, 1);
pub const LONG_MIN = -__LONG_MAX__ - @as(c_long, 1);
pub const UCHAR_MAX = (__SCHAR_MAX__ * @as(c_int, 2)) + @as(c_int, 1);
pub const USHRT_MAX = (__SHRT_MAX__ * @as(c_int, 2)) + @as(c_int, 1);
pub const UINT_MAX = (__INT_MAX__ * @as(c_uint, 2)) + @as(c_uint, 1);
pub const ULONG_MAX = (__LONG_MAX__ * @as(c_ulong, 2)) + @as(c_ulong, 1);
pub const MB_LEN_MAX = @as(c_int, 1);
pub const CHAR_BIT = __CHAR_BIT__;
pub const CHAR_MIN = @as(c_int, 0);
pub const CHAR_MAX = UCHAR_MAX;
pub const LLONG_MIN = -__LONG_LONG_MAX__ - @as(c_longlong, 1);
pub const LLONG_MAX = __LONG_LONG_MAX__;
pub const ULLONG_MAX = (__LONG_LONG_MAX__ * @as(c_ulonglong, 2)) + @as(c_ulonglong, 1);
pub const SSIZE_MAX = INT_MAX;
pub const LWIP_UINT32_MAX = __helpers.promoteIntLiteral(c_int, 0xffffffff, .hex);
pub const LWIP_NO_CTYPE_H = @as(c_int, 0);
pub const _CTYPE_H_ = "";
pub inline fn _tolower(__c: anytype) @TypeOf((__helpers.cast(u8, __c) - 'A') + 'a') {
    _ = &__c;
    return (__helpers.cast(u8, __c) - 'A') + 'a';
}
pub inline fn _toupper(__c: anytype) @TypeOf((__helpers.cast(u8, __c) - 'a') + 'A') {
    _ = &__c;
    return (__helpers.cast(u8, __c) - 'a') + 'A';
}
pub const _U = @as(c_int, 0o1);
pub const _L = @as(c_int, 0o2);
pub const _N = @as(c_int, 0o4);
pub const _S = @as(c_int, 0o10);
pub const _P = @as(c_int, 0o20);
pub const _C = @as(c_int, 0o40);
pub const _X = @as(c_int, 0o100);
pub const _B = @as(c_int, 0o200);
pub inline fn __locale_ctype_ptr() @TypeOf(_ctype_) {
    return _ctype_;
}
pub const __CTYPE_PTR = __locale_ctype_ptr();
pub const __ctype_lookup = @compileError("unable to translate C expr: unexpected token 'a string literal'"); // /usr/arm-none-eabi/include/ctype.h:90:9
pub const __ctype_lookup_l = @compileError("unable to translate C expr: unexpected token 'a string literal'"); // /usr/arm-none-eabi/include/ctype.h:121:9
pub inline fn lwip_isdigit(c: anytype) @TypeOf(isdigit(__helpers.cast(u8, c))) {
    _ = &c;
    return isdigit(__helpers.cast(u8, c));
}
pub inline fn lwip_isxdigit(c: anytype) @TypeOf(isxdigit(__helpers.cast(u8, c))) {
    _ = &c;
    return isxdigit(__helpers.cast(u8, c));
}
pub inline fn lwip_islower(c: anytype) @TypeOf(islower(__helpers.cast(u8, c))) {
    _ = &c;
    return islower(__helpers.cast(u8, c));
}
pub inline fn lwip_isspace(c: anytype) @TypeOf(isspace(__helpers.cast(u8, c))) {
    _ = &c;
    return isspace(__helpers.cast(u8, c));
}
pub inline fn lwip_isupper(c: anytype) @TypeOf(isupper(__helpers.cast(u8, c))) {
    _ = &c;
    return isupper(__helpers.cast(u8, c));
}
pub inline fn lwip_tolower(c: anytype) @TypeOf(tolower(__helpers.cast(u8, c))) {
    _ = &c;
    return tolower(__helpers.cast(u8, c));
}
pub inline fn lwip_toupper(c: anytype) @TypeOf(toupper(__helpers.cast(u8, c))) {
    _ = &c;
    return toupper(__helpers.cast(u8, c));
}
pub inline fn LWIP_CONST_CAST(target_type: anytype, val: anytype) @TypeOf(target_type(__helpers.cast(ptrdiff_t, val))) {
    _ = &target_type;
    _ = &val;
    return target_type(__helpers.cast(ptrdiff_t, val));
}
pub inline fn LWIP_ALIGNMENT_CAST(target_type: anytype, val: anytype) @TypeOf(LWIP_CONST_CAST(target_type, val)) {
    _ = &target_type;
    _ = &val;
    return LWIP_CONST_CAST(target_type, val);
}
pub inline fn LWIP_PTR_NUMERIC_CAST(target_type: anytype, val: anytype) @TypeOf(LWIP_CONST_CAST(target_type, val)) {
    _ = &target_type;
    _ = &val;
    return LWIP_CONST_CAST(target_type, val);
}
pub inline fn LWIP_PACKED_CAST(target_type: anytype, val: anytype) @TypeOf(LWIP_CONST_CAST(target_type, val)) {
    _ = &target_type;
    _ = &val;
    return LWIP_CONST_CAST(target_type, val);
}
pub const LWIP_DECLARE_MEMORY_ALIGNED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/arch.h:271:9
pub inline fn LWIP_MEM_ALIGN_SIZE(size: anytype) @TypeOf(((size + MEM_ALIGNMENT) - @as(c_uint, 1)) & ~(MEM_ALIGNMENT - @as(c_uint, 1))) {
    _ = &size;
    return ((size + MEM_ALIGNMENT) - @as(c_uint, 1)) & ~(MEM_ALIGNMENT - @as(c_uint, 1));
}
pub inline fn LWIP_MEM_ALIGN_BUFFER(size: anytype) @TypeOf((size + MEM_ALIGNMENT) - @as(c_uint, 1)) {
    _ = &size;
    return (size + MEM_ALIGNMENT) - @as(c_uint, 1);
}
pub inline fn LWIP_MEM_ALIGN(addr: anytype) ?*anyopaque {
    _ = &addr;
    return __helpers.cast(?*anyopaque, ((__helpers.cast(mem_ptr_t, addr) + MEM_ALIGNMENT) - @as(c_int, 1)) & ~__helpers.cast(mem_ptr_t, MEM_ALIGNMENT - @as(c_int, 1)));
}
pub inline fn PACK_STRUCT_FLD_8(x: anytype) @TypeOf(PACK_STRUCT_FIELD(x)) {
    _ = &x;
    return PACK_STRUCT_FIELD(x);
}
pub inline fn PACK_STRUCT_FLD_S(x: anytype) @TypeOf(PACK_STRUCT_FIELD(x)) {
    _ = &x;
    return PACK_STRUCT_FIELD(x);
}
pub inline fn LWIP_UNUSED_ARG(x: anytype) anyopaque {
    _ = &x;
    return __helpers.cast(anyopaque, x);
}
pub const LWIP_DBG_LEVEL_ALL = @as(c_int, 0x00);
pub const LWIP_DBG_LEVEL_WARNING = @as(c_int, 0x01);
pub const LWIP_DBG_LEVEL_SERIOUS = @as(c_int, 0x02);
pub const LWIP_DBG_LEVEL_SEVERE = @as(c_int, 0x03);
pub const LWIP_DBG_MASK_LEVEL = @as(c_int, 0x03);
pub const LWIP_DBG_LEVEL_OFF = LWIP_DBG_LEVEL_ALL;
pub const LWIP_DBG_ON = @as(c_uint, 0x80);
pub const LWIP_DBG_OFF = @as(c_uint, 0x00);
pub const LWIP_DBG_TRACE = @as(c_uint, 0x40);
pub const LWIP_DBG_STATE = @as(c_uint, 0x20);
pub const LWIP_DBG_FRESH = @as(c_uint, 0x10);
pub const LWIP_DBG_HALT = @as(c_uint, 0x08);
pub const LWIP_ASSERT = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/debug.h:116:9
pub inline fn LWIP_PLATFORM_ERROR(message: anytype) @TypeOf(LWIP_PLATFORM_DIAG(message)) {
    _ = &message;
    return LWIP_PLATFORM_DIAG(message);
}
pub const LWIP_ERROR = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/debug.h:130:9
pub inline fn LWIP_DEBUG_ENABLED(debug: anytype) @TypeOf((((debug & LWIP_DBG_ON) != 0) and ((debug & LWIP_DBG_TYPES_ON) != 0)) and (__helpers.cast(s16_t, debug & LWIP_DBG_MASK_LEVEL) >= LWIP_DBG_MIN_LEVEL)) {
    _ = &debug;
    return (((debug & LWIP_DBG_ON) != 0) and ((debug & LWIP_DBG_TYPES_ON) != 0)) and (__helpers.cast(s16_t, debug & LWIP_DBG_MASK_LEVEL) >= LWIP_DBG_MIN_LEVEL);
}
pub const LWIP_DEBUGF = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/debug.h:147:9
pub const LWIP_TIMERS = @as(c_int, 1);
pub const LWIP_TIMERS_CUSTOM = @as(c_int, 0);
pub inline fn MEMCPY(dst: anytype, src: anytype, len: anytype) @TypeOf(memcpy(dst, src, len)) {
    _ = &dst;
    _ = &src;
    _ = &len;
    return memcpy(dst, src, len);
}
pub inline fn SMEMCPY(dst: anytype, src: anytype, len: anytype) @TypeOf(memcpy(dst, src, len)) {
    _ = &dst;
    _ = &src;
    _ = &len;
    return memcpy(dst, src, len);
}
pub inline fn MEMMOVE(dst: anytype, src: anytype, len: anytype) @TypeOf(memmove(dst, src, len)) {
    _ = &dst;
    _ = &src;
    _ = &len;
    return memmove(dst, src, len);
}
pub const LWIP_MPU_COMPATIBLE = @as(c_int, 0);
pub const LWIP_TCPIP_CORE_LOCKING = @as(c_int, 1);
pub const LWIP_TCPIP_CORE_LOCKING_INPUT = @as(c_int, 0);
pub const SYS_LIGHTWEIGHT_PROT = @as(c_int, 1);
pub const LWIP_ASSERT_CORE_LOCKED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/opt.h:227:9
pub const MEM_CUSTOM_ALLOCATOR = @as(c_int, 0);
pub const MEMP_MEM_MALLOC = @as(c_int, 0);
pub const MEMP_MEM_INIT = @as(c_int, 0);
pub const MEMP_OVERFLOW_CHECK = @as(c_int, 0);
pub const MEMP_SANITY_CHECK = @as(c_int, 0);
pub const MEM_OVERFLOW_CHECK = @as(c_int, 0);
pub const MEM_SANITY_CHECK = @as(c_int, 0);
pub const MEM_USE_POOLS = @as(c_int, 0);
pub const MEM_USE_POOLS_TRY_BIGGER_POOL = @as(c_int, 0);
pub const MEMP_USE_CUSTOM_POOLS = @as(c_int, 0);
pub const LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT = @as(c_int, 0);
pub const MEMP_NUM_PBUF = @as(c_int, 16);
pub const MEMP_NUM_RAW_PCB = @as(c_int, 4);
pub const MEMP_NUM_UDP_PCB = @as(c_int, 4);
pub const MEMP_NUM_TCP_PCB = @as(c_int, 5);
pub const MEMP_NUM_TCP_PCB_LISTEN = @as(c_int, 8);
pub const MEMP_NUM_ALTCP_PCB = MEMP_NUM_TCP_PCB;
pub const MEMP_NUM_REASSDATA = @as(c_int, 5);
pub const MEMP_NUM_FRAG_PBUF = @as(c_int, 15);
pub const MEMP_NUM_IGMP_GROUP = @as(c_int, 8);
pub const LWIP_NUM_SYS_TIMEOUT_INTERNAL = @compileError("unable to translate macro: undefined identifier `PPP_NUM_TIMEOUTS`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/opt.h:519:9
pub const MEMP_NUM_SYS_TIMEOUT = LWIP_NUM_SYS_TIMEOUT_INTERNAL;
pub const MEMP_NUM_NETBUF = @as(c_int, 2);
pub const MEMP_NUM_NETCONN = @as(c_int, 4);
pub const MEMP_NUM_SELECT_CB = @as(c_int, 4);
pub const MEMP_NUM_TCPIP_MSG_API = @as(c_int, 8);
pub const MEMP_NUM_TCPIP_MSG_INPKT = @as(c_int, 8);
pub const MEMP_NUM_NETDB = @as(c_int, 1);
pub const MEMP_NUM_LOCALHOSTLIST = @as(c_int, 1);
pub const MEMP_NUM_API_MSG = MEMP_NUM_TCPIP_MSG_API;
pub const MEMP_NUM_DNS_API_MSG = MEMP_NUM_TCPIP_MSG_API;
pub const MEMP_NUM_SOCKET_SETGETSOCKOPT_DATA = MEMP_NUM_TCPIP_MSG_API;
pub const MEMP_NUM_NETIFAPI_MSG = MEMP_NUM_TCPIP_MSG_API;
pub const ARP_TABLE_SIZE = @as(c_int, 10);
pub const ARP_MAXAGE = @as(c_int, 300);
pub const ARP_QUEUEING = @as(c_int, 0);
pub const ARP_QUEUE_LEN = @as(c_int, 3);
pub const ETHARP_SUPPORT_VLAN = @as(c_int, 0);
pub const LWIP_VLAN_PCP = @as(c_int, 0);
pub const ETH_PAD_SIZE = @as(c_int, 0);
pub const ETHARP_SUPPORT_STATIC_ENTRIES = @as(c_int, 0);
pub const ETHARP_TABLE_MATCH_NETIF = !(LWIP_SINGLE_NETIF != 0);
pub const IP_FORWARD = @as(c_int, 0);
pub const IP_REASSEMBLY = @as(c_int, 1);
pub const IP_FRAG = @as(c_int, 1);
pub const IP_OPTIONS_ALLOWED = @as(c_int, 1);
pub const IP_REASS_MAXAGE = @as(c_int, 15);
pub const IP_REASS_MAX_PBUFS = @as(c_int, 10);
pub const IP_DEFAULT_TTL = @as(c_int, 255);
pub const IP_SOF_BROADCAST = @as(c_int, 0);
pub const IP_SOF_BROADCAST_RECV = @as(c_int, 0);
pub const IP_FORWARD_ALLOW_TX_ON_RX_NETIF = @as(c_int, 0);
pub const ICMP_TTL = IP_DEFAULT_TTL;
pub const LWIP_BROADCAST_PING = @as(c_int, 0);
pub const LWIP_MULTICAST_PING = @as(c_int, 0);
pub const RAW_TTL = IP_DEFAULT_TTL;
pub const LWIP_DHCP_BOOTP_FILE = @as(c_int, 0);
pub const LWIP_DHCP_GET_NTP_SRV = @as(c_int, 0);
pub const LWIP_DHCP_MAX_NTP_SERVERS = @as(c_int, 1);
pub const LWIP_DHCP_MAX_DNS_SERVERS = DNS_MAX_SERVERS;
pub const LWIP_DHCP_DISCOVER_ADD_HOSTNAME = @as(c_int, 1);
pub const LWIP_AUTOIP = @as(c_int, 0);
pub const LWIP_DHCP_AUTOIP_COOP = @as(c_int, 0);
pub const LWIP_DHCP_AUTOIP_COOP_TRIES = @as(c_int, 9);
pub const LWIP_ACD = (LWIP_AUTOIP != 0) or (LWIP_DHCP_DOES_ACD_CHECK != 0);
pub const LWIP_MIB2_CALLBACKS = @as(c_int, 0);
pub const LWIP_MULTICAST_TX_OPTIONS = ((LWIP_IGMP != 0) or (LWIP_IPV6_MLD != 0)) and ((LWIP_UDP != 0) or (LWIP_RAW != 0));
pub const LWIP_IGMP = @as(c_int, 0);
pub const DNS_TABLE_SIZE = @as(c_int, 4);
pub const DNS_MAX_NAME_LENGTH = @as(c_int, 256);
pub const DNS_MAX_SERVERS = @as(c_int, 2);
pub const DNS_MAX_RETRIES = @as(c_int, 4);
pub const DNS_DOES_NAME_CHECK = @as(c_int, 1);
pub const LWIP_DNS_SECURE = (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING) | LWIP_DNS_SECURE_RAND_SRC_PORT;
pub const LWIP_DNS_SECURE_RAND_XID = @as(c_int, 1);
pub const LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING = @as(c_int, 2);
pub const LWIP_DNS_SECURE_RAND_SRC_PORT = @as(c_int, 4);
pub const DNS_LOCAL_HOSTLIST = @as(c_int, 0);
pub const DNS_LOCAL_HOSTLIST_IS_DYNAMIC = @as(c_int, 0);
pub const LWIP_DNS_SUPPORT_MDNS_QUERIES = @as(c_int, 0);
pub const LWIP_UDPLITE = @as(c_int, 0);
pub const UDP_TTL = IP_DEFAULT_TTL;
pub const LWIP_NETBUF_RECVINFO = @as(c_int, 0);
pub const TCP_TTL = IP_DEFAULT_TTL;
pub const TCP_MAXRTX = @as(c_int, 12);
pub const TCP_SYNMAXRTX = @as(c_int, 6);
pub const TCP_QUEUE_OOSEQ = LWIP_TCP;
pub const LWIP_TCP_SACK_OUT = @as(c_int, 0);
pub const LWIP_TCP_MAX_SACK_NUM = @as(c_int, 4);
pub const TCP_CALCULATE_EFF_SEND_MSS = @as(c_int, 1);
pub const LWIP_TCP_RTO_TIME = @as(c_int, 3000);
pub const TCP_SNDLOWAT = LWIP_MIN(LWIP_MAX(__helpers.div(TCP_SND_BUF, @as(c_int, 2)), (@as(c_int, 2) * TCP_MSS) + @as(c_int, 1)), TCP_SND_BUF - @as(c_int, 1));
pub const TCP_SNDQUEUELOWAT = LWIP_MAX(__helpers.div(TCP_SND_QUEUELEN, @as(c_int, 2)), @as(c_int, 5));
pub const TCP_OOSEQ_MAX_BYTES = @as(c_int, 0);
pub const TCP_OOSEQ_MAX_PBUFS = @as(c_int, 0);
pub const TCP_LISTEN_BACKLOG = @as(c_int, 0);
pub const TCP_DEFAULT_LISTEN_BACKLOG = @as(c_int, 0xff);
pub const TCP_OVERSIZE = TCP_MSS;
pub const LWIP_TCP_TIMESTAMPS = @as(c_int, 0);
pub const TCP_WND_UPDATE_THRESHOLD = LWIP_MIN(__helpers.div(TCP_WND, @as(c_int, 4)), TCP_MSS * @as(c_int, 4));
pub const LWIP_EVENT_API = @as(c_int, 0);
pub const LWIP_CALLBACK_API = @as(c_int, 1);
pub const LWIP_WND_SCALE = @as(c_int, 0);
pub const TCP_RCV_SCALE = @as(c_int, 0);
pub const LWIP_TCP_PCB_NUM_EXT_ARGS = @as(c_int, 0);
pub const LWIP_ALTCP = @as(c_int, 0);
pub const LWIP_ALTCP_TLS = @as(c_int, 0);
pub const PBUF_LINK_HLEN = @as(c_int, 14) + ETH_PAD_SIZE;
pub const PBUF_LINK_ENCAPSULATION_HLEN = @as(c_int, 0);
pub const PBUF_POOL_BUFSIZE = LWIP_MEM_ALIGN_SIZE((((TCP_MSS + PBUF_IP_HLEN) + PBUF_TRANSPORT_HLEN) + PBUF_LINK_ENCAPSULATION_HLEN) + PBUF_LINK_HLEN);
pub const LWIP_PBUF_REF_T = u8_t;
pub const LWIP_PBUF_CUSTOM_DATA = "";
pub const LWIP_PBUF_CUSTOM_DATA_INIT = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/opt.h:1644:9
pub const LWIP_SINGLE_NETIF = @as(c_int, 0);
pub const LWIP_NETIF_API = @as(c_int, 0);
pub const LWIP_NETIF_EXT_STATUS_CALLBACK = @as(c_int, 0);
pub const LWIP_NETIF_REMOVE_CALLBACK = @as(c_int, 0);
pub const LWIP_NETIF_HWADDRHINT = @as(c_int, 0);
pub const LWIP_NUM_NETIF_CLIENT_DATA = @as(c_int, 0);
pub const LWIP_HAVE_LOOPIF = (LWIP_NETIF_LOOPBACK != 0) and !(LWIP_SINGLE_NETIF != 0);
pub const LWIP_LOOPIF_MULTICAST = @as(c_int, 0);
pub const LWIP_NETIF_LOOPBACK = @as(c_int, 0);
pub const LWIP_LOOPBACK_MAX_PBUFS = @as(c_int, 0);
pub const LWIP_NETIF_LOOPBACK_MULTITHREADING = !(NO_SYS != 0);
pub const TCPIP_THREAD_NAME = "tcpip_thread";
pub const TCPIP_THREAD_STACKSIZE = @as(c_int, 0);
pub const TCPIP_THREAD_PRIO = @as(c_int, 1);
pub const TCPIP_MBOX_SIZE = @as(c_int, 0);
pub const LWIP_TCPIP_THREAD_ALIVE = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/opt.h:1872:9
pub const SLIPIF_THREAD_NAME = "slipif_loop";
pub const SLIPIF_THREAD_STACKSIZE = @as(c_int, 0);
pub const SLIPIF_THREAD_PRIO = @as(c_int, 1);
pub const DEFAULT_THREAD_NAME = "lwIP";
pub const DEFAULT_THREAD_STACKSIZE = @as(c_int, 0);
pub const DEFAULT_THREAD_PRIO = @as(c_int, 1);
pub const DEFAULT_RAW_RECVMBOX_SIZE = @as(c_int, 0);
pub const DEFAULT_UDP_RECVMBOX_SIZE = @as(c_int, 0);
pub const DEFAULT_TCP_RECVMBOX_SIZE = @as(c_int, 0);
pub const DEFAULT_ACCEPTMBOX_SIZE = @as(c_int, 0);
pub const LWIP_TCPIP_TIMEOUT = @as(c_int, 0);
pub const LWIP_NETCONN_SEM_PER_THREAD = @as(c_int, 0);
pub const LWIP_NETCONN_FULLDUPLEX = @as(c_int, 0);
pub const LWIP_COMPAT_SOCKETS = @as(c_int, 1);
pub const LWIP_POSIX_SOCKETS_IO_NAMES = @as(c_int, 1);
pub const LWIP_SOCKET_OFFSET = @as(c_int, 0);
pub const LWIP_SOCKET_EXTERNAL_HEADERS = @as(c_int, 0);
pub const LWIP_SO_SNDTIMEO = @as(c_int, 0);
pub const LWIP_SO_RCVTIMEO = @as(c_int, 0);
pub const LWIP_SO_SNDRCVTIMEO_NONSTANDARD = @as(c_int, 0);
pub const LWIP_SO_RCVBUF = @as(c_int, 0);
pub const LWIP_SO_LINGER = @as(c_int, 0);
pub const RECV_BUFSIZE_DEFAULT = INT_MAX;
pub const LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT = @as(c_int, 20000);
pub const SO_REUSE = @as(c_int, 0);
pub const SO_REUSE_RXTOALL = @as(c_int, 0);
pub const LWIP_FIONREAD_LINUXMODE = @as(c_int, 0);
pub const LWIP_SOCKET_SELECT = @as(c_int, 1);
pub const LWIP_SOCKET_POLL = @as(c_int, 1);
pub const ETHARP_STATS = LWIP_ARP;
pub const IP_STATS = @as(c_int, 1);
pub const IPFRAG_STATS = (IP_REASSEMBLY != 0) or (IP_FRAG != 0);
pub const ICMP_STATS = @as(c_int, 1);
pub const IGMP_STATS = LWIP_IGMP;
pub const UDP_STATS = LWIP_UDP;
pub const TCP_STATS = LWIP_TCP;
pub const IP6_STATS = LWIP_IPV6;
pub const ICMP6_STATS = (LWIP_IPV6 != 0) and (LWIP_ICMP6 != 0);
pub const IP6_FRAG_STATS = (LWIP_IPV6 != 0) and ((LWIP_IPV6_FRAG != 0) or (LWIP_IPV6_REASS != 0));
pub const MLD6_STATS = (LWIP_IPV6 != 0) and (LWIP_IPV6_MLD != 0);
pub const ND6_STATS = LWIP_IPV6;
pub const MIB2_STATS = @as(c_int, 0);
pub const LWIP_CHECKSUM_CTRL_PER_NETIF = @as(c_int, 0);
pub const CHECKSUM_GEN_IP = @as(c_int, 1);
pub const CHECKSUM_GEN_UDP = @as(c_int, 1);
pub const CHECKSUM_GEN_TCP = @as(c_int, 1);
pub const CHECKSUM_GEN_ICMP = @as(c_int, 1);
pub const CHECKSUM_GEN_ICMP6 = @as(c_int, 1);
pub const CHECKSUM_CHECK_IP = @as(c_int, 1);
pub const CHECKSUM_CHECK_UDP = @as(c_int, 1);
pub const CHECKSUM_CHECK_TCP = @as(c_int, 1);
pub const CHECKSUM_CHECK_ICMP = @as(c_int, 1);
pub const CHECKSUM_CHECK_ICMP6 = @as(c_int, 1);
pub const LWIP_CHECKSUM_ON_COPY = @as(c_int, 0);
pub const LWIP_IPV6 = @as(c_int, 0);
pub const IPV6_REASS_MAXAGE = @as(c_int, 60);
pub const LWIP_IPV6_SCOPES = (LWIP_IPV6 != 0) and !(LWIP_SINGLE_NETIF != 0);
pub const LWIP_IPV6_SCOPES_DEBUG = @as(c_int, 0);
pub const LWIP_IPV6_NUM_ADDRESSES = @as(c_int, 3);
pub const LWIP_IPV6_FORWARD = @as(c_int, 0);
pub const LWIP_IPV6_FRAG = @as(c_int, 1);
pub const LWIP_IPV6_REASS = LWIP_IPV6;
pub const LWIP_IPV6_SEND_ROUTER_SOLICIT = LWIP_IPV6;
pub const LWIP_IPV6_AUTOCONFIG = LWIP_IPV6;
pub const LWIP_IPV6_ADDRESS_LIFETIMES = LWIP_IPV6_AUTOCONFIG;
pub const LWIP_IPV6_DUP_DETECT_ATTEMPTS = @as(c_int, 1);
pub const LWIP_ICMP6 = LWIP_IPV6;
pub const LWIP_ICMP6_DATASIZE = @as(c_int, 0);
pub const LWIP_ICMP6_HL = @as(c_int, 255);
pub const LWIP_IPV6_MLD = LWIP_IPV6;
pub const MEMP_NUM_MLD6_GROUP = @as(c_int, 4);
pub const LWIP_ND6_QUEUEING = LWIP_IPV6;
pub const MEMP_NUM_ND6_QUEUE = @as(c_int, 20);
pub const LWIP_ND6_NUM_NEIGHBORS = @as(c_int, 10);
pub const LWIP_ND6_NUM_DESTINATIONS = @as(c_int, 10);
pub const LWIP_ND6_NUM_PREFIXES = @as(c_int, 5);
pub const LWIP_ND6_NUM_ROUTERS = @as(c_int, 3);
pub const LWIP_ND6_MAX_MULTICAST_SOLICIT = @as(c_int, 3);
pub const LWIP_ND6_MAX_UNICAST_SOLICIT = @as(c_int, 3);
pub const LWIP_ND6_MAX_ANYCAST_DELAY_TIME = @as(c_int, 1000);
pub const LWIP_ND6_MAX_NEIGHBOR_ADVERTISEMENT = @as(c_int, 3);
pub const LWIP_ND6_REACHABLE_TIME = @as(c_int, 30000);
pub const LWIP_ND6_RETRANS_TIMER = @as(c_int, 1000);
pub const LWIP_ND6_DELAY_FIRST_PROBE_TIME = @as(c_int, 5000);
pub const LWIP_ND6_ALLOW_RA_UPDATES = @as(c_int, 1);
pub const LWIP_ND6_TCP_REACHABILITY_HINTS = @as(c_int, 1);
pub const LWIP_ND6_RDNSS_MAX_DNS_SERVERS = @as(c_int, 0);
pub const LWIP_IPV6_DHCP6 = @as(c_int, 0);
pub const LWIP_IPV6_DHCP6_STATEFUL = @as(c_int, 0);
pub const LWIP_IPV6_DHCP6_STATELESS = LWIP_IPV6_DHCP6;
pub const LWIP_DHCP6_GET_NTP_SRV = @as(c_int, 0);
pub const LWIP_DHCP6_MAX_NTP_SERVERS = @as(c_int, 1);
pub const LWIP_DHCP6_MAX_DNS_SERVERS = DNS_MAX_SERVERS;
pub const LWIP_DBG_MIN_LEVEL = LWIP_DBG_LEVEL_ALL;
pub const LWIP_DBG_TYPES_ON = LWIP_DBG_ON;
pub const IGMP_DEBUG = LWIP_DBG_OFF;
pub const TIMERS_DEBUG = LWIP_DBG_OFF;
pub const AUTOIP_DEBUG = LWIP_DBG_OFF;
pub const ACD_DEBUG = LWIP_DBG_OFF;
pub const DNS_DEBUG = LWIP_DBG_OFF;
pub const IP6_DEBUG = LWIP_DBG_OFF;
pub const DHCP6_DEBUG = LWIP_DBG_OFF;
pub const LWIP_TESTMODE = @as(c_int, 0);
pub const LWIP_PERF = @as(c_int, 0);
pub const ENABLE_LOOPBACK = (LWIP_NETIF_LOOPBACK != 0) or (LWIP_HAVE_LOOPIF != 0);
pub const LWIP_HDR_ERR_H = "";
pub const LWIP_HDR_IP_ADDR_H = "";
pub const LWIP_HDR_DEF_H = "";
pub const PERF_START = "";
pub const PERF_STOP = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/def.h:58:9
pub inline fn LWIP_MAX(x: anytype, y: anytype) @TypeOf(if (x > y) x else y) {
    _ = &x;
    _ = &y;
    return if (x > y) x else y;
}
pub inline fn LWIP_MIN(x: anytype, y: anytype) @TypeOf(if (x < y) x else y) {
    _ = &x;
    _ = &y;
    return if (x < y) x else y;
}
pub const LWIP_ARRAYSIZE = @compileError("unable to translate C expr: unexpected token '('"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/def.h:69:9
pub inline fn LWIP_MAKEU32(a: anytype, b: anytype, c: anytype, d: anytype) @TypeOf((((__helpers.cast(u32_t, a & @as(c_int, 0xff)) << @as(c_int, 24)) | (__helpers.cast(u32_t, b & @as(c_int, 0xff)) << @as(c_int, 16))) | (__helpers.cast(u32_t, c & @as(c_int, 0xff)) << @as(c_int, 8))) | __helpers.cast(u32_t, d & @as(c_int, 0xff))) {
    _ = &a;
    _ = &b;
    _ = &c;
    _ = &d;
    return (((__helpers.cast(u32_t, a & @as(c_int, 0xff)) << @as(c_int, 24)) | (__helpers.cast(u32_t, b & @as(c_int, 0xff)) << @as(c_int, 16))) | (__helpers.cast(u32_t, c & @as(c_int, 0xff)) << @as(c_int, 8))) | __helpers.cast(u32_t, d & @as(c_int, 0xff));
}
pub inline fn lwip_ntohs(x: anytype) @TypeOf(lwip_htons(x)) {
    _ = &x;
    return lwip_htons(x);
}
pub inline fn lwip_ntohl(x: anytype) @TypeOf(lwip_htonl(x)) {
    _ = &x;
    return lwip_htonl(x);
}
pub inline fn PP_HTONS(x: anytype) u16_t {
    _ = &x;
    return __helpers.cast(u16_t, ((x & __helpers.cast(u16_t, @as(c_uint, 0x00ff))) << @as(c_int, 8)) | ((x & __helpers.cast(u16_t, @as(c_uint, 0xff00))) >> @as(c_int, 8)));
}
pub inline fn PP_NTOHS(x: anytype) @TypeOf(PP_HTONS(x)) {
    _ = &x;
    return PP_HTONS(x);
}
pub inline fn PP_HTONL(x: anytype) @TypeOf(((((x & __helpers.cast(u32_t, @as(c_ulong, 0x000000ff))) << @as(c_int, 24)) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0x0000ff00))) << @as(c_int, 8))) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0x00ff0000))) >> @as(c_int, 8))) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0xff000000))) >> @as(c_int, 24))) {
    _ = &x;
    return ((((x & __helpers.cast(u32_t, @as(c_ulong, 0x000000ff))) << @as(c_int, 24)) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0x0000ff00))) << @as(c_int, 8))) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0x00ff0000))) >> @as(c_int, 8))) | ((x & __helpers.cast(u32_t, @as(c_ulong, 0xff000000))) >> @as(c_int, 24));
}
pub inline fn PP_NTOHL(x: anytype) @TypeOf(PP_HTONL(x)) {
    _ = &x;
    return PP_HTONL(x);
}
pub inline fn htons(x: anytype) @TypeOf(lwip_htons(x)) {
    _ = &x;
    return lwip_htons(x);
}
pub inline fn ntohs(x: anytype) @TypeOf(lwip_ntohs(x)) {
    _ = &x;
    return lwip_ntohs(x);
}
pub inline fn htonl(x: anytype) @TypeOf(lwip_htonl(x)) {
    _ = &x;
    return lwip_htonl(x);
}
pub inline fn ntohl(x: anytype) @TypeOf(lwip_ntohl(x)) {
    _ = &x;
    return lwip_ntohl(x);
}
pub const LWIP_HDR_IP4_ADDR_H = "";
pub const IPADDR_NONE = __helpers.cast(u32_t, @as(c_ulong, 0xffffffff));
pub const IPADDR_LOOPBACK = __helpers.cast(u32_t, @as(c_ulong, 0x7f000001));
pub const IPADDR_ANY = __helpers.cast(u32_t, @as(c_ulong, 0x00000000));
pub const IPADDR_BROADCAST = __helpers.cast(u32_t, @as(c_ulong, 0xffffffff));
pub inline fn IP_CLASSA(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0x80000000)) == @as(c_int, 0)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0x80000000)) == @as(c_int, 0);
}
pub const IP_CLASSA_NET = __helpers.promoteIntLiteral(c_int, 0xff000000, .hex);
pub const IP_CLASSA_NSHIFT = @as(c_int, 24);
pub const IP_CLASSA_HOST = __helpers.promoteIntLiteral(c_int, 0xffffffff, .hex) & ~IP_CLASSA_NET;
pub const IP_CLASSA_MAX = @as(c_int, 128);
pub inline fn IP_CLASSB(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0xc0000000)) == @as(c_ulong, 0x80000000)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0xc0000000)) == @as(c_ulong, 0x80000000);
}
pub const IP_CLASSB_NET = __helpers.promoteIntLiteral(c_int, 0xffff0000, .hex);
pub const IP_CLASSB_NSHIFT = @as(c_int, 16);
pub const IP_CLASSB_HOST = __helpers.promoteIntLiteral(c_int, 0xffffffff, .hex) & ~IP_CLASSB_NET;
pub const IP_CLASSB_MAX = __helpers.promoteIntLiteral(c_int, 65536, .decimal);
pub inline fn IP_CLASSC(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0xe0000000)) == @as(c_ulong, 0xc0000000)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0xe0000000)) == @as(c_ulong, 0xc0000000);
}
pub const IP_CLASSC_NET = __helpers.promoteIntLiteral(c_int, 0xffffff00, .hex);
pub const IP_CLASSC_NSHIFT = @as(c_int, 8);
pub const IP_CLASSC_HOST = __helpers.promoteIntLiteral(c_int, 0xffffffff, .hex) & ~IP_CLASSC_NET;
pub inline fn IP_CLASSD(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xe0000000)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xe0000000);
}
pub const IP_CLASSD_NET = __helpers.promoteIntLiteral(c_int, 0xf0000000, .hex);
pub const IP_CLASSD_NSHIFT = @as(c_int, 28);
pub const IP_CLASSD_HOST = __helpers.promoteIntLiteral(c_int, 0x0fffffff, .hex);
pub inline fn IP_MULTICAST(a: anytype) @TypeOf(IP_CLASSD(a)) {
    _ = &a;
    return IP_CLASSD(a);
}
pub inline fn IP_EXPERIMENTAL(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xf0000000)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xf0000000);
}
pub inline fn IP_BADCLASS(a: anytype) @TypeOf((__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xf0000000)) {
    _ = &a;
    return (__helpers.cast(u32_t, a) & @as(c_ulong, 0xf0000000)) == @as(c_ulong, 0xf0000000);
}
pub const IP_LOOPBACKNET = @as(c_int, 127);
pub const IP4_ADDR = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:104:9
pub const ip4_addr_copy = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:107:9
pub const ip4_addr_set = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:109:9
pub const ip4_addr_set_zero = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:113:9
pub const ip4_addr_set_any = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:115:9
pub const ip4_addr_set_loopback = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:117:9
pub inline fn ip4_addr_isloopback(ipaddr: anytype) @TypeOf((ipaddr.*.addr & PP_HTONL(IP_CLASSA_NET)) == PP_HTONL(__helpers.cast(u32_t, IP_LOOPBACKNET) << @as(c_int, 24))) {
    _ = &ipaddr;
    return (ipaddr.*.addr & PP_HTONL(IP_CLASSA_NET)) == PP_HTONL(__helpers.cast(u32_t, IP_LOOPBACKNET) << @as(c_int, 24));
}
pub const ip4_addr_set_hton = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:122:9
pub const ip4_addr_set_u32 = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:126:9
pub inline fn ip4_addr_get_u32(src_ipaddr: anytype) @TypeOf(src_ipaddr.*.addr) {
    _ = &src_ipaddr;
    return src_ipaddr.*.addr;
}
pub const ip4_addr_get_network = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:131:9
pub inline fn ip4_addr_netcmp(addr1: anytype, addr2: anytype, mask: anytype) @TypeOf(ip4_addr_net_eq(addr1, addr2, mask)) {
    _ = &addr1;
    _ = &addr2;
    _ = &mask;
    return ip4_addr_net_eq(addr1, addr2, mask);
}
pub inline fn ip4_addr_net_eq(addr1: anytype, addr2: anytype, mask: anytype) @TypeOf((addr1.*.addr & mask.*.addr) == (addr2.*.addr & mask.*.addr)) {
    _ = &addr1;
    _ = &addr2;
    _ = &mask;
    return (addr1.*.addr & mask.*.addr) == (addr2.*.addr & mask.*.addr);
}
pub inline fn ip4_addr_cmp(addr1: anytype, addr2: anytype) @TypeOf(ip4_addr_eq(addr1, addr2)) {
    _ = &addr1;
    _ = &addr2;
    return ip4_addr_eq(addr1, addr2);
}
pub inline fn ip4_addr_eq(addr1: anytype, addr2: anytype) @TypeOf(addr1.*.addr == addr2.*.addr) {
    _ = &addr1;
    _ = &addr2;
    return addr1.*.addr == addr2.*.addr;
}
pub inline fn ip4_addr_isany_val(addr1: anytype) @TypeOf(addr1.addr == IPADDR_ANY) {
    _ = &addr1;
    return addr1.addr == IPADDR_ANY;
}
pub inline fn ip4_addr_isany(addr1: anytype) @TypeOf((addr1 == NULL) or (ip4_addr_isany_val(addr1.*) != 0)) {
    _ = &addr1;
    return (addr1 == NULL) or (ip4_addr_isany_val(addr1.*) != 0);
}
pub inline fn ip4_addr_isbroadcast(addr1: anytype, netif_1: anytype) @TypeOf(ip4_addr_isbroadcast_u32(addr1.*.addr, netif_1)) {
    _ = &addr1;
    _ = &netif_1;
    return ip4_addr_isbroadcast_u32(addr1.*.addr, netif_1);
}
pub inline fn ip_addr_netmask_valid(netmask: anytype) @TypeOf(ip4_addr_netmask_valid(netmask.*.addr)) {
    _ = &netmask;
    return ip4_addr_netmask_valid(netmask.*.addr);
}
pub inline fn ip4_addr_ismulticast(addr1: anytype) @TypeOf((addr1.*.addr & PP_HTONL(@as(c_ulong, 0xf0000000))) == PP_HTONL(@as(c_ulong, 0xe0000000))) {
    _ = &addr1;
    return (addr1.*.addr & PP_HTONL(@as(c_ulong, 0xf0000000))) == PP_HTONL(@as(c_ulong, 0xe0000000));
}
pub inline fn ip4_addr_islinklocal(addr1: anytype) @TypeOf((addr1.*.addr & PP_HTONL(@as(c_ulong, 0xffff0000))) == PP_HTONL(@as(c_ulong, 0xa9fe0000))) {
    _ = &addr1;
    return (addr1.*.addr & PP_HTONL(@as(c_ulong, 0xffff0000))) == PP_HTONL(@as(c_ulong, 0xa9fe0000));
}
pub inline fn ip4_addr_debug_print_parts(debug: anytype, a: anytype, b: anytype, c: anytype, d: anytype) @TypeOf(LWIP_DEBUGF(debug, blk_1: {
    _ = "%" ++ U16_F ++ ".%" ++ U16_F ++ ".%" ++ U16_F ++ ".%" ++ U16_F;
    _ = &a;
    _ = &b;
    _ = &c;
    break :blk_1 d;
})) {
    _ = &debug;
    _ = &a;
    _ = &b;
    _ = &c;
    _ = &d;
    return LWIP_DEBUGF(debug, blk_1: {
        _ = "%" ++ U16_F ++ ".%" ++ U16_F ++ ".%" ++ U16_F ++ ".%" ++ U16_F;
        _ = &a;
        _ = &b;
        _ = &c;
        break :blk_1 d;
    });
}
pub inline fn ip4_addr_debug_print(debug: anytype, ipaddr: anytype) @TypeOf(ip4_addr_debug_print_parts(debug, __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr1_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr2_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr3_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr4_16(ipaddr) else @as(c_int, 0)))) {
    _ = &debug;
    _ = &ipaddr;
    return ip4_addr_debug_print_parts(debug, __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr1_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr2_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr3_16(ipaddr) else @as(c_int, 0)), __helpers.cast(u16_t, if (ipaddr != NULL) ip4_addr4_16(ipaddr) else @as(c_int, 0)));
}
pub inline fn ip4_addr_debug_print_val(debug: anytype, ipaddr: anytype) @TypeOf(ip4_addr_debug_print_parts(debug, ip4_addr1_16_val(ipaddr), ip4_addr2_16_val(ipaddr), ip4_addr3_16_val(ipaddr), ip4_addr4_16_val(ipaddr))) {
    _ = &debug;
    _ = &ipaddr;
    return ip4_addr_debug_print_parts(debug, ip4_addr1_16_val(ipaddr), ip4_addr2_16_val(ipaddr), ip4_addr3_16_val(ipaddr), ip4_addr4_16_val(ipaddr));
}
pub const ip4_addr_get_byte = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4_addr.h:185:9
pub inline fn ip4_addr1(ipaddr: anytype) @TypeOf(ip4_addr_get_byte(ipaddr, @as(c_int, 0))) {
    _ = &ipaddr;
    return ip4_addr_get_byte(ipaddr, @as(c_int, 0));
}
pub inline fn ip4_addr2(ipaddr: anytype) @TypeOf(ip4_addr_get_byte(ipaddr, @as(c_int, 1))) {
    _ = &ipaddr;
    return ip4_addr_get_byte(ipaddr, @as(c_int, 1));
}
pub inline fn ip4_addr3(ipaddr: anytype) @TypeOf(ip4_addr_get_byte(ipaddr, @as(c_int, 2))) {
    _ = &ipaddr;
    return ip4_addr_get_byte(ipaddr, @as(c_int, 2));
}
pub inline fn ip4_addr4(ipaddr: anytype) @TypeOf(ip4_addr_get_byte(ipaddr, @as(c_int, 3))) {
    _ = &ipaddr;
    return ip4_addr_get_byte(ipaddr, @as(c_int, 3));
}
pub inline fn ip4_addr_get_byte_val(ipaddr: anytype, idx: anytype) u8_t {
    _ = &ipaddr;
    _ = &idx;
    return __helpers.cast(u8_t, (ipaddr.addr >> (idx * @as(c_int, 8))) & @as(c_int, 0xff));
}
pub inline fn ip4_addr1_val(ipaddr: anytype) @TypeOf(ip4_addr_get_byte_val(ipaddr, @as(c_int, 0))) {
    _ = &ipaddr;
    return ip4_addr_get_byte_val(ipaddr, @as(c_int, 0));
}
pub inline fn ip4_addr2_val(ipaddr: anytype) @TypeOf(ip4_addr_get_byte_val(ipaddr, @as(c_int, 1))) {
    _ = &ipaddr;
    return ip4_addr_get_byte_val(ipaddr, @as(c_int, 1));
}
pub inline fn ip4_addr3_val(ipaddr: anytype) @TypeOf(ip4_addr_get_byte_val(ipaddr, @as(c_int, 2))) {
    _ = &ipaddr;
    return ip4_addr_get_byte_val(ipaddr, @as(c_int, 2));
}
pub inline fn ip4_addr4_val(ipaddr: anytype) @TypeOf(ip4_addr_get_byte_val(ipaddr, @as(c_int, 3))) {
    _ = &ipaddr;
    return ip4_addr_get_byte_val(ipaddr, @as(c_int, 3));
}
pub inline fn ip4_addr1_16(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr1(ipaddr));
}
pub inline fn ip4_addr2_16(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr2(ipaddr));
}
pub inline fn ip4_addr3_16(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr3(ipaddr));
}
pub inline fn ip4_addr4_16(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr4(ipaddr));
}
pub inline fn ip4_addr1_16_val(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr1_val(ipaddr));
}
pub inline fn ip4_addr2_16_val(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr2_val(ipaddr));
}
pub inline fn ip4_addr3_16_val(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr3_val(ipaddr));
}
pub inline fn ip4_addr4_16_val(ipaddr: anytype) u16_t {
    _ = &ipaddr;
    return __helpers.cast(u16_t, ip4_addr4_val(ipaddr));
}
pub const IP4ADDR_STRLEN_MAX = @as(c_int, 16);
pub inline fn ip_ntoa(ipaddr: anytype) @TypeOf(ipaddr_ntoa(ipaddr)) {
    _ = &ipaddr;
    return ipaddr_ntoa(ipaddr);
}
pub const LWIP_HDR_IP6_ADDR_H = "";
pub inline fn IP_ADDR_PCB_VERSION_MATCH(addr: anytype, pcb: anytype) @TypeOf(@as(c_int, 1)) {
    _ = &addr;
    _ = &pcb;
    return @as(c_int, 1);
}
pub inline fn IP_ADDR_PCB_VERSION_MATCH_EXACT(pcb: anytype, ipaddr: anytype) @TypeOf(@as(c_int, 1)) {
    _ = &pcb;
    _ = &ipaddr;
    return @as(c_int, 1);
}
pub inline fn ip_addr_set_any_val(is_ipv6: anytype, ipaddr: anytype) @TypeOf(ip_addr_set_any(is_ipv6, &ipaddr)) {
    _ = &is_ipv6;
    _ = &ipaddr;
    return ip_addr_set_any(is_ipv6, &ipaddr);
}
pub inline fn ip_addr_set_loopback_val(is_ipv6: anytype, ipaddr: anytype) @TypeOf(ip_addr_set_loopback(is_ipv6, &ipaddr)) {
    _ = &is_ipv6;
    _ = &ipaddr;
    return ip_addr_set_loopback(is_ipv6, &ipaddr);
}
pub const IPADDR4_INIT = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip_addr.h:295:9
pub inline fn IPADDR4_INIT_BYTES(a: anytype, b: anytype, c: anytype, d: anytype) @TypeOf(IPADDR4_INIT(PP_HTONL(LWIP_MAKEU32(a, b, c, d)))) {
    _ = &a;
    _ = &b;
    _ = &c;
    _ = &d;
    return IPADDR4_INIT(PP_HTONL(LWIP_MAKEU32(a, b, c, d)));
}
pub inline fn IP_IS_V4_VAL(ipaddr: anytype) @TypeOf(@as(c_int, 1)) {
    _ = &ipaddr;
    return @as(c_int, 1);
}
pub inline fn IP_IS_V6_VAL(ipaddr: anytype) @TypeOf(@as(c_int, 0)) {
    _ = &ipaddr;
    return @as(c_int, 0);
}
pub inline fn IP_IS_V4(ipaddr: anytype) @TypeOf(@as(c_int, 1)) {
    _ = &ipaddr;
    return @as(c_int, 1);
}
pub inline fn IP_IS_V6(ipaddr: anytype) @TypeOf(@as(c_int, 0)) {
    _ = &ipaddr;
    return @as(c_int, 0);
}
pub inline fn IP_IS_ANY_TYPE_VAL(ipaddr: anytype) @TypeOf(@as(c_int, 0)) {
    _ = &ipaddr;
    return @as(c_int, 0);
}
pub const IP_SET_TYPE_VAL = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip_addr.h:302:9
pub const IP_SET_TYPE = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip_addr.h:303:9
pub inline fn IP_GET_TYPE(ipaddr: anytype) @TypeOf(IPADDR_TYPE_V4) {
    _ = &ipaddr;
    return IPADDR_TYPE_V4;
}
pub inline fn IP_ADDR_RAW_SIZE(ipaddr: anytype) @TypeOf(__helpers.sizeof(ip4_addr_t)) {
    _ = &ipaddr;
    return __helpers.sizeof(ip4_addr_t);
}
pub inline fn ip_2_ip4(ipaddr: anytype) @TypeOf(ipaddr) {
    _ = &ipaddr;
    return ipaddr;
}
pub inline fn IP_ADDR4(ipaddr: anytype, a: anytype, b: anytype, c: anytype, d: anytype) @TypeOf(IP4_ADDR(ipaddr, a, b, c, d)) {
    _ = &ipaddr;
    _ = &a;
    _ = &b;
    _ = &c;
    _ = &d;
    return IP4_ADDR(ipaddr, a, b, c, d);
}
pub inline fn ip_addr_copy(dest: anytype, src: anytype) @TypeOf(ip4_addr_copy(dest, src)) {
    _ = &dest;
    _ = &src;
    return ip4_addr_copy(dest, src);
}
pub inline fn ip_addr_copy_from_ip4(dest: anytype, src: anytype) @TypeOf(ip4_addr_copy(dest, src)) {
    _ = &dest;
    _ = &src;
    return ip4_addr_copy(dest, src);
}
pub inline fn ip_addr_set_ip4_u32(ipaddr: anytype, val: anytype) @TypeOf(ip4_addr_set_u32(ip_2_ip4(ipaddr), val)) {
    _ = &ipaddr;
    _ = &val;
    return ip4_addr_set_u32(ip_2_ip4(ipaddr), val);
}
pub inline fn ip_addr_set_ip4_u32_val(ipaddr: anytype, val: anytype) @TypeOf(ip_addr_set_ip4_u32(&ipaddr, val)) {
    _ = &ipaddr;
    _ = &val;
    return ip_addr_set_ip4_u32(&ipaddr, val);
}
pub inline fn ip_addr_get_ip4_u32(ipaddr: anytype) @TypeOf(ip4_addr_get_u32(ip_2_ip4(ipaddr))) {
    _ = &ipaddr;
    return ip4_addr_get_u32(ip_2_ip4(ipaddr));
}
pub inline fn ip_addr_set(dest: anytype, src: anytype) @TypeOf(ip4_addr_set(dest, src)) {
    _ = &dest;
    _ = &src;
    return ip4_addr_set(dest, src);
}
pub inline fn ip_addr_set_ipaddr(dest: anytype, src: anytype) @TypeOf(ip4_addr_set(dest, src)) {
    _ = &dest;
    _ = &src;
    return ip4_addr_set(dest, src);
}
pub inline fn ip_addr_set_zero(ipaddr: anytype) @TypeOf(ip4_addr_set_zero(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_set_zero(ipaddr);
}
pub inline fn ip_addr_set_zero_ip4(ipaddr: anytype) @TypeOf(ip4_addr_set_zero(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_set_zero(ipaddr);
}
pub inline fn ip_addr_set_any(is_ipv6: anytype, ipaddr: anytype) @TypeOf(ip4_addr_set_any(ipaddr)) {
    _ = &is_ipv6;
    _ = &ipaddr;
    return ip4_addr_set_any(ipaddr);
}
pub inline fn ip_addr_set_loopback(is_ipv6: anytype, ipaddr: anytype) @TypeOf(ip4_addr_set_loopback(ipaddr)) {
    _ = &is_ipv6;
    _ = &ipaddr;
    return ip4_addr_set_loopback(ipaddr);
}
pub inline fn ip_addr_set_hton(dest: anytype, src: anytype) @TypeOf(ip4_addr_set_hton(dest, src)) {
    _ = &dest;
    _ = &src;
    return ip4_addr_set_hton(dest, src);
}
pub inline fn ip_addr_get_network(target: anytype, host: anytype, mask: anytype) @TypeOf(ip4_addr_get_network(target, host, mask)) {
    _ = &target;
    _ = &host;
    _ = &mask;
    return ip4_addr_get_network(target, host, mask);
}
pub inline fn ip_addr_netcmp(addr1: anytype, addr2: anytype, mask: anytype) @TypeOf(ip4_addr_net_eq(addr1, addr2, mask)) {
    _ = &addr1;
    _ = &addr2;
    _ = &mask;
    return ip4_addr_net_eq(addr1, addr2, mask);
}
pub inline fn ip_addr_net_eq(addr1: anytype, addr2: anytype, mask: anytype) @TypeOf(ip4_addr_net_eq(addr1, addr2, mask)) {
    _ = &addr1;
    _ = &addr2;
    _ = &mask;
    return ip4_addr_net_eq(addr1, addr2, mask);
}
pub inline fn ip_addr_cmp(addr1: anytype, addr2: anytype) @TypeOf(ip4_addr_eq(addr1, addr2)) {
    _ = &addr1;
    _ = &addr2;
    return ip4_addr_eq(addr1, addr2);
}
pub inline fn ip_addr_eq(addr1: anytype, addr2: anytype) @TypeOf(ip4_addr_eq(addr1, addr2)) {
    _ = &addr1;
    _ = &addr2;
    return ip4_addr_eq(addr1, addr2);
}
pub inline fn ip_addr_isany(ipaddr: anytype) @TypeOf(ip4_addr_isany(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_isany(ipaddr);
}
pub inline fn ip_addr_isany_val(ipaddr: anytype) @TypeOf(ip4_addr_isany_val(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_isany_val(ipaddr);
}
pub inline fn ip_addr_isloopback(ipaddr: anytype) @TypeOf(ip4_addr_isloopback(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_isloopback(ipaddr);
}
pub inline fn ip_addr_islinklocal(ipaddr: anytype) @TypeOf(ip4_addr_islinklocal(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_islinklocal(ipaddr);
}
pub inline fn ip_addr_isbroadcast(addr: anytype, netif_1: anytype) @TypeOf(ip4_addr_isbroadcast(addr, netif_1)) {
    _ = &addr;
    _ = &netif_1;
    return ip4_addr_isbroadcast(addr, netif_1);
}
pub inline fn ip_addr_ismulticast(ipaddr: anytype) @TypeOf(ip4_addr_ismulticast(ipaddr)) {
    _ = &ipaddr;
    return ip4_addr_ismulticast(ipaddr);
}
pub inline fn ip_addr_debug_print(debug: anytype, ipaddr: anytype) @TypeOf(ip4_addr_debug_print(debug, ipaddr)) {
    _ = &debug;
    _ = &ipaddr;
    return ip4_addr_debug_print(debug, ipaddr);
}
pub inline fn ip_addr_debug_print_val(debug: anytype, ipaddr: anytype) @TypeOf(ip4_addr_debug_print_val(debug, ipaddr)) {
    _ = &debug;
    _ = &ipaddr;
    return ip4_addr_debug_print_val(debug, ipaddr);
}
pub inline fn ipaddr_ntoa(ipaddr: anytype) @TypeOf(ip4addr_ntoa(ipaddr)) {
    _ = &ipaddr;
    return ip4addr_ntoa(ipaddr);
}
pub inline fn ipaddr_ntoa_r(ipaddr: anytype, buf: anytype, buflen: anytype) @TypeOf(ip4addr_ntoa_r(ipaddr, buf, buflen)) {
    _ = &ipaddr;
    _ = &buf;
    _ = &buflen;
    return ip4addr_ntoa_r(ipaddr, buf, buflen);
}
pub inline fn ipaddr_aton(cp: anytype, addr: anytype) @TypeOf(ip4addr_aton(cp, addr)) {
    _ = &cp;
    _ = &addr;
    return ip4addr_aton(cp, addr);
}
pub const IPADDR_STRLEN_MAX = IP4ADDR_STRLEN_MAX;
pub inline fn IP46_ADDR_ANY(@"type": anytype) @TypeOf(IP4_ADDR_ANY) {
    _ = &@"type";
    return IP4_ADDR_ANY;
}
pub const IP_ADDR_ANY = IP4_ADDR_ANY;
pub const IP4_ADDR_ANY = &ip_addr_any;
pub const IP4_ADDR_ANY4 = ip_2_ip4(&ip_addr_any);
pub const IP_ADDR_BROADCAST = &ip_addr_broadcast;
pub const IP4_ADDR_BROADCAST = ip_2_ip4(&ip_addr_broadcast);
pub const IP_ANY_TYPE = IP_ADDR_ANY;
pub const LWIP_HDR_PBUF_H = "";
pub const LWIP_SUPPORT_CUSTOM_PBUF = ((IP_FRAG != 0) and !(LWIP_NETIF_TX_SINGLE_PBUF != 0)) or ((LWIP_IPV6 != 0) and (LWIP_IPV6_FRAG != 0));
pub inline fn PBUF_NEEDS_COPY(p: anytype) @TypeOf(p.*.type_internal & PBUF_TYPE_FLAG_DATA_VOLATILE) {
    _ = &p;
    return p.*.type_internal & PBUF_TYPE_FLAG_DATA_VOLATILE;
}
pub const PBUF_TRANSPORT_HLEN = @as(c_int, 20);
pub const PBUF_IP_HLEN = @as(c_int, 20);
pub const PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS = @as(c_int, 0x80);
pub const PBUF_TYPE_FLAG_DATA_VOLATILE = @as(c_int, 0x40);
pub const PBUF_TYPE_ALLOC_SRC_MASK = @as(c_int, 0x0F);
pub const PBUF_ALLOC_FLAG_RX = @as(c_int, 0x0100);
pub const PBUF_ALLOC_FLAG_DATA_CONTIGUOUS = @as(c_int, 0x0200);
pub const PBUF_TYPE_ALLOC_SRC_MASK_STD_HEAP = @as(c_int, 0x00);
pub const PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF = @as(c_int, 0x01);
pub const PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF_POOL = @as(c_int, 0x02);
pub const PBUF_TYPE_ALLOC_SRC_MASK_APP_MIN = @as(c_int, 0x03);
pub const PBUF_TYPE_ALLOC_SRC_MASK_APP_MAX = PBUF_TYPE_ALLOC_SRC_MASK;
pub const PBUF_FLAG_PUSH = @as(c_uint, 0x01);
pub const PBUF_FLAG_IS_CUSTOM = @as(c_uint, 0x02);
pub const PBUF_FLAG_MCASTLOOP = @as(c_uint, 0x04);
pub const PBUF_FLAG_LLBCAST = @as(c_uint, 0x08);
pub const PBUF_FLAG_LLMCAST = @as(c_uint, 0x10);
pub const PBUF_FLAG_TCP_FIN = @as(c_uint, 0x20);
pub const PBUF_POOL_FREE_OOSEQ = @as(c_int, 1);
pub const PBUF_CHECK_FREE_OOSEQ = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/pbuf.h:263:9
pub const pbuf_init = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/pbuf.h:273:9
pub inline fn pbuf_get_allocsrc(p: anytype) @TypeOf(p.*.type_internal & PBUF_TYPE_ALLOC_SRC_MASK) {
    _ = &p;
    return p.*.type_internal & PBUF_TYPE_ALLOC_SRC_MASK;
}
pub inline fn pbuf_match_allocsrc(p: anytype, @"type": anytype) @TypeOf(pbuf_get_allocsrc(p) == (@"type" & PBUF_TYPE_ALLOC_SRC_MASK)) {
    _ = &p;
    _ = &@"type";
    return pbuf_get_allocsrc(p) == (@"type" & PBUF_TYPE_ALLOC_SRC_MASK);
}
pub inline fn pbuf_match_type(p: anytype, @"type": anytype) @TypeOf(pbuf_match_allocsrc(p, @"type")) {
    _ = &p;
    _ = &@"type";
    return pbuf_match_allocsrc(p, @"type");
}
pub const LWIP_HDR_STATS_H = "";
pub const LWIP_HDR_MEM_H = "";
pub const MEM_SIZE_F = U16_F;
pub const LWIP_HDR_MEMP_H = "";
pub const LWIP_HDR_MEMP_PRIV_H = "";
pub const LWIP_HDR_MEM_PRIV_H = "";
pub const MEMP_SIZE = @as(c_int, 0);
pub inline fn MEMP_ALIGN_SIZE(x: anytype) @TypeOf(LWIP_MEM_ALIGN_SIZE(x)) {
    _ = &x;
    return LWIP_MEM_ALIGN_SIZE(x);
}
pub const DECLARE_LWIP_MEMPOOL_DESC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/priv/memp_priv.h:134:9
pub const LWIP_MEMPOOL_DECLARE_STATS_INSTANCE = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/priv/memp_priv.h:143:9
pub const LWIP_MEMPOOL_DECLARE_STATS_REFERENCE = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/priv/memp_priv.h:144:9
pub const LWIP_MEMPOOL_PROTOTYPE = @compileError("unable to translate macro: undefined identifier `memp_`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/memp.h:67:9
pub const LWIP_MEMPOOL_DECLARE = @compileError("unable to translate macro: undefined identifier `memp_memory_`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/memp.h:95:9
pub const LWIP_MEMPOOL_INIT = @compileError("unable to translate macro: undefined identifier `memp_`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/memp.h:117:9
pub const LWIP_MEMPOOL_ALLOC = @compileError("unable to translate macro: undefined identifier `memp_`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/memp.h:122:9
pub const LWIP_MEMPOOL_FREE = @compileError("unable to translate macro: undefined identifier `memp_`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/memp.h:127:9
pub const LWIP_STATS_LARGE = @as(c_int, 0);
pub const STAT_COUNTER = u16_t;
pub const STAT_COUNTER_F = U16_F;
pub const STATS_INC = @compileError("TODO unary inc/dec expr"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:312:9
pub const STATS_DEC = @compileError("TODO unary inc/dec expr"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:313:9
pub const STATS_INC_USED = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:314:9
pub const STATS_GET = @compileError("unable to translate C expr: expected 'an identifier' instead got ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:319:9
pub inline fn TCP_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn TCP_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.tcp, "TCP")) {
    return stats_display_proto(&lwip_stats.tcp, "TCP");
}
pub inline fn UDP_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn UDP_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.udp, "UDP")) {
    return stats_display_proto(&lwip_stats.udp, "UDP");
}
pub inline fn ICMP_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn ICMP_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.icmp, "ICMP")) {
    return stats_display_proto(&lwip_stats.icmp, "ICMP");
}
pub const IGMP_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:355:9
pub const IGMP_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:356:9
pub inline fn IP_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn IP_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.ip, "IP")) {
    return stats_display_proto(&lwip_stats.ip, "IP");
}
pub inline fn IPFRAG_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn IPFRAG_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.ip_frag, "IP_FRAG")) {
    return stats_display_proto(&lwip_stats.ip_frag, "IP_FRAG");
}
pub inline fn ETHARP_STATS_INC(x: anytype) @TypeOf(STATS_INC(x)) {
    _ = &x;
    return STATS_INC(x);
}
pub inline fn ETHARP_STATS_DISPLAY() @TypeOf(stats_display_proto(&lwip_stats.etharp, "ETHARP")) {
    return stats_display_proto(&lwip_stats.etharp, "ETHARP");
}
pub const LINK_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:387:9
pub const LINK_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:388:9
pub const MEM_STATS_AVAIL = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:398:9
pub const MEM_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:399:9
pub const MEM_STATS_INC_USED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:400:9
pub const MEM_STATS_DEC_USED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:401:9
pub const MEM_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:402:9
pub const MEMP_STATS_DEC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:410:9
pub const MEMP_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:411:9
pub inline fn MEMP_STATS_GET(x: anytype, i: anytype) @TypeOf(@as(c_int, 0)) {
    _ = &x;
    _ = &i;
    return @as(c_int, 0);
}
pub const SYS_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:421:9
pub const SYS_STATS_DEC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:422:9
pub const SYS_STATS_INC_USED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:423:9
pub const SYS_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:424:9
pub const IP6_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:431:9
pub const IP6_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:432:9
pub const ICMP6_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:439:9
pub const ICMP6_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:440:9
pub const IP6_FRAG_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:447:9
pub const IP6_FRAG_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:448:9
pub const MLD6_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:455:9
pub const MLD6_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:456:9
pub const ND6_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:463:9
pub const ND6_STATS_DISPLAY = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:464:9
pub const MIB2_STATS_INC = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/stats.h:470:9
pub const NETIF_MAX_HWADDR_LEN = @as(c_uint, 6);
pub const NETIF_NAMESIZE = @as(c_int, 6);
pub const NETIF_FLAG_UP = @as(c_uint, 0x01);
pub const NETIF_FLAG_BROADCAST = @as(c_uint, 0x02);
pub const NETIF_FLAG_LINK_UP = @as(c_uint, 0x04);
pub const NETIF_FLAG_ETHARP = @as(c_uint, 0x08);
pub const NETIF_FLAG_ETHERNET = @as(c_uint, 0x10);
pub const NETIF_FLAG_IGMP = @as(c_uint, 0x20);
pub const NETIF_FLAG_MLD6 = @as(c_uint, 0x40);
pub const netif_set_client_data = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:236:9
pub inline fn netif_get_client_data(netif_1: anytype, id: anytype) @TypeOf(netif_1.*.client_data[@as(usize, @intCast(id))]) {
    _ = &netif_1;
    _ = &id;
    return netif_1.*.client_data[@as(usize, @intCast(id))];
}
pub const NETIF_ADDR_IDX_MAX = @as(c_int, 0x7F);
pub const LWIP_NETIF_USE_HINTS = @as(c_int, 0);
pub inline fn NETIF_CHECKSUM_ENABLED(netif_1: anytype, chksumflag: anytype) @TypeOf(@as(c_int, 0)) {
    _ = &netif_1;
    _ = &chksumflag;
    return @as(c_int, 0);
}
pub const NETIF_SET_CHECKSUM_CTRL = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:415:9
pub const IF__NETIF_CHECKSUM_ENABLED = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:416:9
pub const NETIF_FOREACH = @compileError("unable to translate C expr: unexpected token 'for'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:424:9
pub const netif_ip4_addr = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:457:9
pub const netif_ip4_netmask = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:459:9
pub const netif_ip4_gw = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:461:9
pub const netif_ip_addr4 = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:463:9
pub const netif_ip_netmask4 = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:465:9
pub const netif_ip_gw4 = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:467:9
pub const netif_set_flags = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:470:9
pub const netif_clear_flags = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:471:9
pub inline fn netif_is_flag_set(netif_1: anytype, flag: anytype) @TypeOf((netif_1.*.flags & flag) != @as(c_int, 0)) {
    _ = &netif_1;
    _ = &flag;
    return (netif_1.*.flags & flag) != @as(c_int, 0);
}
pub inline fn netif_is_up(netif_1: anytype) @TypeOf(if (netif_1.*.flags & NETIF_FLAG_UP) __helpers.cast(u8_t, @as(c_int, 1)) else __helpers.cast(u8_t, @as(c_int, 0))) {
    _ = &netif_1;
    return if (netif_1.*.flags & NETIF_FLAG_UP) __helpers.cast(u8_t, @as(c_int, 1)) else __helpers.cast(u8_t, @as(c_int, 0));
}
pub inline fn netif_is_link_up(netif_1: anytype) @TypeOf(if (netif_1.*.flags & NETIF_FLAG_LINK_UP) __helpers.cast(u8_t, @as(c_int, 1)) else __helpers.cast(u8_t, @as(c_int, 0))) {
    _ = &netif_1;
    return if (netif_1.*.flags & NETIF_FLAG_LINK_UP) __helpers.cast(u8_t, @as(c_int, 1)) else __helpers.cast(u8_t, @as(c_int, 0));
}
pub const netif_set_hostname = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:499:9
pub inline fn netif_get_hostname(netif_1: anytype) @TypeOf(if (netif_1 != NULL) netif_1.*.hostname else NULL) {
    _ = &netif_1;
    return if (netif_1 != NULL) netif_1.*.hostname else NULL;
}
pub const NETIF_SET_HINTS = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:569:9
pub const NETIF_RESET_HINTS = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:570:9
pub inline fn netif_get_index(netif_1: anytype) u8_t {
    _ = &netif_1;
    return __helpers.cast(u8_t, netif_1.*.num + @as(c_int, 1));
}
pub const NETIF_NO_INDEX = @as(c_int, 0);
pub const LWIP_NSC_NONE = @as(c_int, 0x0000);
pub const LWIP_NSC_NETIF_ADDED = @as(c_int, 0x0001);
pub const LWIP_NSC_NETIF_REMOVED = @as(c_int, 0x0002);
pub const LWIP_NSC_LINK_CHANGED = @as(c_int, 0x0004);
pub const LWIP_NSC_STATUS_CHANGED = @as(c_int, 0x0008);
pub const LWIP_NSC_IPV4_ADDRESS_CHANGED = @as(c_int, 0x0010);
pub const LWIP_NSC_IPV4_GATEWAY_CHANGED = @as(c_int, 0x0020);
pub const LWIP_NSC_IPV4_NETMASK_CHANGED = @as(c_int, 0x0040);
pub const LWIP_NSC_IPV4_SETTINGS_CHANGED = @as(c_int, 0x0080);
pub const LWIP_NSC_IPV6_SET = @as(c_int, 0x0100);
pub const LWIP_NSC_IPV6_ADDR_STATE_CHANGED = @as(c_int, 0x0200);
pub const LWIP_NSC_IPV4_ADDR_VALID = @as(c_int, 0x0400);
pub const NETIF_DECLARE_EXT_CALLBACK = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:683:9
pub const netif_add_ext_callback = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:684:9
pub const netif_remove_ext_callback = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:685:9
pub const netif_invoke_ext_callback = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/netif.h:686:9
pub const LWIP_HDR_DHCP_H = "";
pub const LWIP_HDR_UDP_H = "";
pub const LWIP_HDR_IP_H = "";
pub const LWIP_HDR_IP4_H = "";
pub const LWIP_HDR_PROT_IP4_H = "";
pub const IP_HLEN = @as(c_int, 20);
pub const IP_HLEN_MAX = @as(c_int, 60);
pub const IP_RF = @as(c_uint, 0x8000);
pub const IP_DF = @as(c_uint, 0x4000);
pub const IP_MF = @as(c_uint, 0x2000);
pub const IP_OFFMASK = @as(c_uint, 0x1fff);
pub inline fn IPH_V(hdr: anytype) @TypeOf(hdr.*._v_hl >> @as(c_int, 4)) {
    _ = &hdr;
    return hdr.*._v_hl >> @as(c_int, 4);
}
pub inline fn IPH_HL(hdr: anytype) @TypeOf(hdr.*._v_hl & @as(c_int, 0x0f)) {
    _ = &hdr;
    return hdr.*._v_hl & @as(c_int, 0x0f);
}
pub inline fn IPH_HL_BYTES(hdr: anytype) u8_t {
    _ = &hdr;
    return __helpers.cast(u8_t, IPH_HL(hdr) * @as(c_int, 4));
}
pub inline fn IPH_TOS(hdr: anytype) @TypeOf(hdr.*._tos) {
    _ = &hdr;
    return hdr.*._tos;
}
pub inline fn IPH_LEN(hdr: anytype) @TypeOf(hdr.*._len) {
    _ = &hdr;
    return hdr.*._len;
}
pub inline fn IPH_ID(hdr: anytype) @TypeOf(hdr.*._id) {
    _ = &hdr;
    return hdr.*._id;
}
pub inline fn IPH_OFFSET(hdr: anytype) @TypeOf(hdr.*._offset) {
    _ = &hdr;
    return hdr.*._offset;
}
pub inline fn IPH_OFFSET_BYTES(hdr: anytype) u16_t {
    _ = &hdr;
    return __helpers.cast(u16_t, (lwip_ntohs(IPH_OFFSET(hdr)) & IP_OFFMASK) * @as(c_uint, 8));
}
pub inline fn IPH_TTL(hdr: anytype) @TypeOf(hdr.*._ttl) {
    _ = &hdr;
    return hdr.*._ttl;
}
pub inline fn IPH_PROTO(hdr: anytype) @TypeOf(hdr.*._proto) {
    _ = &hdr;
    return hdr.*._proto;
}
pub inline fn IPH_CHKSUM(hdr: anytype) @TypeOf(hdr.*._chksum) {
    _ = &hdr;
    return hdr.*._chksum;
}
pub const IPH_VHL_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:117:9
pub const IPH_TOS_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:118:9
pub const IPH_LEN_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:119:9
pub const IPH_ID_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:120:9
pub const IPH_OFFSET_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:121:9
pub const IPH_TTL_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:122:9
pub const IPH_PROTO_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:123:9
pub const IPH_CHKSUM_SET = @compileError("unable to translate C expr: unexpected token '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/prot/ip4.h:124:9
pub const LWIP_IPV4_SRC_ROUTING = @as(c_int, 0);
pub const IP_OPTIONS_SEND = (LWIP_IPV4 != 0) and (LWIP_IGMP != 0);
pub const ip_init = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4.h:64:9
pub inline fn ip4_route_src(src: anytype, dest: anytype) @TypeOf(ip4_route(dest)) {
    _ = &src;
    _ = &dest;
    return ip4_route(dest);
}
pub inline fn ip4_netif_get_local_ip(netif_1: anytype) @TypeOf(if (netif_1 != NULL) netif_ip_addr4(netif_1) else NULL) {
    _ = &netif_1;
    return if (netif_1 != NULL) netif_ip_addr4(netif_1) else NULL;
}
pub const ip4_debug_print = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip4.h:100:9
pub const LWIP_HDR_IP6_H = "";
pub const LWIP_HDR_PROT_IP_H = "";
pub const IP_PROTO_ICMP = @as(c_int, 1);
pub const IP_PROTO_IGMP = @as(c_int, 2);
pub const IP_PROTO_UDP = @as(c_int, 17);
pub const IP_PROTO_UDPLITE = @as(c_int, 136);
pub const IP_PROTO_TCP = @as(c_int, 6);
pub inline fn IP_HDR_GET_VERSION(ptr: anytype) @TypeOf(__helpers.cast([*c]u8_t, ptr).* >> @as(c_int, 4)) {
    _ = &ptr;
    return __helpers.cast([*c]u8_t, ptr).* >> @as(c_int, 4);
}
pub const LWIP_IP_HDRINCL = NULL;
pub inline fn LWIP_IP_CHECK_PBUF_REF_COUNT_FOR_TX(p: anytype) @TypeOf(LWIP_ASSERT("p->ref == 1", p.*.ref == @as(c_int, 1))) {
    _ = &p;
    return LWIP_ASSERT("p->ref == 1", p.*.ref == @as(c_int, 1));
}
pub const IP_PCB_NETIFHINT = "";
pub const IP_PCB = @compileError("unable to translate macro: undefined identifier `local_ip`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:76:9
pub const pcb_tci_init = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:104:9
pub const SOF_REUSEADDR = @as(c_uint, 0x04);
pub const SOF_KEEPALIVE = @as(c_uint, 0x08);
pub const SOF_BROADCAST = @as(c_uint, 0x20);
pub const SOF_INHERITED = SOF_REUSEADDR | SOF_KEEPALIVE;
pub inline fn ip_current_netif() @TypeOf(ip_data.current_netif) {
    return ip_data.current_netif;
}
pub inline fn ip_current_input_netif() @TypeOf(ip_data.current_input_netif) {
    return ip_data.current_input_netif;
}
pub inline fn ip_current_header_tot_len() @TypeOf(ip_data.current_ip_header_tot_len) {
    return ip_data.current_ip_header_tot_len;
}
pub inline fn ip_current_src_addr() @TypeOf(&ip_data.current_iphdr_src) {
    return &ip_data.current_iphdr_src;
}
pub inline fn ip_current_dest_addr() @TypeOf(&ip_data.current_iphdr_dest) {
    return &ip_data.current_iphdr_dest;
}
pub inline fn ip4_current_header() @TypeOf(ip_data.current_ip4_header) {
    return ip_data.current_ip4_header;
}
pub inline fn ip_current_is_v6() @TypeOf(@as(c_int, 0)) {
    return @as(c_int, 0);
}
pub inline fn ip_current_header_proto() @TypeOf(IPH_PROTO(ip4_current_header())) {
    return IPH_PROTO(ip4_current_header());
}
pub const ip_next_header_ptr = @compileError("unable to translate C expr: unexpected token 'const'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:197:9
pub inline fn ip4_current_src_addr() @TypeOf(&ip_data.current_iphdr_src) {
    return &ip_data.current_iphdr_src;
}
pub inline fn ip4_current_dest_addr() @TypeOf(&ip_data.current_iphdr_dest) {
    return &ip_data.current_iphdr_dest;
}
pub inline fn ip_get_option(pcb: anytype, opt: anytype) @TypeOf(pcb.*.so_options & opt) {
    _ = &pcb;
    _ = &opt;
    return pcb.*.so_options & opt;
}
pub const ip_set_option = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:230:9
pub const ip_reset_option = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:232:9
pub inline fn ip_output(p: anytype, src: anytype, dest: anytype, ttl: anytype, tos: anytype, proto: anytype) @TypeOf(ip4_output(p, src, dest, ttl, tos, proto)) {
    _ = &p;
    _ = &src;
    _ = &dest;
    _ = &ttl;
    _ = &tos;
    _ = &proto;
    return ip4_output(p, src, dest, ttl, tos, proto);
}
pub inline fn ip_output_if(p: anytype, src: anytype, dest: anytype, ttl: anytype, tos: anytype, proto: anytype, netif_1: anytype) @TypeOf(ip4_output_if(p, src, dest, ttl, tos, proto, netif_1)) {
    _ = &p;
    _ = &src;
    _ = &dest;
    _ = &ttl;
    _ = &tos;
    _ = &proto;
    _ = &netif_1;
    return ip4_output_if(p, src, dest, ttl, tos, proto, netif_1);
}
pub inline fn ip_output_if_src(p: anytype, src: anytype, dest: anytype, ttl: anytype, tos: anytype, proto: anytype, netif_1: anytype) @TypeOf(ip4_output_if_src(p, src, dest, ttl, tos, proto, netif_1)) {
    _ = &p;
    _ = &src;
    _ = &dest;
    _ = &ttl;
    _ = &tos;
    _ = &proto;
    _ = &netif_1;
    return ip4_output_if_src(p, src, dest, ttl, tos, proto, netif_1);
}
pub const ip_output_hinted = @compileError("unable to translate macro: undefined identifier `ip4_output_hinted`"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:296:9
pub inline fn ip_output_if_hdrincl(p: anytype, src: anytype, dest: anytype, netif_1: anytype) @TypeOf(ip4_output_if(p, src, LWIP_IP_HDRINCL, @as(c_int, 0), @as(c_int, 0), @as(c_int, 0), netif_1)) {
    _ = &p;
    _ = &src;
    _ = &dest;
    _ = &netif_1;
    return ip4_output_if(p, src, LWIP_IP_HDRINCL, @as(c_int, 0), @as(c_int, 0), @as(c_int, 0), netif_1);
}
pub inline fn ip_route(src: anytype, dest: anytype) @TypeOf(ip4_route_src(src, dest)) {
    _ = &src;
    _ = &dest;
    return ip4_route_src(src, dest);
}
pub inline fn ip_netif_get_local_ip(netif_1: anytype, dest: anytype) @TypeOf(ip4_netif_get_local_ip(netif_1)) {
    _ = &netif_1;
    _ = &dest;
    return ip4_netif_get_local_ip(netif_1);
}
pub inline fn ip_debug_print(is_ipv6: anytype, p: anytype) @TypeOf(ip4_debug_print(p)) {
    _ = &is_ipv6;
    _ = &p;
    return ip4_debug_print(p);
}
pub const ip_input = ip4_input;
pub const ip_route_get_local_ip = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/ip.h:330:9
pub const LWIP_HDR_PROT_UDP_H = "";
pub const UDP_HLEN = @as(c_int, 8);
pub const UDP_FLAGS_NOCHKSUM = @as(c_uint, 0x01);
pub const UDP_FLAGS_UDPLITE = @as(c_uint, 0x02);
pub const UDP_FLAGS_CONNECTED = @as(c_uint, 0x04);
pub const UDP_FLAGS_MULTICAST_LOOP = @as(c_uint, 0x08);
pub inline fn udp_flags(pcb: anytype) @TypeOf(pcb.*.flags) {
    _ = &pcb;
    return pcb.*.flags;
}
pub const udp_setflags = @compileError("unable to translate C expr: expected ')' instead got '='"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/udp.h:156:18
pub const udp_set_flags = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/udp.h:158:18
pub const udp_clear_flags = @compileError("unable to translate C expr: unexpected token 'do'"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/udp.h:159:18
pub inline fn udp_is_flag_set(pcb: anytype, flag: anytype) @TypeOf((pcb.*.flags & flag) != @as(c_int, 0)) {
    _ = &pcb;
    _ = &flag;
    return (pcb.*.flags & flag) != @as(c_int, 0);
}
pub inline fn udp_new_ip6() @TypeOf(udp_new_ip_type(IPADDR_TYPE_V6)) {
    return udp_new_ip_type(IPADDR_TYPE_V6);
}
pub const udp_debug_print = @compileError("unable to translate C expr: unexpected token ''"); // /home/ianic/Code/pico/pico-sdk/lib/lwip/src/include/lwip/udp.h:184:9
pub const DHCP_COARSE_TIMER_SECS = @as(c_int, 60);
pub const DHCP_COARSE_TIMER_MSECS = DHCP_COARSE_TIMER_SECS * @as(c_ulong, 1000);
pub const DHCP_FINE_TIMER_MSECS = @as(c_int, 500);
pub const DHCP_BOOT_FILE_LEN = @as(c_uint, 128);
pub const DHCP_FLAG_SUBNET_MASK_GIVEN = @as(c_int, 0x01);
pub const DHCP_FLAG_EXTERNAL_MEM = @as(c_int, 0x02);
pub inline fn dhcp_remove_struct(netif_1: anytype) @TypeOf(netif_set_client_data(netif_1, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP, NULL)) {
    _ = &netif_1;
    return netif_set_client_data(netif_1, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP, NULL);
}
pub inline fn netif_dhcp_data(netif_1: anytype) [*c]struct_dhcp {
    _ = &netif_1;
    return __helpers.cast([*c]struct_dhcp, netif_get_client_data(netif_1, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
}
pub const CYW43_INCLUDED_CYW43_LL_H = "";
pub const CYW43_IOCTL_GET_SSID = @as(c_int, 0x32);
pub const CYW43_IOCTL_GET_CHANNEL = @as(c_int, 0x3a);
pub const CYW43_IOCTL_SET_DISASSOC = @as(c_int, 0x69);
pub const CYW43_IOCTL_GET_ANTDIV = @as(c_int, 0x7e);
pub const CYW43_IOCTL_SET_ANTDIV = @as(c_int, 0x81);
pub const CYW43_IOCTL_SET_MONITOR = @as(c_int, 0xd9);
pub const CYW43_IOCTL_GET_RSSI = @as(c_int, 0xfe);
pub const CYW43_IOCTL_GET_VAR = @as(c_int, 0x20c);
pub const CYW43_IOCTL_SET_VAR = @as(c_int, 0x20f);
pub const CYW43_EV_SET_SSID = @as(c_int, 0);
pub const CYW43_EV_JOIN = @as(c_int, 1);
pub const CYW43_EV_AUTH = @as(c_int, 3);
pub const CYW43_EV_DEAUTH = @as(c_int, 5);
pub const CYW43_EV_DEAUTH_IND = @as(c_int, 6);
pub const CYW43_EV_ASSOC = @as(c_int, 7);
pub const CYW43_EV_DISASSOC = @as(c_int, 11);
pub const CYW43_EV_DISASSOC_IND = @as(c_int, 12);
pub const CYW43_EV_LINK = @as(c_int, 16);
pub const CYW43_EV_PRUNE = @as(c_int, 23);
pub const CYW43_EV_PSK_SUP = @as(c_int, 46);
pub const CYW43_EV_ICV_ERROR = @as(c_int, 49);
pub const CYW43_EV_ESCAN_RESULT = @as(c_int, 69);
pub const CYW43_EV_CSA_COMPLETE_IND = @as(c_int, 80);
pub const CYW43_EV_ASSOC_REQ_IE = @as(c_int, 87);
pub const CYW43_EV_ASSOC_RESP_IE = @as(c_int, 88);
pub const CYW43_STATUS_SUCCESS = @as(c_int, 0);
pub const CYW43_STATUS_FAIL = @as(c_int, 1);
pub const CYW43_STATUS_TIMEOUT = @as(c_int, 2);
pub const CYW43_STATUS_NO_NETWORKS = @as(c_int, 3);
pub const CYW43_STATUS_ABORT = @as(c_int, 4);
pub const CYW43_STATUS_NO_ACK = @as(c_int, 5);
pub const CYW43_STATUS_UNSOLICITED = @as(c_int, 6);
pub const CYW43_STATUS_ATTEMPT = @as(c_int, 7);
pub const CYW43_STATUS_PARTIAL = @as(c_int, 8);
pub const CYW43_STATUS_NEWSCAN = @as(c_int, 9);
pub const CYW43_STATUS_NEWASSOC = @as(c_int, 10);
pub const CYW43_SUP_DISCONNECTED = @as(c_int, 0);
pub const CYW43_SUP_CONNECTING = @as(c_int, 1);
pub const CYW43_SUP_IDREQUIRED = @as(c_int, 2);
pub const CYW43_SUP_AUTHENTICATING = @as(c_int, 3);
pub const CYW43_SUP_AUTHENTICATED = @as(c_int, 4);
pub const CYW43_SUP_KEYXCHANGE = @as(c_int, 5);
pub const CYW43_SUP_KEYED = @as(c_int, 6);
pub const CYW43_SUP_TIMEOUT = @as(c_int, 7);
pub const CYW43_SUP_LAST_BASIC_STATE = @as(c_int, 8);
pub const CYW43_SUP_KEYXCHANGE_WAIT_M1 = CYW43_SUP_AUTHENTICATED;
pub const CYW43_SUP_KEYXCHANGE_PREP_M2 = CYW43_SUP_KEYXCHANGE;
pub const CYW43_SUP_KEYXCHANGE_WAIT_M3 = CYW43_SUP_LAST_BASIC_STATE;
pub const CYW43_SUP_KEYXCHANGE_PREP_M4 = @as(c_int, 9);
pub const CYW43_SUP_KEYXCHANGE_WAIT_G1 = @as(c_int, 10);
pub const CYW43_SUP_KEYXCHANGE_PREP_G2 = @as(c_int, 11);
pub const CYW43_REASON_INITIAL_ASSOC = @as(c_int, 0);
pub const CYW43_REASON_LOW_RSSI = @as(c_int, 1);
pub const CYW43_REASON_DEAUTH = @as(c_int, 2);
pub const CYW43_REASON_DISASSOC = @as(c_int, 3);
pub const CYW43_REASON_BCNS_LOST = @as(c_int, 4);
pub const CYW43_REASON_FAST_ROAM_FAILED = @as(c_int, 5);
pub const CYW43_REASON_DIRECTED_ROAM = @as(c_int, 6);
pub const CYW43_REASON_TSPEC_REJECTED = @as(c_int, 7);
pub const CYW43_REASON_BETTER_AP = @as(c_int, 8);
pub const CYW43_REASON_PRUNE_ENCR_MISMATCH = @as(c_int, 1);
pub const CYW43_REASON_PRUNE_BCAST_BSSID = @as(c_int, 2);
pub const CYW43_REASON_PRUNE_MAC_DENY = @as(c_int, 3);
pub const CYW43_REASON_PRUNE_MAC_NA = @as(c_int, 4);
pub const CYW43_REASON_PRUNE_REG_PASSV = @as(c_int, 5);
pub const CYW43_REASON_PRUNE_SPCT_MGMT = @as(c_int, 6);
pub const CYW43_REASON_PRUNE_RADAR = @as(c_int, 7);
pub const CYW43_REASON_RSN_MISMATCH = @as(c_int, 8);
pub const CYW43_REASON_PRUNE_NO_COMMON_RATES = @as(c_int, 9);
pub const CYW43_REASON_PRUNE_BASIC_RATES = @as(c_int, 10);
pub const CYW43_REASON_PRUNE_CCXFAST_PREVAP = @as(c_int, 11);
pub const CYW43_REASON_PRUNE_CIPHER_NA = @as(c_int, 12);
pub const CYW43_REASON_PRUNE_KNOWN_STA = @as(c_int, 13);
pub const CYW43_REASON_PRUNE_CCXFAST_DROAM = @as(c_int, 14);
pub const CYW43_REASON_PRUNE_WDS_PEER = @as(c_int, 15);
pub const CYW43_REASON_PRUNE_QBSS_LOAD = @as(c_int, 16);
pub const CYW43_REASON_PRUNE_HOME_AP = @as(c_int, 17);
pub const CYW43_REASON_PRUNE_AP_BLOCKED = @as(c_int, 18);
pub const CYW43_REASON_PRUNE_NO_DIAG_SUPPORT = @as(c_int, 19);
pub const CYW43_REASON_SUP_OTHER = @as(c_int, 0);
pub const CYW43_REASON_SUP_DECRYPT_KEY_DATA = @as(c_int, 1);
pub const CYW43_REASON_SUP_BAD_UCAST_WEP128 = @as(c_int, 2);
pub const CYW43_REASON_SUP_BAD_UCAST_WEP40 = @as(c_int, 3);
pub const CYW43_REASON_SUP_UNSUP_KEY_LEN = @as(c_int, 4);
pub const CYW43_REASON_SUP_PW_KEY_CIPHER = @as(c_int, 5);
pub const CYW43_REASON_SUP_MSG3_TOO_MANY_IE = @as(c_int, 6);
pub const CYW43_REASON_SUP_MSG3_IE_MISMATCH = @as(c_int, 7);
pub const CYW43_REASON_SUP_NO_INSTALL_FLAG = @as(c_int, 8);
pub const CYW43_REASON_SUP_MSG3_NO_GTK = @as(c_int, 9);
pub const CYW43_REASON_SUP_GRP_KEY_CIPHER = @as(c_int, 10);
pub const CYW43_REASON_SUP_GRP_MSG1_NO_GTK = @as(c_int, 11);
pub const CYW43_REASON_SUP_GTK_DECRYPT_FAIL = @as(c_int, 12);
pub const CYW43_REASON_SUP_SEND_FAIL = @as(c_int, 13);
pub const CYW43_REASON_SUP_DEAUTH = @as(c_int, 14);
pub const CYW43_REASON_SUP_WPA_PSK_TMO = @as(c_int, 15);
pub const CYW43_AUTH_OPEN = @as(c_int, 0);
pub const CYW43_AUTH_WPA_TKIP_PSK = __helpers.promoteIntLiteral(c_int, 0x00200002, .hex);
pub const CYW43_AUTH_WPA2_AES_PSK = __helpers.promoteIntLiteral(c_int, 0x00400004, .hex);
pub const CYW43_AUTH_WPA2_MIXED_PSK = __helpers.promoteIntLiteral(c_int, 0x00400006, .hex);
pub const CYW43_AUTH_WPA3_SAE_AES_PSK = __helpers.promoteIntLiteral(c_int, 0x01000004, .hex);
pub const CYW43_AUTH_WPA3_WPA2_AES_PSK = __helpers.promoteIntLiteral(c_int, 0x01400004, .hex);
pub const CYW43_NO_POWERSAVE_MODE = @as(c_int, 0);
pub const CYW43_PM1_POWERSAVE_MODE = @as(c_int, 1);
pub const CYW43_PM2_POWERSAVE_MODE = @as(c_int, 2);
pub const CYW43_BUS_MAX_BLOCK_SIZE = @as(c_int, 64);
pub const CYW43_BACKPLANE_READ_PAD_LEN_BYTES = @as(c_int, 16);
pub const CYW43_LL_STATE_SIZE_WORDS = ((@as(c_int, 526) + @as(c_int, 1)) + (__helpers.div(CYW43_BACKPLANE_READ_PAD_LEN_BYTES, @as(c_int, 4)) + @as(c_int, 1))) + (CYW43_INCLUDE_LEGACY_F1_OVERFLOW_WORKAROUND_VARIABLES * @as(c_int, 4));
pub const CYW43_CHANNEL_NONE = __helpers.promoteIntLiteral(c_int, 0xffffffff, .hex);
pub const _STRING_H_ = "";
pub const _STRINGS_H_ = "";
pub const _SIZE_T_DECLARED = "";
pub const CYW43_VERSION_MAJOR = @as(c_int, 1);
pub const CYW43_VERSION_MINOR = @as(c_int, 1);
pub const CYW43_VERSION_MICRO = @as(c_int, 0);
pub const CYW43_VERSION = ((CYW43_VERSION_MAJOR << @as(c_int, 16)) | (CYW43_VERSION_MINOR << @as(c_int, 8))) | CYW43_VERSION_MICRO;
pub const CYW43_TRACE_ASYNC_EV = @as(c_int, 0x0001);
pub const CYW43_TRACE_ETH_TX = @as(c_int, 0x0002);
pub const CYW43_TRACE_ETH_RX = @as(c_int, 0x0004);
pub const CYW43_TRACE_ETH_FULL = @as(c_int, 0x0008);
pub const CYW43_TRACE_MAC = @as(c_int, 0x0010);
pub const CYW43_LINK_DOWN = @as(c_int, 0);
pub const CYW43_LINK_JOIN = @as(c_int, 1);
pub const CYW43_LINK_NOIP = @as(c_int, 2);
pub const CYW43_LINK_UP = @as(c_int, 3);
pub const CYW43_LINK_FAIL = -@as(c_int, 1);
pub const CYW43_LINK_NONET = -@as(c_int, 2);
pub const CYW43_LINK_BADAUTH = -@as(c_int, 3);
pub const CYW43_DEFAULT_PM = CYW43_PERFORMANCE_PM;
pub const CYW43_NONE_PM = cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, @as(c_int, 10), @as(c_int, 0), @as(c_int, 0), @as(c_int, 0));
pub const CYW43_AGGRESSIVE_PM = cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, @as(c_int, 10), @as(c_int, 0), @as(c_int, 0), @as(c_int, 0));
pub const CYW43_PERFORMANCE_PM = cyw43_pm_value(CYW43_PM2_POWERSAVE_MODE, @as(c_int, 200), @as(c_int, 1), @as(c_int, 1), @as(c_int, 10));
pub const CYW43_INCLUDED_CYW43_COUNTRY_H = "";
pub inline fn CYW43_COUNTRY(A: anytype, B: anytype, REV: anytype) @TypeOf((__helpers.cast(u8, A) | (__helpers.cast(u8, B) << @as(c_int, 8))) | (REV << @as(c_int, 16))) {
    _ = &A;
    _ = &B;
    _ = &REV;
    return (__helpers.cast(u8, A) | (__helpers.cast(u8, B) << @as(c_int, 8))) | (REV << @as(c_int, 16));
}
pub const CYW43_COUNTRY_WORLDWIDE = CYW43_COUNTRY('X', 'X', @as(c_int, 0));
pub const CYW43_COUNTRY_AUSTRALIA = CYW43_COUNTRY('A', 'U', @as(c_int, 0));
pub const CYW43_COUNTRY_AUSTRIA = CYW43_COUNTRY('A', 'T', @as(c_int, 0));
pub const CYW43_COUNTRY_BELGIUM = CYW43_COUNTRY('B', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_BRAZIL = CYW43_COUNTRY('B', 'R', @as(c_int, 0));
pub const CYW43_COUNTRY_CANADA = CYW43_COUNTRY('C', 'A', @as(c_int, 0));
pub const CYW43_COUNTRY_CHILE = CYW43_COUNTRY('C', 'L', @as(c_int, 0));
pub const CYW43_COUNTRY_CHINA = CYW43_COUNTRY('C', 'N', @as(c_int, 0));
pub const CYW43_COUNTRY_COLOMBIA = CYW43_COUNTRY('C', 'O', @as(c_int, 0));
pub const CYW43_COUNTRY_CZECH_REPUBLIC = CYW43_COUNTRY('C', 'Z', @as(c_int, 0));
pub const CYW43_COUNTRY_DENMARK = CYW43_COUNTRY('D', 'K', @as(c_int, 0));
pub const CYW43_COUNTRY_ESTONIA = CYW43_COUNTRY('E', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_FINLAND = CYW43_COUNTRY('F', 'I', @as(c_int, 0));
pub const CYW43_COUNTRY_FRANCE = CYW43_COUNTRY('F', 'R', @as(c_int, 0));
pub const CYW43_COUNTRY_GERMANY = CYW43_COUNTRY('D', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_GREECE = CYW43_COUNTRY('G', 'R', @as(c_int, 0));
pub const CYW43_COUNTRY_HONG_KONG = CYW43_COUNTRY('H', 'K', @as(c_int, 0));
pub const CYW43_COUNTRY_HUNGARY = CYW43_COUNTRY('H', 'U', @as(c_int, 0));
pub const CYW43_COUNTRY_ICELAND = CYW43_COUNTRY('I', 'S', @as(c_int, 0));
pub const CYW43_COUNTRY_INDIA = CYW43_COUNTRY('I', 'N', @as(c_int, 0));
pub const CYW43_COUNTRY_ISRAEL = CYW43_COUNTRY('I', 'L', @as(c_int, 0));
pub const CYW43_COUNTRY_ITALY = CYW43_COUNTRY('I', 'T', @as(c_int, 0));
pub const CYW43_COUNTRY_JAPAN = CYW43_COUNTRY('J', 'P', @as(c_int, 0));
pub const CYW43_COUNTRY_KENYA = CYW43_COUNTRY('K', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_LATVIA = CYW43_COUNTRY('L', 'V', @as(c_int, 0));
pub const CYW43_COUNTRY_LIECHTENSTEIN = CYW43_COUNTRY('L', 'I', @as(c_int, 0));
pub const CYW43_COUNTRY_LITHUANIA = CYW43_COUNTRY('L', 'T', @as(c_int, 0));
pub const CYW43_COUNTRY_LUXEMBOURG = CYW43_COUNTRY('L', 'U', @as(c_int, 0));
pub const CYW43_COUNTRY_MALAYSIA = CYW43_COUNTRY('M', 'Y', @as(c_int, 0));
pub const CYW43_COUNTRY_MALTA = CYW43_COUNTRY('M', 'T', @as(c_int, 0));
pub const CYW43_COUNTRY_MEXICO = CYW43_COUNTRY('M', 'X', @as(c_int, 0));
pub const CYW43_COUNTRY_NETHERLANDS = CYW43_COUNTRY('N', 'L', @as(c_int, 0));
pub const CYW43_COUNTRY_NEW_ZEALAND = CYW43_COUNTRY('N', 'Z', @as(c_int, 0));
pub const CYW43_COUNTRY_NIGERIA = CYW43_COUNTRY('N', 'G', @as(c_int, 0));
pub const CYW43_COUNTRY_NORWAY = CYW43_COUNTRY('N', 'O', @as(c_int, 0));
pub const CYW43_COUNTRY_PERU = CYW43_COUNTRY('P', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_PHILIPPINES = CYW43_COUNTRY('P', 'H', @as(c_int, 0));
pub const CYW43_COUNTRY_POLAND = CYW43_COUNTRY('P', 'L', @as(c_int, 0));
pub const CYW43_COUNTRY_PORTUGAL = CYW43_COUNTRY('P', 'T', @as(c_int, 0));
pub const CYW43_COUNTRY_SINGAPORE = CYW43_COUNTRY('S', 'G', @as(c_int, 0));
pub const CYW43_COUNTRY_SLOVAKIA = CYW43_COUNTRY('S', 'K', @as(c_int, 0));
pub const CYW43_COUNTRY_SLOVENIA = CYW43_COUNTRY('S', 'I', @as(c_int, 0));
pub const CYW43_COUNTRY_SOUTH_AFRICA = CYW43_COUNTRY('Z', 'A', @as(c_int, 0));
pub const CYW43_COUNTRY_SOUTH_KOREA = CYW43_COUNTRY('K', 'R', @as(c_int, 0));
pub const CYW43_COUNTRY_SPAIN = CYW43_COUNTRY('E', 'S', @as(c_int, 0));
pub const CYW43_COUNTRY_SWEDEN = CYW43_COUNTRY('S', 'E', @as(c_int, 0));
pub const CYW43_COUNTRY_SWITZERLAND = CYW43_COUNTRY('C', 'H', @as(c_int, 0));
pub const CYW43_COUNTRY_TAIWAN = CYW43_COUNTRY('T', 'W', @as(c_int, 0));
pub const CYW43_COUNTRY_THAILAND = CYW43_COUNTRY('T', 'H', @as(c_int, 0));
pub const CYW43_COUNTRY_TURKEY = CYW43_COUNTRY('T', 'R', @as(c_int, 0));
pub const CYW43_COUNTRY_UK = CYW43_COUNTRY('G', 'B', @as(c_int, 0));
pub const CYW43_COUNTRY_USA = CYW43_COUNTRY('U', 'S', @as(c_int, 0));
pub const _PICO_ASYNC_CONTEXT_H = "";
pub const ASYNC_CONTEXT_FLAG_CALLBACK_FROM_NON_IRQ = @as(c_int, 0x1);
pub const ASYNC_CONTEXT_FLAG_CALLBACK_FROM_IRQ = @as(c_int, 0x2);
pub const ASYNC_CONTEXT_FLAG_POLLED = @as(c_int, 0x4);
pub const _PICO_CYW43_ARCH_ARCH_THREADSAFE_BACKGROUND_H = "";
pub const PARAM_ASSERTIONS_ENABLED_PICO_CYW43_ARCH = @as(c_int, 0);
pub const PICO_CYW43_ARCH_DEBUG_ENABLED = @as(c_int, 1);
pub const PICO_CYW43_ARCH_DEFAULT_COUNTRY_CODE = CYW43_COUNTRY_WORLDWIDE;
pub const _PICO_STDLIB_H = "";
pub const _PICO_STDIO_H = "";
pub const PICO_STDOUT_MUTEX = @as(c_int, 1);
pub const PICO_STDIO_ENABLE_CRLF_SUPPORT = @as(c_int, 1);
pub const PICO_STDIO_DEFAULT_CRLF = @as(c_int, 1);
pub const PICO_STDIO_STACK_BUFFER_SIZE = @as(c_int, 128);
pub const PICO_STDIO_DEADLOCK_TIMEOUT_MS = @as(c_int, 1000);
pub const PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS = @as(c_int, 1);
pub const _HARDWARE_UART_H = "";
pub const _HARDWARE_STRUCTS_UART_H = "";
pub const _HARDWARE_REGS_UART_H = "";
pub const uart0_hw = __helpers.cast([*c]uart_hw_t, UART0_BASE);
pub const uart1_hw = __helpers.cast([*c]uart_hw_t, UART1_BASE);
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_UART = @as(c_int, 0);
pub const PICO_UART_ENABLE_CRLF_SUPPORT = @as(c_int, 1);
pub const PICO_UART_DEFAULT_CRLF = @as(c_int, 0);
pub const PICO_DEFAULT_UART_BAUD_RATE = __helpers.promoteIntLiteral(c_int, 115200, .decimal);
pub const uart0 = __helpers.cast([*c]uart_inst_t, uart0_hw);
pub const uart1 = __helpers.cast([*c]uart_inst_t, uart1_hw);
pub const PICO_DEFAULT_UART_INSTANCE = @compileError("unable to translate macro: undefined identifier `uart`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h:98:9
pub const uart_default = PICO_DEFAULT_UART_INSTANCE();
pub inline fn UART_NUM(uart: anytype) @TypeOf(uart == uart1) {
    _ = &uart;
    return uart == uart1;
}
pub inline fn UART_INSTANCE(num: anytype) @TypeOf(if (num) uart1 else uart0) {
    _ = &num;
    return if (num) uart1 else uart0;
}
pub const _DREQ_H = "";
pub const UART_DREQ_NUM = @compileError("unable to translate C expr: unexpected token '{'"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h:152:9
pub const UART_CLOCK_NUM = @compileError("unable to translate macro: undefined identifier `clk_peri`"); // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h:166:9
pub inline fn UART_FUNCSEL_NUM(uart: anytype, gpio: anytype) @TypeOf(if (gpio & @as(c_int, 0x2)) GPIO_FUNC_UART_AUX else GPIO_FUNC_UART) {
    _ = &uart;
    _ = &gpio;
    return if (gpio & @as(c_int, 0x2)) GPIO_FUNC_UART_AUX else GPIO_FUNC_UART;
}
pub inline fn UART_IRQ_NUM(uart: anytype) @TypeOf(UART0_IRQ + UART_NUM(uart)) {
    _ = &uart;
    return UART0_IRQ + UART_NUM(uart);
}
pub const _HARDWARE_RESETS_H = "";
pub const _HARDWARE_STRUCTS_RESETS_H = "";
pub const _HARDWARE_REGS_RESETS_H = "";
pub const resets_hw = __helpers.cast([*c]resets_hw_t, RESETS_BASE);
pub const PARAM_ASSERTIONS_ENABLED_HARDWARE_RESETS = @as(c_int, 0);
pub const HARDWARE_RESETS_ENABLE_SDK1XX_COMPATIBILITY = @as(c_int, 1);
pub inline fn UART_RESET_NUM(uart: anytype) @TypeOf(if (uart_get_index(uart)) RESET_UART1 else RESET_UART0) {
    _ = &uart;
    return if (uart_get_index(uart)) RESET_UART1 else RESET_UART0;
}
pub const _PICO_STDIO_USB_H = "";
pub const PICO_STDIO_USB_DEFAULT_CRLF = PICO_STDIO_DEFAULT_CRLF;
pub const PICO_STDIO_USB_STDOUT_TIMEOUT_US = __helpers.promoteIntLiteral(c_int, 500000, .decimal);
pub const PICO_STDIO_USB_TASK_INTERVAL_US = @as(c_int, 1000);
pub const PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK = @as(c_int, 1);
pub const PICO_STDIO_USB_ENABLE_TINYUSB_INIT = @as(c_int, 1);
pub const PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE = @as(c_int, 1);
pub const PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE = @as(c_int, 1200);
pub const PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS = @as(c_int, 0);
pub const PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS = @as(c_int, 50);
pub const PICO_STDIO_USB_DEINIT_DELAY_MS = @as(c_int, 110);
pub const PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED_ACTIVE_LOW = @as(c_int, 0);
pub const PICO_STDIO_USB_RESET_BOOTSEL_FIXED_ACTIVITY_LED = @as(c_int, 0);
pub const PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK = @as(c_uint, 0);
pub const PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE = @as(c_int, 1);
pub const PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL = @as(c_int, 1);
pub const PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT = @as(c_int, 1);
pub const PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR = @as(c_int, 1);
pub const PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS = @as(c_int, 100);
pub const PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS = @as(c_int, 1);
pub const PICO_STDIO_USB_CONNECTION_WITHOUT_DTR = @as(c_int, 0);
pub const PICO_STDIO_USB_DEVICE_SELF_POWERED = @as(c_int, 0);
pub const PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK = @as(c_int, 1);
pub const PICO_DEFAULT_LED_PIN_INVERTED = @as(c_int, 0);
pub const pico_error_codes = enum_pico_error_codes;
pub const gpio_function_rp2350 = enum_gpio_function_rp2350;
pub const irq_num_rp2350 = enum_irq_num_rp2350;
pub const gpio_dir = enum_gpio_dir;
pub const gpio_irq_level = enum_gpio_irq_level;
pub const gpio_override = enum_gpio_override;
pub const gpio_slew_rate = enum_gpio_slew_rate;
pub const gpio_drive_strength = enum_gpio_drive_strength;
pub const alarm_pool = struct_alarm_pool;
pub const repeating_timer = struct_repeating_timer;
pub const __lock = struct___lock;
pub const _Bigint = struct__Bigint;
pub const __tm = struct___tm;
pub const _on_exit_args = struct__on_exit_args;
pub const _atexit = struct__atexit;
pub const __sbuf = struct___sbuf;
pub const __locale_t = struct___locale_t;
pub const _rand48 = struct__rand48;
pub const _reent = struct__reent;
pub const __sFILE = struct___sFILE;
pub const _glue = struct__glue;
pub const timeval = struct_timeval;
pub const timespec = struct_timespec;
pub const itimerspec = struct_itimerspec;
pub const sched_param = struct_sched_param;
pub const timezone = struct_timezone;
pub const bintime = struct_bintime;
pub const itimerval = struct_itimerval;
pub const tm = struct_tm;
pub const rng_128 = struct_rng_128;
pub const ip4_addr = struct_ip4_addr;
pub const pbuf = struct_pbuf;
pub const netif = struct_netif;
pub const lwip_ip_addr_type = enum_lwip_ip_addr_type;
pub const pbuf_rom = struct_pbuf_rom;
pub const memp = struct_memp;
pub const memp_desc = struct_memp_desc;
pub const stats_proto = struct_stats_proto;
pub const stats_igmp = struct_stats_igmp;
pub const stats_mem = struct_stats_mem;
pub const stats_syselem = struct_stats_syselem;
pub const stats_sys = struct_stats_sys;
pub const stats_mib2 = struct_stats_mib2;
pub const stats_mib2_netif_ctrs = struct_stats_mib2_netif_ctrs;
pub const stats_ = struct_stats_;
pub const lwip_internal_netif_client_data_index = enum_lwip_internal_netif_client_data_index;
pub const netif_mac_filter_action = enum_netif_mac_filter_action;
pub const ip4_addr_packed = struct_ip4_addr_packed;
pub const ip_hdr = struct_ip_hdr;
pub const ip_pcb = struct_ip_pcb;
pub const ip_globals = struct_ip_globals;
pub const udp_hdr = struct_udp_hdr;
pub const udp_pcb = struct_udp_pcb;
pub const dhcp = struct_dhcp;
pub const _cyw43_ev_scan_result_t = struct__cyw43_ev_scan_result_t;
pub const _cyw43_async_event_t = struct__cyw43_async_event_t;
pub const _cyw43_wifi_scan_options_t = struct__cyw43_wifi_scan_options_t;
pub const _cyw43_ll_t = struct__cyw43_ll_t;
pub const _cyw43_t = struct__cyw43_t;
pub const async_work_on_timeout = struct_async_work_on_timeout;
pub const async_when_pending_worker = struct_async_when_pending_worker;
pub const async_context_type = struct_async_context_type;
pub const async_context = struct_async_context;
pub const stdio_driver = struct_stdio_driver;
pub const uart_inst = struct_uart_inst;
pub const dreq_num_rp2350 = enum_dreq_num_rp2350;
pub const reset_num_rp2350 = enum_reset_num_rp2350;
// preventing optimization
pub inline fn tight_loop_contents() void {
    asm volatile ("nop");
}
// demoted to external
pub extern fn gpioc_lo_out_put(x: u32) void;
pub extern fn gpioc_lo_out_xor(x: u32) void;
pub extern fn gpioc_lo_out_set(x: u32) void;
pub extern fn gpioc_lo_out_clr(x: u32) void;
pub extern fn gpioc_hi_out_put(x: u32) void;
pub extern fn gpioc_hi_out_xor(x: u32) void;
pub extern fn gpioc_hi_out_set(x: u32) void;
pub extern fn gpioc_hi_out_clr(x: u32) void;
pub extern fn gpioc_hilo_out_put(x: u64) void;
pub extern fn gpioc_hilo_out_xor(x: u64) void;
pub extern fn gpioc_hilo_out_set(x: u64) void;
pub extern fn gpioc_hilo_out_clr(x: u64) void;
pub extern fn gpioc_lo_oe_put(x: u32) void;
pub extern fn gpioc_lo_oe_xor(x: u32) void;
pub extern fn gpioc_lo_oe_set(x: u32) void;
pub extern fn gpioc_lo_oe_clr(x: u32) void;
pub extern fn gpioc_hi_oe_put(x: u32) void;
pub extern fn gpioc_hi_oe_xor(x: u32) void;
pub extern fn gpioc_hi_oe_set(x: u32) void;
pub extern fn gpioc_hi_oe_clr(x: u32) void;
pub extern fn gpioc_hilo_oe_put(x: u64) void;
pub extern fn gpioc_hilo_oe_xor(x: u64) void;
pub extern fn gpioc_hilo_oe_set(x: u64) void;
pub extern fn gpioc_hilo_oe_clr(x: u64) void;
pub extern fn gpioc_bit_out_put(pin: uint, val: bool) void;
pub extern fn gpioc_bit_out_xor(pin: uint) void;
pub extern fn gpioc_bit_out_set(pin: uint) void;
pub extern fn gpioc_bit_out_clr(pin: uint) void;
pub extern fn gpioc_bit_out_xor2(pin: uint, val: bool) void;
pub extern fn gpioc_bit_out_set2(pin: uint, val: bool) void;
pub extern fn gpioc_bit_out_clr2(pin: uint, val: bool) void;
pub extern fn gpioc_bit_oe_put(pin: uint, val: bool) void;
pub extern fn gpioc_bit_oe_xor(pin: uint) void;
pub extern fn gpioc_bit_oe_set(pin: uint) void;
pub extern fn gpioc_bit_oe_clr(pin: uint) void;
pub extern fn gpioc_bit_oe_xor2(pin: uint, val: bool) void;
pub extern fn gpioc_bit_oe_set2(pin: uint, val: bool) void;
pub extern fn gpioc_bit_oe_clr2(pin: uint, val: bool) void;
pub extern fn gpioc_index_out_put(reg_index: uint, val: u32) void;
pub extern fn gpioc_index_out_xor(reg_index: uint, mask: u32) void;
pub extern fn gpioc_index_out_set(reg_index: uint, mask: u32) void;
pub extern fn gpioc_index_out_clr(reg_index: uint, mask: u32) void;
pub extern fn gpioc_index_oe_put(reg_index: uint, val: u32) void;
pub extern fn gpioc_index_oe_xor(reg_index: uint, mask: u32) void;
pub extern fn gpioc_index_oe_set(reg_index: uint, mask: u32) void;
pub extern fn gpioc_index_oe_clr(reg_index: uint, mask: u32) void;
pub extern fn gpioc_lo_out_get() u32;
pub extern fn gpioc_hi_out_get() u32;
pub extern fn gpioc_hilo_out_get() u64;
pub extern fn gpioc_lo_oe_get() u32;
pub extern fn gpioc_hi_oe_get() u32;
pub extern fn gpioc_hilo_oe_get() u64;
pub extern fn gpioc_lo_in_get() u32;
pub extern fn gpioc_hi_in_get() u32;
pub extern fn gpioc_hilo_in_get() u64;

// fix function by removing @bitCast
pub fn bintime_shift(arg__bt: [*c]struct_bintime, arg__exp: c_int) callconv(.c) void {
    var _bt = arg__bt;
    _ = &_bt;
    var _exp = arg__exp;
    _ = &_exp;
    if (_exp > @as(c_int, 0)) {
        _bt.*.sec <<= @intCast(_exp);
        {
            const ref = &_bt.*.sec;
            ref.* = @bitCast(@as(c_ulonglong, @truncate(@as(u64, @bitCast(@as(c_longlong, ref.*))) | (_bt.*.frac >> @intCast(@as(c_longlong, @as(c_int, 64) - _exp))))));
        }
        _bt.*.frac <<= @intCast(@as(c_longlong, _exp));
    } else if (_exp < @as(c_int, 0)) {
        _bt.*.frac >>= @intCast(@as(c_longlong, -_exp));
        _bt.*.frac |= @as(u64, @bitCast(@as(c_longlong, _bt.*.sec))) << @intCast(@as(c_longlong, @as(c_int, 64) + _exp));
        _bt.*.sec >>= @intCast(-_exp);
    }
}

// return some removes consts
pub const RESET_UART0: c_int = 26;
pub const RESET_UART1: c_int = 27;
pub const ACCESSCTRL_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40060000, .hex));
pub const SIO_BASE = _u(__helpers.promoteIntLiteral(c_int, 0xd0000000, .hex));
pub const SIO_NONSEC_BASE = _u(__helpers.promoteIntLiteral(c_int, 0xd0020000, .hex));
pub const SIO_INTERP0_ACCUM0_OFFSET = _u(@as(c_int, 0x00000080));
pub const PADS_BANK0_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40038000, .hex));
pub const M33_NVIC_ISER0_OFFSET = _u(__helpers.promoteIntLiteral(c_int, 0x0000e100, .hex));
pub const M33_CPUID_OFFSET = _u(__helpers.promoteIntLiteral(c_int, 0x0000ed00, .hex));
pub const RESETS_BASE = _u(__helpers.promoteIntLiteral(c_int, 0x40020000, .hex));
