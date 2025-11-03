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
