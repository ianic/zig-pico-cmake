#!/bin/bash
set -e

cp zig-out/c_sdk.zig src/c_sdk.zig

echo before cleanup "$(wc -l src/c_sdk.zig)"

# remove constants
sed -i '/^pub const IO_BANK.*;$/d' src/c_sdk.zig
sed -i '/^pub const .*IO_BANK.*;$/d' src/c_sdk.zig
sed -i '/^pub const M33_.*;$/d' src/c_sdk.zig
sed -i '/^pub const ACCESSCTRL_.*;$/d' src/c_sdk.zig
sed -i '/^pub const PADS_.*;$/d' src/c_sdk.zig
sed -i '/^pub const SIO_.*;$/d' src/c_sdk.zig
sed -i '/^pub const UART_.*;$/d' src/c_sdk.zig
sed -i '/^pub const RESET_.*;$/d' src/c_sdk.zig
sed -i '/^pub const RESETS_.*;$/d' src/c_sdk.zig

# remove this line
sed -i '/^pub inline fn tight_loop_contents.*/d' src/c_sdk.zig
# remove bintime_shift function
sed -i '/^pub fn bintime_shift/,/^}/d' src/c_sdk.zig
# remove all gpioc_ functions
sed -i '/^pub inline fn gpioc_lo_out_put/,/^pub const GPIO_OUT/{ /^pub const GPIO_OUT/!d }' src/c_sdk.zig

# add fixes
cat src/c_sdk_fixes.zig >>src/c_sdk.zig

echo after $(wc -l src/c_sdk.zig)

zig build

# pub const io_bank0_hw = __helpers.cast([*c]io_bank0_hw_t, IO_BANK0_BASE);
