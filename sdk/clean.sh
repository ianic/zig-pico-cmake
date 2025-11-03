#!/bin/bash
set -e

cd $(git rev-parse --show-toplevel)

out=src/sdk.zig

cp zig-out/sdk.zig $out

echo before cleanup "$(wc -l $out)"

# remove constants
sed -i '/^pub const IO_BANK.*;$/d' $out
sed -i '/^pub const .*IO_BANK.*;$/d' $out
sed -i '/^pub const M33_.*;$/d' $out
sed -i '/^pub const ACCESSCTRL_.*;$/d' $out
sed -i '/^pub const PADS_.*;$/d' $out
sed -i '/^pub const SIO_.*;$/d' $out
sed -i '/^pub const UART_.*;$/d' $out
sed -i '/^pub const RESET_.*;$/d' $out
sed -i '/^pub const RESETS_.*;$/d' $out

# remove this line
sed -i '/^pub inline fn tight_loop_contents.*/d' $out
# remove bintime_shift function
sed -i '/^pub fn bintime_shift/,/^}/d' $out
# remove all gpioc_ functions
sed -i '/^pub inline fn gpioc_lo_out_put/,/^pub const GPIO_OUT/{ /^pub const GPIO_OUT/!d }' $out

# add fixes
cat sdk/fixes.zig >>$out

echo after $(wc -l $out)

# zig build

# pub const io_bank0_hw = __helpers.cast([*c]io_bank0_hw_t, IO_BANK0_BASE);
