#!/bin/bash -e
set -e

rm -rf zig-out

zig=~/.build/zig/zig-x86_64-linux-0.15.2/zig
#zig=~/.build/zig/zig-x86_64-linux-0.16.0-dev.747+493ad58ff/zig

$zig build -Doptimize=ReleaseSafe -freference-trace=10
$zig build gen

#picotool load -x -f zig-out/firmware.uf2

exit 0

cd ~/Code/pico/
git clone git@github.com:raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
