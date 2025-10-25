#!/bin/bash -e
set -e

zig=~/.build/zig/zig-x86_64-linux-0.14.1/zig
export PICO_SDK_PATH=~/Code/pico/pico-sdk
$zig build -freference-trace=10

picotool load -x zig-out/firmware.uf2

exit 0

cd ~/Code/pico/
git clone git@github.com:raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
