#!/bin/bash -e
set -e

zig=~/.build/zig/zig-x86_64-linux-0.15.1/zig

export PICO_SDK_PATH=~/Code/pico/pico-sdk
$zig build -Doptimize=ReleaseSafe -freference-trace=10 --verbose-cimport

picotool load -x -f zig-out/firmware.uf2

exit 0

cd ~/Code/pico/
git clone git@github.com:raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
