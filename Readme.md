# Usage
Set up the pico-sdk as described in the Getting Started guide and set the PICO_SDK_PATH environment variable. Then run `zig build`. A `firmware.uf2` should appear in the zig-out.

The current configuration is for the Pico W and this is basically the Pico W blink example. You can set -DPICO_BOARD at the top of build.zig.

Like in the C Version, you need to adjust the linked libraries in the CMakeLists.txt.

To build this example with the regular non-W Pico, remove the `pico_cyw43_arch_none` library from CMakeLists and rewrite `main.zig` to not use the W-specific cyw43 functions.

**A zig 0.13 version is available in branch "0.13"**

Made with zig 0.14.0-dev.2457+82f35c518 and Pico-SDK 2.1.0
