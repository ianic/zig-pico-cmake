const std = @import("std");

const config: Config = .{
    .board = .pico2_w,
    .stdio_output = .usb,
    .pico_sdk_path = "../pico-sdk",
};

pub fn build(b: *std.Build) anyerror!void {
    const target = b.resolveTargetQuery(std.Target.Query{
        .abi = .eabi,
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = config.board.cpuModel() },
        .os_tag = .freestanding,
    });
    const optimize = b.standardOptimizeOption(.{});
    const root = b.build_root.handle;
    try root.setAsCwd();

    // Create build directory
    root.makeDir("build") catch |err| {
        if (err != error.PathAlreadyExists) return err;
    };

    // Find and check pico sdk location
    const pico_sdk_path = brk: {
        const pico_sdk_path = if (config.pico_sdk_path) |sdk_path|
            try root.realpathAlloc(b.allocator, sdk_path)
        else
            std.process.getEnvVarOwned(b.allocator, "PICO_SDK_PATH") catch |err| {
                std.log.err("The Pico SDK path must be set either through the PICO_SDK_PATH environment variable or at the top of build.zig.", .{});
                return err;
            };
        // perform basic verification on the pico sdk path
        // if the sdk path contains the pico_sdk_init.cmake file then we know its correct
        const pico_init_cmake_path = b.pathJoin(&.{ pico_sdk_path, "pico_sdk_init.cmake" });
        root.access(pico_init_cmake_path, .{}) catch {
            std.log.err(
                \\Provided Pico SDK path does not contain the file pico_sdk_init.cmake
                \\Tried: {s}
                \\Are you sure you entered the path correctly?"
            , .{pico_init_cmake_path});
            return error.FileNotFound;
        };
        break :brk pico_sdk_path;
    };

    const lib = b.addObject(.{
        .name = "zig-pico",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .link_libc = true,
        }),
    });
    const lib_install = b.addInstallFile(lib.getEmittedBin(), "lib.o");
    lib_install.step.dependOn(&lib.step);

    const cmake_argv = [_][]const u8{
        "cmake",
        "-B",
        "./build",
        "-S .",
        "-DPICO_BOARD=" ++ @tagName(config.board),
        "-DPICO_PLATFORM=" ++ @tagName(config.board.platform()),
        b.fmt("-DPICO_SDK_PATH={s}", .{pico_sdk_path}),
        if (config.stdio_output == .usb) "-DSTDIO_USB=1" else "-DSTDIO_UART=1",
    };
    const cmake_step = b.addSystemCommand(&cmake_argv);
    cmake_step.step.dependOn(&lib_install.step);

    const make_argv = [_][]const u8{ "cmake", "--build", "./build", "--parallel" };
    const make_step = b.addSystemCommand(&make_argv);
    make_step.step.dependOn(&cmake_step.step);

    const firmware_install = b.addInstallFile(b.path("build/app.uf2"), "firmware.uf2");
    firmware_install.step.dependOn(&make_step.step);

    const uf2_step = b.step("uf2", "Create firmware.uf2");
    uf2_step.dependOn(&firmware_install.step);
    b.default_step = uf2_step;

    // c_sdk.zig generation
    // not run by default
    // use `zig build gen` to run
    // src/c_sdk.h defines which headers will be included
    {
        const sdk = b.addTranslateC(.{
            .root_source_file = b.path("src/c_sdk.h"),
            .target = target,
            .optimize = optimize,
        });
        defineMacros(sdk);

        const config_autogen = brk: {
            // Find the board header
            const board_header = blk: {
                const file_name = @tagName(config.board) ++ ".h";
                const path = b.pathJoin(&.{ pico_sdk_path, "src/boards/include/boards", file_name });
                root.access(path, .{}) catch |err| {
                    std.log.err("Could not find the header file for board at '{s}'\n", .{path});
                    return err;
                };
                break :blk file_name;
            };
            // Autogenerate the header file like the pico sdk would
            const cmsys_exception_prefix = if (config.board.platform() == .rp2040) "" else "//";
            const header_str = try std.fmt.allocPrint(b.allocator,
                \\#include "{s}/src/boards/include/boards/{s}"
                \\{s}#include "{s}/src/rp2_common/cmsis/include/cmsis/rename_exceptions.h"
            , .{ pico_sdk_path, board_header, cmsys_exception_prefix, pico_sdk_path });

            // Write and include the generated header
            break :brk b.addWriteFile("pico/config_autogen.h", header_str);
        };
        sdk.step.dependOn(&config_autogen.step);
        sdk.addIncludePath(config_autogen.getDirectory());

        // Standard libary headers may be in different locations on different platforms
        const arm_header_path = blk: {
            if (config.arm_none_eabi_path) |path| {
                break :blk path;
            }

            if (std.process.getEnvVarOwned(b.allocator, "ARM_NONE_EABI_PATH") catch null) |path| {
                break :blk path;
            }

            const unix_path = "/usr/arm-none-eabi/include";
            if (std.fs.accessAbsolute(unix_path, .{})) |_| {
                break :blk unix_path;
            } else |_| {}

            std.log.err(
                \\Could not determine ARM Toolchain include directory.
                \\Please set the ARM_NONE_EABI_PATH environment variable with the correct path
                \\or set the ARMNoneEabiPath variable at the top of build.zig
            , .{});
            return error.FileNotFound;
        };
        sdk.addSystemIncludePath(.{ .cwd_relative = arm_header_path });

        // Find all folders called include in the Pico SDK folder
        {
            const pico_sdk_src = try std.fmt.allocPrint(b.allocator, "{s}/src", .{pico_sdk_path});
            var dir = try root.openDir(pico_sdk_src, .{
                .iterate = true,
                .follow_symlinks = false,
            });

            const allowed_paths = [_][]const u8{ @tagName(config.board.platform()), "rp2_common", "common" };

            var walker = try dir.walk(b.allocator);
            defer walker.deinit();
            while (try walker.next()) |entry| {
                if (std.mem.eql(u8, entry.basename, "include")) {
                    for (allowed_paths) |path| {
                        if (std.mem.indexOf(u8, entry.path, path)) |_| {
                            const pico_sdk_include = try std.fmt.allocPrint(b.allocator, "{s}/src/{s}", .{ pico_sdk_path, entry.path });
                            sdk.addIncludePath(.{ .cwd_relative = pico_sdk_include });
                            continue;
                        }
                    }
                }
            }
        }

        if (config.board.wifi()) {
            // required for pico_w wifi
            sdk.defineCMacro("PICO_CYW43_ARCH_THREADSAFE_BACKGROUND", "1");
            const cyw43_include = try std.fmt.allocPrint(b.allocator, "{s}/lib/cyw43-driver/src", .{pico_sdk_path});
            sdk.addIncludePath(.{ .cwd_relative = cyw43_include });
            // required by cyw43
            const lwip_include = try std.fmt.allocPrint(b.allocator, "{s}/lib/lwip/src/include", .{pico_sdk_path});
            sdk.addIncludePath(.{ .cwd_relative = lwip_include });
            // options headers
            sdk.addIncludePath(b.path("config/"));
        }

        // requires running cmake at least once
        sdk.addSystemIncludePath(b.path("build/generated/pico_base"));

        const sdk_install = b.addInstallFile(sdk.getOutput(), "c_sdk.zig");
        sdk_install.step.dependOn(&sdk.step);

        const gen_step = b.step("gen", "Translate C SDK headers to Zig (result: zig-out/c_sdk.zig)");
        gen_step.dependOn(&sdk_install.step);
    }
}

const Config = struct {
    board: Board,
    stdio_output: StdioOutput,
    pico_sdk_path: ?[]const u8 = null,
    arm_none_eabi_path: ?[]const u8 = null,
};

const StdioOutput = enum {
    usb,
    uart,
};

const Board = enum {
    pico,
    pico_w,
    pico2,
    pico2_w,

    const Platform = enum {
        rp2040,
        rp2350,
    };

    fn cpuModel(board: Board) *const std.Target.Cpu.Model {
        return switch (board) {
            .pico, .pico_w => &std.Target.arm.cpu.cortex_m0plus,
            .pico2, .pico2_w => &std.Target.arm.cpu.cortex_m33,
        };
    }

    inline fn platform(board: Board) Platform {
        return switch (board) {
            .pico, .pico_w => .rp2040,
            .pico2, .pico2_w => .rp2350,
        };
    }

    fn wifi(board: Board) bool {
        return switch (board) {
            .pico_w, .pico2_w => true,
            .pico, .pico2 => false,
        };
    }
};

pub fn defineMacros(sdk: *std.Build.Step.TranslateC) void {
    // Define UART or USB constant for headers
    sdk.defineCMacro(
        if (config.stdio_output == .usb) "LIB_PICO_STDIO_USB" else "LIB_PICO_STDIO_UART",
        "1",
    );

    switch (config.board.platform()) {
        .rp2040 => {
            sdk.defineCMacro("PICO_RP2040", "1");
            sdk.defineCMacro("PICO_32BIT", "1");
            sdk.defineCMacro("PICO_ARM", "1");
            sdk.defineCMacro("PICO_CMSIS_DEVICE", "RP2040");
            sdk.defineCMacro("PICO_DEFAULT_FLASH_SIZE_BYTES", "\"2 * 1024 * 1024\"");
        },
        .rp2350 => {
            // from sdk: src/rp2350-arm-s.cmake
            sdk.defineCMacro("PICO_RP2350", "1");
            sdk.defineCMacro("PICO_32BIT", "1");
            sdk.defineCMacro("PICO_ARM", "1");
            sdk.defineCMacro("PICO_PIO_VERSION", "1");
            sdk.defineCMacro("NUM_DOORBELLS", "1");
            sdk.defineCMacro("PICO_CMSIS_DEVICE", "RP2350");
            sdk.defineCMacro("PICO_DEFAULT_FLASH_SIZE_BYTES", "\"4 * 1024 * 1024\"");

            // can't translate spinlock without this:
            // /home/ianic/Code/pico/pico-sdk/src/rp2_common/hardware_sync_spin_lock/include/hardware/sync/spin_lock.h:165:2: error: no SW_SPIN_LOCK_LOCK available for PICO_USE_SW_SPIN_LOCK on this platform
            // #error no SW_SPIN_LOCK_LOCK available for PICO_USE_SW_SPIN_LOCK on this platform
            sdk.defineCMacro("__ARM_ARCH_8M_MAIN__", "1");
        },
    }
}
