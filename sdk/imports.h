typedef int wint_t;

#include <hardware/adc.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

/* #include <pico/bootrom.h> */
/* #include <pico/multicore.h> */
/* #include <pico/platform.h> */
/* #include <pico/sync.h> */

/* #include <hardware/adc.h> */
/* // #include <hardware/clocks.h> // ovaj se da ispraviti u source */
/* #include <hardware/divider.h> */
/* // #include <hardware/dma.h> */
/* #include <hardware/flash.h> */
/* #include <hardware/gpio.h> */
/* #include <hardware/i2c.h> */
/* // #include <hardware/interp.h> */
/* #include <hardware/irq.h> */
/* #include <hardware/pio.h> */
/* #include <hardware/pll.h> */
/* // #include <hardware/pwm.h> */
/* #include <hardware/regs/addressmap.h> */
/* #include <hardware/regs/usb.h> */
/* #include <hardware/resets.h> */
/* // #include <hardware/rtc.h> // isto source */
/* #include <hardware/spi.h> */
/* // #include <hardware/sync.h> */
/* #include <hardware/timer.h> */
/* #include <hardware/uart.h> */
/* #include <hardware/vreg.h> */
/* #include <hardware/watchdog.h> */
/* #include <hardware/xosc.h> */

/*
   -> pico/stdlib.h will pull:
   hardware_divider
   hardware_gpio
   hardware_uart
   pico_runtime
   pico_platform
   pico_stdio
   pico_time
   pico_util

   -> pico/cyw43_arch.h will pull:
   stdint.h
   stdio.h
*/
