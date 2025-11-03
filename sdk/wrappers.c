// reference: https://zig.news/anders/call-c-from-zig-on-the-pico-47p

#include "hardware/gpio.h"

void __wrap_gpioc_lo_out_put(uint32_t x) { gpioc_lo_out_put(x); }
void __wrap_gpioc_lo_out_xor(uint32_t x) { gpioc_lo_out_xor(x); }
void __wrap_gpioc_lo_out_set(uint32_t x) { gpioc_lo_out_set(x); }
void __wrap_gpioc_lo_out_clr(uint32_t x) { gpioc_lo_out_clr(x); }
void __wrap_gpioc_hi_out_put(uint32_t x) { gpioc_hi_out_put(x); }
void __wrap_gpioc_hi_out_xor(uint32_t x) { gpioc_hi_out_xor(x); }
void __wrap_gpioc_hi_out_set(uint32_t x) { gpioc_hi_out_set(x); }
void __wrap_gpioc_hi_out_clr(uint32_t x) { gpioc_hi_out_clr(x); }
void __wrap_gpioc_hilo_out_put(uint64_t x) { gpioc_hilo_out_put(x); }
void __wrap_gpioc_hilo_out_xor(uint64_t x) { gpioc_hilo_out_xor(x); }
void __wrap_gpioc_hilo_out_set(uint64_t x) { gpioc_hilo_out_set(x); }
void __wrap_gpioc_hilo_out_clr(uint64_t x) { gpioc_hilo_out_clr(x); }
void __wrap_gpioc_lo_oe_put(uint32_t x) { gpioc_lo_oe_put(x); }
void __wrap_gpioc_lo_oe_xor(uint32_t x) { gpioc_lo_oe_xor(x); }
void __wrap_gpioc_lo_oe_set(uint32_t x) { gpioc_lo_oe_set(x); }
void __wrap_gpioc_lo_oe_clr(uint32_t x) { gpioc_lo_oe_clr(x); }
void __wrap_gpioc_hi_oe_put(uint32_t x) { gpioc_hi_oe_put(x); }
void __wrap_gpioc_hi_oe_xor(uint32_t x) { gpioc_hi_oe_xor(x); }
void __wrap_gpioc_hi_oe_set(uint32_t x) { gpioc_hi_oe_set(x); }
void __wrap_gpioc_hi_oe_clr(uint32_t x) { gpioc_hi_oe_clr(x); }
void __wrap_gpioc_hilo_oe_put(uint64_t x) { gpioc_hilo_oe_put(x); }
void __wrap_gpioc_hilo_oe_xor(uint64_t x) { gpioc_hilo_oe_xor(x); }
void __wrap_gpioc_hilo_oe_set(uint64_t x) { gpioc_hilo_oe_set(x); }
void __wrap_gpioc_hilo_oe_clr(uint64_t x) { gpioc_hilo_oe_clr(x); }
void __wrap_gpioc_bit_out_put(uint pin, bool val) {
    gpioc_bit_out_put(pin, val);
}
void __wrap_gpioc_bit_out_xor(uint pin) { gpioc_bit_out_xor(pin); }
void __wrap_gpioc_bit_out_set(uint pin) { gpioc_bit_out_set(pin); }
void __wrap_gpioc_bit_out_clr(uint pin) { gpioc_bit_out_clr(pin); }
void __wrap_gpioc_bit_out_xor2(uint pin, bool val) {
    gpioc_bit_out_xor2(pin, val);
}
void __wrap_gpioc_bit_out_set2(uint pin, bool val) {
    gpioc_bit_out_set2(pin, val);
}
void __wrap_gpioc_bit_out_clr2(uint pin, bool val) {
    gpioc_bit_out_clr2(pin, val);
}
void __wrap_gpioc_bit_oe_put(uint pin, bool val) { gpioc_bit_oe_put(pin, val); }
void __wrap_gpioc_bit_oe_xor(uint pin) { gpioc_bit_oe_xor(pin); }
void __wrap_gpioc_bit_oe_set(uint pin) { gpioc_bit_oe_set(pin); }
void __wrap_gpioc_bit_oe_clr(uint pin) { gpioc_bit_oe_clr(pin); }
void __wrap_gpioc_bit_oe_xor2(uint pin, bool val) {
    gpioc_bit_oe_xor2(pin, val);
}
void __wrap_gpioc_bit_oe_set2(uint pin, bool val) {
    gpioc_bit_oe_set2(pin, val);
}
void __wrap_gpioc_bit_oe_clr2(uint pin, bool val) {
    gpioc_bit_oe_clr2(pin, val);
}
void __wrap_gpioc_index_out_put(uint reg_index, uint32_t val) {
    gpioc_index_out_put(reg_index, val);
}
void __wrap_gpioc_index_out_xor(uint reg_index, uint32_t mask) {
    gpioc_index_out_xor(reg_index, mask);
}
void __wrap_gpioc_index_out_set(uint reg_index, uint32_t mask) {
    gpioc_index_out_set(reg_index, mask);
}
void __wrap_gpioc_index_out_clr(uint reg_index, uint32_t mask) {
    gpioc_index_out_clr(reg_index, mask);
}
void __wrap_gpioc_index_oe_put(uint reg_index, uint32_t val) {
    gpioc_index_oe_put(reg_index, val);
}
void __wrap_gpioc_index_oe_xor(uint reg_index, uint32_t mask) {
    gpioc_index_oe_xor(reg_index, mask);
}
void __wrap_gpioc_index_oe_set(uint reg_index, uint32_t mask) {
    gpioc_index_oe_set(reg_index, mask);
}
void __wrap_gpioc_index_oe_clr(uint reg_index, uint32_t mask) {
    gpioc_index_oe_clr(reg_index, mask);
}
uint32_t __wrap_gpioc_lo_out_get(void) { gpioc_lo_out_get(); }
uint32_t __wrap_gpioc_hi_out_get(void) { gpioc_hi_out_get(); }
uint64_t __wrap_gpioc_hilo_out_get(void) { gpioc_hilo_out_get(); }
uint32_t __wrap_gpioc_lo_oe_get(void) { gpioc_lo_oe_get(); }
uint32_t __wrap_gpioc_hi_oe_get(void) { gpioc_hi_oe_get(); }
uint64_t __wrap_gpioc_hilo_oe_get(void) { gpioc_hilo_oe_get(); }
uint32_t __wrap_gpioc_lo_in_get(void) { gpioc_lo_in_get(); }
uint32_t __wrap_gpioc_hi_in_get(void) { gpioc_hi_in_get(); }
uint64_t __wrap_gpioc_hilo_in_get(void) { gpioc_hilo_in_get(); }

/* void __wrap_gpioc_bit_out_put(uint pin, bool val) { */
/*     gpioc_bit_out_put(pin, val); */
/* } */

/* void __wrap_gpioc_bit_oe_put(uint pin, bool val) { gpioc_bit_oe_put(pin,
 * val); } */
