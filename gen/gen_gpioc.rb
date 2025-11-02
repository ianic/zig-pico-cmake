signatures=<<~TEXT
void gpioc_lo_out_put(uint32_t x) {
void gpioc_lo_out_xor(uint32_t x) {
void gpioc_lo_out_set(uint32_t x) {
void gpioc_lo_out_clr(uint32_t x) {
void gpioc_hi_out_put(uint32_t x) {
void gpioc_hi_out_xor(uint32_t x) {
void gpioc_hi_out_set(uint32_t x) {
void gpioc_hi_out_clr(uint32_t x) {
void gpioc_hilo_out_put(uint64_t x) {
void gpioc_hilo_out_xor(uint64_t x) {
void gpioc_hilo_out_set(uint64_t x) {
void gpioc_hilo_out_clr(uint64_t x) {
void gpioc_lo_oe_put(uint32_t x) {
void gpioc_lo_oe_xor(uint32_t x) {
void gpioc_lo_oe_set(uint32_t x) {
void gpioc_lo_oe_clr(uint32_t x) {
void gpioc_hi_oe_put(uint32_t x) {
void gpioc_hi_oe_xor(uint32_t x) {
void gpioc_hi_oe_set(uint32_t x) {
void gpioc_hi_oe_clr(uint32_t x) {
void gpioc_hilo_oe_put(uint64_t x) {
void gpioc_hilo_oe_xor(uint64_t x) {
void gpioc_hilo_oe_set(uint64_t x) {
void gpioc_hilo_oe_clr(uint64_t x) {
void gpioc_bit_out_put(uint pin, bool val) {
void gpioc_bit_out_xor(uint pin) {
void gpioc_bit_out_set(uint pin) {
void gpioc_bit_out_clr(uint pin) {
void gpioc_bit_out_xor2(uint pin, bool val) {
void gpioc_bit_out_set2(uint pin, bool val) {
void gpioc_bit_out_clr2(uint pin, bool val) {
void gpioc_bit_oe_put(uint pin, bool val) {
void gpioc_bit_oe_xor(uint pin) {
void gpioc_bit_oe_set(uint pin) {
void gpioc_bit_oe_clr(uint pin) {
void gpioc_bit_oe_xor2(uint pin, bool val) {
void gpioc_bit_oe_set2(uint pin, bool val) {
void gpioc_bit_oe_clr2(uint pin, bool val) {
void gpioc_index_out_put(uint reg_index, uint32_t val) {
void gpioc_index_out_xor(uint reg_index, uint32_t mask) {
void gpioc_index_out_set(uint reg_index, uint32_t mask) {
void gpioc_index_out_clr(uint reg_index, uint32_t mask) {
void gpioc_index_oe_put(uint reg_index, uint32_t val) {
void gpioc_index_oe_xor(uint reg_index, uint32_t mask) {
void gpioc_index_oe_set(uint reg_index, uint32_t mask) {
void gpioc_index_oe_clr(uint reg_index, uint32_t mask) {
uint32_t gpioc_lo_out_get(void) {
uint32_t gpioc_hi_out_get(void) {
uint64_t gpioc_hilo_out_get(void) {
uint32_t gpioc_lo_oe_get(void) {
uint32_t gpioc_hi_oe_get(void) {
uint64_t gpioc_hilo_oe_get(void) {
uint32_t gpioc_lo_in_get(void) {
uint32_t gpioc_hi_in_get(void) {
uint64_t gpioc_hilo_in_get(void) {
TEXT

signatures.each_line do |sig|
  parts = sig.split(/[ \()]/).compact
  args=parts[2..parts.length-3]
  print "#{parts[0]} __wrap_#{parts[1]}("

  if (args.length==1)
    print "#{args[0]})"
  elsif (args.length==2)
    print "#{args[0]} #{args[1]})"
  elsif (args.length==4)
    print "#{args[0]} #{args[1]} #{args[2]} #{args[3]})"
  end

  print "{ #{parts[1]}("
  if (args.length==1)
    print ")"
  elsif (args.length==2)
    print "#{args[1]})"
  elsif (args.length==4)
    print "#{args[1]} #{args[3]})"
  end
  print "; }\n"
  #p sig
end

signatures.each_line do |sig|
  parts = sig.split(/[ \()]/)
  print "-Wl,--wrap=#{parts[1]}\n"
end


zig_signatures=<<~TEXT
pub extern fn gpioc_lo_out_put(arg_x: u32) void;
pub extern fn gpioc_lo_out_xor(arg_x: u32) void;
pub extern fn gpioc_lo_out_set(arg_x: u32) void;
pub extern fn gpioc_lo_out_clr(arg_x: u32) void;
pub extern fn gpioc_hi_out_put(arg_x: u32) void;
pub extern fn gpioc_hi_out_xor(arg_x: u32) void;
pub extern fn gpioc_hi_out_set(arg_x: u32) void;
pub extern fn gpioc_hi_out_clr(arg_x: u32) void;
pub extern fn gpioc_hilo_out_put(arg_x: u64) void;
pub extern fn gpioc_hilo_out_xor(arg_x: u64) void;
pub extern fn gpioc_hilo_out_set(arg_x: u64) void;
pub extern fn gpioc_hilo_out_clr(arg_x: u64) void;
pub extern fn gpioc_lo_oe_put(arg_x: u32) void;
pub extern fn gpioc_lo_oe_xor(arg_x: u32) void;
pub extern fn gpioc_lo_oe_set(arg_x: u32) void;
pub extern fn gpioc_lo_oe_clr(arg_x: u32) void;
pub extern fn gpioc_hi_oe_put(arg_x: u32) void;
pub extern fn gpioc_hi_oe_xor(arg_x: u32) void;
pub extern fn gpioc_hi_oe_set(arg_x: u32) void;
pub extern fn gpioc_hi_oe_clr(arg_x: u32) void;
pub extern fn gpioc_hilo_oe_put(arg_x: u64) void;
pub extern fn gpioc_hilo_oe_xor(arg_x: u64) void;
pub extern fn gpioc_hilo_oe_set(arg_x: u64) void;
pub extern fn gpioc_hilo_oe_clr(arg_x: u64) void;
pub extern fn gpioc_bit_out_put(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_out_xor(arg_pin: uint) void;
pub extern fn gpioc_bit_out_set(arg_pin: uint) void;
pub extern fn gpioc_bit_out_clr(arg_pin: uint) void;
pub extern fn gpioc_bit_out_xor2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_out_set2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_out_clr2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_oe_put(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_oe_xor(arg_pin: uint) void;
pub extern fn gpioc_bit_oe_set(arg_pin: uint) void;
pub extern fn gpioc_bit_oe_clr(arg_pin: uint) void;
pub extern fn gpioc_bit_oe_xor2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_oe_set2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_bit_oe_clr2(arg_pin: uint, arg_val: bool) void;
pub extern fn gpioc_index_out_put(arg_reg_index: uint, arg_val: u32) void;
pub extern fn gpioc_index_out_xor(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_index_out_set(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_index_out_clr(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_index_oe_put(arg_reg_index: uint, arg_val: u32) void;
pub extern fn gpioc_index_oe_xor(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_index_oe_set(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_index_oe_clr(arg_reg_index: uint, arg_mask: u32) void;
pub extern fn gpioc_lo_out_get() u32;
pub extern fn gpioc_hi_out_get() u32;
pub extern fn gpioc_hilo_out_get() u64;
pub extern fn gpioc_lo_oe_get() u32;
pub extern fn gpioc_hi_oe_get() u32;
pub extern fn gpioc_hilo_oe_get() u64;
pub extern fn gpioc_lo_in_get() u32;
pub extern fn gpioc_hi_in_get() u32;
pub extern fn gpioc_hilo_in_get() u64;

pub inline fn tight_loop_contents() void {
    asm volatile ("nop");
}
TEXT
