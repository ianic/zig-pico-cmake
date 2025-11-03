input = "zig-out/c_sdk.zig"
content = File.read input

consts = <<~TEXT
pub const __builtin = @import("std").zig.c_translation.builtins;
pub const __helpers = @import("std").zig.c_translation.helpers;
pub const _u = __helpers.U_SUFFIX;

TEXT
fn=""
count=0
total=0
used=0

content.each_line do |line|
  total += 1
  if line =~ /^pub const ([_A-Z0-9]+)[:]* .*=.*;.*/
    consts += line
    count += 1

    constant_name = $1
    is_used = content.scan(/\b#{Regexp.escape(constant_name)}\b/i).size > line.scan(/\b#{Regexp.escape(constant_name)}\b/i).size
    fn += line if is_used
  else
    fn += line
  end
end

content = fn
fn = ""
content.each_line do |line|
  total += 1
  if line =~ /^pub const ([_A-Z0-9]+)[:]* .*=.*;.*/
    constant_name = $1
    is_used = content.scan(/\b#{Regexp.escape(constant_name)}\b/i).size > line.scan(/\b#{Regexp.escape(constant_name)}\b/i).size
    used += 1 if is_used
    fn += line if is_used
  else
    fn += line
  end
end

print "const lines: #{count} of #{total} #{(count.to_f/total*100.0).to_i}% used: #{used} #{(used.to_f/total*100).to_i}% \n"

File.open("src/c_sdk_fn.zig", "w") do |file|
  file.puts fn
  file.puts "\n"
  file.puts File.read "src/c_sdk_fixes.zig"
end
File.open("src/c_sdk_const.zig", "w") do |file|
  file.puts consts
end

exit 0


# removed=""
# cleaned=""
# const=""

# 5.times do |i|
#   cleaned = ""
#   content.each_line do |line|

#     if line =~ /^pub const ([A-Z_a-z0-9]+)[:]* .*=.*;.*/
#       const += line if i == 0

#       constant_name = $1
#       count = content.scan(/\b#{Regexp.escape(constant_name)}\b/i).size
#       line_count = line.scan(/\b#{Regexp.escape(constant_name)}\b/i).size
#       if count == line_count
#         removed += line
#         print "#{i} #{line}"
#         # puts line
#         next
#       end
#     end
#     cleaned += line
#   end
#   content = cleaned
# end

# File.open("src/c_sdk_fn.zig", "w") do |file|
#   file.puts cleaned
# end
# File.open("src/s_sdk_const.zig", "w") do |file|
#   file.puts const
# end
