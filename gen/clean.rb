out = "zig-out/c_sdk.zig"
content = File.read out

#p content

skip_comment = false
content.each_line do |line|

  #puts line.chomp  # process each line here
  if line =~ /^pub const ([A-Z_a-z0-9]+) =.*;$/
    constant_name = $1
    count = content.scan(/\b#{Regexp.escape(constant_name)}\b/i).size
    line_count = line.scan(/\b#{Regexp.escape(constant_name)}\b/i).sizeOf
    skip_comment = line.include?("@compileError")
    # count = content.scan(constant_name).size
    # line_count = line.scan(constant_name).size
    next if count == line_count
  else
    skip = (line.start_with?("//") and skip_comment)
    skip_comment = false
    next if skip
  end
  puts line.chomp
end
