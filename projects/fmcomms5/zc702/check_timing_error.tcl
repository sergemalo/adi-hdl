# check_timing_error.tcl
open_checkpoint fmcomms5_zc702.runs/impl_1/system_top_routed.dcp

set p [lindex [get_timing_paths -setup -max_paths 1] 0]
puts ">>> Slack          : [get_property SLACK $p] ns"
puts ">>> Startpoint     : [get_property STARTPOINT_PIN $p]"
puts ">>> Endpoint       : [get_property ENDPOINT_PIN $p]"
puts ">>> Launch  clk    : [get_property STARTPOINT_CLOCK $p]"
puts ">>> Capture clk    : [get_property ENDPOINT_CLOCK $p]"
puts ">>> Datapath delay : [get_property DATAPATH_DELAY $p] ns"
puts ">>> Logic levels   : [get_property LOGIC_LEVELS $p]"
puts ">>> Requirement    : [get_property REQUIREMENT $p] ns"
foreach c [get_clocks] {
  puts [format ">>> clock %-18s period %s ns" $c [get_property PERIOD $c]]
}