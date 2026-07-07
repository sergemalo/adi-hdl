# check_iq_timing.tcl  —  run: vivado -mode batch -source check_iq_timing.tcl
open_checkpoint fmcomms5_zc702.runs/impl_1/system_top_routed.dcp

# Design-wide worst slacks (the WNS/WHS the gate checks):
puts "WNS setup = [get_property SLACK [get_timing_paths -setup -max_paths 1]] ns"
puts "WHS hold  = [get_property SLACK [get_timing_paths -hold  -max_paths 1]] ns"

# Worst path passing THROUGH your override cell:
set c [get_cells -hierarchical -filter {NAME =~ *iq_override_0*}]
puts "Matched: $c"
if {[llength $c] > 0} {
  report_timing -through $c -delay_type max -nworst 1 -max_paths 5 \
    -sort_by slack -file iq_override_timing.rpt
  puts "Wrote iq_override_timing.rpt"
} else {
  puts "No iq_override_0 in routed netlist — optimized to constants (see note)."
}