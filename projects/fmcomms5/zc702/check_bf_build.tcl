# check_bf_build.tcl -- B4 post-implementation checks for BFW4 (rev 5, RTL rev 2).
# Run in the implemented design, e.g. (headless):
#   vivado -mode batch -source check_bf_build.tcl -tclargs fmcomms5_zc702.xpr
# or `source check_bf_build.tcl` with impl_1 already open in the GUI.
if {[llength $argv] > 0} { open_project [lindex $argv 0]; open_run impl_1 }

proc cnt  {f} { return [llength [get_cells -quiet -hier -filter $f]] }
proc pcnt {f} { return [llength [get_pins  -quiet -hier -filter $f]] }
proc row {name got exp} {
  set ok [expr {$got == $exp ? "ok" : "CHECK"}]
  puts [format "  %-48s %6s   expect %-6s %s" $name $got $exp $ok]
}

puts "\n=== BFW4 build checks ==="
row "DSP48E1, whole design"                   [cnt {REF_NAME == DSP48E1}] 208
row "DSP48E1 in bf_bank_0"                    [cnt {REF_NAME == DSP48E1 && NAME =~ *bf_bank_0*}] 16
row "CARRY4 in bf_bank_0 (fabric adders)"     [cnt {REF_NAME == CARRY4 && NAME =~ *bf_bank_0*}] 0
row "bank-select synchronisers (sel_sync)"    [cnt {NAME =~ *bf_bank_0*sel_sync_reg* && IS_SEQUENTIAL}] 1
row "bfx_m1 synchroniser first stages"        [cnt {NAME =~ *bfx_m1_* && IS_SEQUENTIAL}] 38
# Proof that rev 2 (OUT_EN / FAULTED / SLIP) reached the netlist and not a
# cached rev 1 (the ipcache trap): these registers do not exist in rev 1.
row "rev 2 in netlist: slip_w_toggle"         [cnt {NAME =~ *bf_dac_if_0*slip_w_toggle_reg* && IS_SEQUENTIAL}] 1
row "rev 2 in netlist: st_faulted"            [cnt {NAME =~ *axi_bf_ctrl*st_faulted_reg* && IS_SEQUENTIAL}] 1
# 25, not 33: bits [3:0] of both DAC words are constant 0 and are optimised away
row "FIFO max_delay: rd_word D pins"          [pcnt {NAME =~ *bf_dac_if_0/inst/rd_word_reg*/D}] 25
puts [format "  %-48s %6s   expect > 0" "FIFO max_delay: memory CLK pins" \
      [pcnt {NAME =~ *bf_dac_if_0/inst/mem_reg*/CLK}]]

set fp [get_timing_paths -quiet -setup \
          -from [get_pins -quiet -hier -filter {NAME =~ *bf_dac_if_0/inst/mem_reg*/CLK}] \
          -to   [get_pins -quiet -hier -filter {NAME =~ *bf_dac_if_0/inst/rd_word_reg*/D}]]
if {[llength $fp]} {
  puts [format "  %-48s %6s   (requirement %s ns)" "FIFO memory -> read register slack" \
        [get_property SLACK $fp] [get_property REQUIREMENT $fp]]
}

# Is the FIFO exception present in THIS design? Without it the path is timed
# synchronously against ~4.9 ns of clock skew and cannot have positive slack.
row "FIFO set_max_delay -datapath_only effective" \
    [expr {[llength $fp] && [get_property SLACK $fp] > 0 ? 1 : 0}] 1

set wns [get_property SLACK [get_timing_paths -setup]]
set whs [get_property SLACK [get_timing_paths -hold]]
puts [format "  %-48s %6s" "WNS (ns)" $wns]
puts [format "  %-48s %6s" "WHS (ns)" $whs]
set wp [get_timing_paths -setup]
puts [format "  %-48s %s -> %s" "worst setup path clocks" \
      [get_property STARTPOINT_CLOCK $wp] [get_property ENDPOINT_CLOCK $wp]]
puts [format "  %-48s %s" "worst setup path endpoint" [get_property ENDPOINT_PIN $wp]]

puts "\n  failing setup endpoints, whole design (first 10):"
set bad [get_timing_paths -quiet -setup -max_paths 10 -nworst 1 -slack_lesser_than 0]
if {[llength $bad]} {
  foreach p $bad {
    puts [format "    %7s  %-22s -> %-22s  %s" [get_property SLACK $p] \
          [get_property STARTPOINT_CLOCK $p] [get_property ENDPOINT_CLOCK $p] \
          [get_property ENDPOINT_PIN $p]]
  }
} else { puts "    (none)" }

puts "\n  utilisation of the BFW4 cells:"
report_utilization -hierarchical -hierarchical_depth 4 -cells \
  [get_cells -quiet -hier -filter {NAME =~ *bf_bank_0 || NAME =~ *bf_dac_if_0 || NAME =~ *bf_capture_0 || NAME =~ *axi_bf_ctrl}]

puts "\n  CDC report lines mentioning BFW4 cells:"
set n 0
foreach line [split [report_cdc -details -return_string] "\n"] {
  if {[regexp {bf_bank_0|bf_dac_if_0|bf_capture_0|axi_bf_ctrl} $line]} { puts "    $line"; incr n }
}
if {$n == 0} { puts "    (none)" }
