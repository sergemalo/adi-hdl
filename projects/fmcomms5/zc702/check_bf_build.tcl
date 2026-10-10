# check_bf_build.tcl -- B4 post-implementation checks for BFW4 (rev 2).
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
row "bfx_m1 synchroniser first stages"        [cnt {NAME =~ *bfx_m1_* && IS_SEQUENTIAL}] 20
# 24, not 32: bits [3:0] of both DAC words are constant 0 and are optimised away
row "FIFO max_delay: read-register D pins"    [pcnt {NAME =~ *bf_dac_if_0/inst/dac_data_*_reg*/D}] 24
puts [format "  %-48s %6s   expect > 0" "FIFO max_delay: memory CLK pins" \
      [pcnt {NAME =~ *bf_dac_if_0/inst/mem_reg*/CLK}]]

set fp [get_timing_paths -quiet -setup \
          -from [get_pins -quiet -hier -filter {NAME =~ *bf_dac_if_0/inst/mem_reg*/CLK}] \
          -to   [get_pins -quiet -hier -filter {NAME =~ *bf_dac_if_0/inst/dac_data_*_reg*/D}]]
if {[llength $fp]} {
  puts [format "  %-48s %6s   (requirement %s ns)" "FIFO memory -> read register slack" \
        [get_property SLACK $fp] [get_property REQUIREMENT $fp]]
}

# Is the FIFO exception actually present in THIS design?
set ex ""
catch {set ex [report_exceptions -quiet -return_string -to \
          [get_pins -quiet -hier -filter {NAME =~ *bf_dac_if_0/inst/dac_data_q_reg?15?/D}]]}
row "FIFO set_max_delay -datapath_only present" \
    [expr {[regexp -nocase {max_delay|datapath} $ex] ? 1 : 0}] 1

set wns [get_property SLACK [get_timing_paths -setup]]
set whs [get_property SLACK [get_timing_paths -hold]]
puts [format "  %-48s %6s" "WNS (ns)" $wns]
puts [format "  %-48s %6s" "WHS (ns)" $whs]

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
