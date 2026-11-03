# dump_ps8.tcl - print the PS8 settings related to the PS-GTR lanes (read-only)
open_project fmcomms5_zcu102.xpr
open_bd_design [get_files system.bd]
set ps [get_bd_cells sys_ps8]
set fh [open ps8_gtr_props.txt w]
foreach p [lsort [list_property $ps]] {
  if {[regexp {PCIE|SATA|DISPLAYPORT|DPAUX|USB3|USB0__|GT_LANE|LANE_SEL|SERDES|GTR} $p]} {
    puts $fh "$p = [get_property $p $ps]"
  }
}
close $fh
close_project