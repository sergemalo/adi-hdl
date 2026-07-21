###############################################################################
## Copyright (C) Serge Malo 2026. All rights reserved.
###############################################################################
set fir_num_coeff 21
set fir_num_chan 8

#source $ad_hdl_dir/projects/common/zc702/zc702_system_bd.tcl
source ../common/zc702_system_bd.tcl
source $ad_hdl_dir/projects/scripts/adi_pd.tcl

#system ID
ad_ip_parameter axi_sysid_0 CONFIG.ROM_ADDR_BITS 9
ad_ip_parameter rom_sys_0 CONFIG.PATH_TO_FILE "$mem_init_sys_file_path/mem_init_sys.txt"
ad_ip_parameter rom_sys_0 CONFIG.ROM_ADDR_BITS 9

sysid_gen_sys_init_file

ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK2_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ 150.0

ad_connect sys_dma_clk sys_ps7/FCLK_CLK2
set sys_dma_clk [get_bd_nets sys_dma_clk]

# Custom RTL must be in the project + compile-ordered before the BD references it
# IQ_OVERRIDE
add_files -norecurse -fileset sources_1 iq_override.v
# FIR
add_files -norecurse  -fileset sources_1 [list \
  "$ad_hdl_dir/library/common/up_axi.v" \
  "axi_fir_ctrl.v" \
  "fir_i0.v" ]
update_compile_order -fileset sources_1
source ../common/fmcomms5_bd.tcl

## FIR CTRL
create_bd_cell -type module -reference axi_fir_ctrl axi_fir_ctrl
set_property -dict [list \
  CONFIG.NUM_COEFF    $fir_num_coeff \
  CONFIG.NUM_CHANNELS $fir_num_chan] [get_bd_cells axi_fir_ctrl]
ad_cpu_interconnect 0x79070000 axi_fir_ctrl
ad_connect axi_fir_ctrl/active_sel fir_i0_0/active_sel
ad_connect axi_fir_ctrl/coeff_frac fir_i0_0/coeff_frac
#ad_connect axi_fir_ctrl/coeff_flat0 fir_i0_0/coeff_flat0
#ad_connect axi_fir_ctrl/coeff_flat1 fir_i0_0/coeff_flat1

# Rung 2: drive the single golden FIR from CHANNEL 0's slice of the wide coeff
# buses (channel 0 = the low fir_num_coeff*COEFF_WIDTH bits of each bank).
# xlslice is transitional: at Rung 3 the fir_bank wrapper slices all eight lanes
# in HDL and both these cells AND this single-FIR golden datapath are removed --
# nothing here survives into the 8-lane production bitstream.
set fir_coeff_w 18
set fir_slice_w [expr {$fir_num_coeff * $fir_coeff_w}]
set fir_bus_w   [expr {$fir_num_chan  * $fir_slice_w}]
foreach {bank slice} {0 fir_ch0_slice0 1 fir_ch0_slice1} {
  create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 $slice
  set_property -dict [list \
    CONFIG.DIN_WIDTH  $fir_bus_w \
    CONFIG.DIN_FROM   [expr {$fir_slice_w - 1}] \
    CONFIG.DIN_TO     0 \
    CONFIG.DOUT_WIDTH $fir_slice_w] [get_bd_cells $slice]
  ad_connect axi_fir_ctrl/coeff_flat$bank $slice/Din
  ad_connect $slice/Dout fir_i0_0/coeff_flat$bank
}

## FIR CTRL

# --- Runtime I/Q override control (AXI GPIO) ---
ad_ip_instance axi_gpio axi_iq_ctrl
puts "GPIO pins: [get_bd_pins -of_objects [get_bd_cells axi_iq_ctrl] -filter {DIR == O}]"
ad_ip_parameter axi_iq_ctrl CONFIG.C_IS_DUAL       1
ad_ip_parameter axi_iq_ctrl CONFIG.C_ALL_OUTPUTS   1
ad_ip_parameter axi_iq_ctrl CONFIG.C_ALL_OUTPUTS_2 1
ad_ip_parameter axi_iq_ctrl CONFIG.C_GPIO_WIDTH    32
ad_ip_parameter axi_iq_ctrl CONFIG.C_GPIO2_WIDTH   1

ad_cpu_interconnect 0x79060000 axi_iq_ctrl

ad_connect axi_iq_ctrl/gpio_io_o  iq_override_0/ctrl_iq
ad_connect axi_iq_ctrl/gpio2_io_o iq_override_0/override_en
ad_connect util_ad9361_divclk/clk_out iq_override_0/clk

ad_ip_parameter axi_ad9361_0 CONFIG.ADC_INIT_DELAY 24
ad_ip_parameter axi_ad9361_1 CONFIG.ADC_INIT_DELAY 24
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_DEST 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_SRC 1

# Receive-only CRPA: TX ports are sample-fed, so drop the TX tone
# generators and TX I/Q correctors on both AD9361s.
ad_ip_parameter axi_ad9361_0 CONFIG.DAC_DDS_DISABLE          1
ad_ip_parameter axi_ad9361_1 CONFIG.DAC_DDS_DISABLE          1
ad_ip_parameter axi_ad9361_0 CONFIG.DAC_IQCORRECTION_DISABLE 1
ad_ip_parameter axi_ad9361_1 CONFIG.DAC_IQCORRECTION_DISABLE 1