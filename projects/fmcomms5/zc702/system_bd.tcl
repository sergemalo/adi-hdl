###############################################################################
## Copyright (C) 2014-2023 Analog Devices, Inc. All rights reserved.
### SPDX short identifier: ADIBSD
###############################################################################

source $ad_hdl_dir/projects/common/zc702/zc702_system_bd.tcl
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
ad_cpu_interconnect 0x79070000 axi_fir_ctrl
ad_connect axi_fir_ctrl/coeff_flat fir_i0_0/coeff_flat
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
ad_connect axi_fir_ctrl/coeff_frac fir_i0_0/coeff_frac

ad_ip_parameter axi_ad9361_0 CONFIG.ADC_INIT_DELAY 24
ad_ip_parameter axi_ad9361_1 CONFIG.ADC_INIT_DELAY 24
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_DEST 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_SRC 1
