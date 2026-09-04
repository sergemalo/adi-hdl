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
# chan_delay.v must precede fir_bank.v: fir_bank.v's generate block
# instantiates chan_delay for lanes 0,1 (the channel-0 DSP reclaim -- rx0 is
# the delay/phase reference and its FIR is a permanent identity filter, so a
# plain delay line replaces the 21-tap multiply-accumulate for those two
# lanes only; see fir_bank.v's header for the full rationale).
add_files -norecurse  -fileset sources_1 [list \
  "$ad_hdl_dir/library/common/up_axi.v" \
  "axi_fir_ctrl.v" \
  "fir_i0.v" \
  "chan_delay.v" \
  "fir_bank.v" \
  "axi_iq_ctrl.v"]

# Phase rotator + bank + AXI control
add_files -norecurse -fileset sources_1 [list \
  "phase_rot.v" \
  "phase_bank.v" \
  "axi_phase_ctrl.v"]

# Covariance-matrix accumulator + AXI control (MVDR part 1).
# up_axi.v is NOT re-added here -- the FIR file group above already pulls it
# into sources_1, and axi_covar_ctrl.v instantiates that same shim.
add_files -norecurse -fileset sources_1 [list \
  "covar_bank.v" \
  "axi_covar_ctrl.v"]
update_compile_order -fileset sources_1

source ../common/fmcomms5_bd.tcl

## FIR CTRL
create_bd_cell -type module -reference axi_fir_ctrl axi_fir_ctrl
set_property -dict [list \
  CONFIG.NUM_COEFF    $fir_num_coeff \
  CONFIG.NUM_CHANNELS $fir_num_chan] [get_bd_cells axi_fir_ctrl]
ad_cpu_interconnect 0x79070000 axi_fir_ctrl
ad_connect axi_fir_ctrl/coeff_flat0 fir_bank_0/coeff_flat0
ad_connect axi_fir_ctrl/coeff_flat1 fir_bank_0/coeff_flat1
ad_connect axi_fir_ctrl/active_sel  fir_bank_0/active_sel
ad_connect axi_fir_ctrl/coeff_frac  fir_bank_0/coeff_frac
## FIR CTRL

## PHASE CTRL
create_bd_cell -type module -reference axi_phase_ctrl axi_phase_ctrl
ad_cpu_interconnect 0x79080000 axi_phase_ctrl
ad_connect axi_phase_ctrl/a_bank0      phase_bank_0/a_bank0
ad_connect axi_phase_ctrl/b_bank0      phase_bank_0/b_bank0
ad_connect axi_phase_ctrl/a_bank1      phase_bank_0/a_bank1
ad_connect axi_phase_ctrl/b_bank1      phase_bank_0/b_bank1
ad_connect axi_phase_ctrl/active_sel   phase_bank_0/active_sel
ad_connect axi_phase_ctrl/coeff_frac_o phase_bank_0/coeff_frac
## PHASE CTRL

## COVAR CTRL
# 0x79090000 = next slot after PHG4 (0x79080000), continuing the
# IQC8 / FIR8 / PHG4 / COV4 base-address sequence. Must stay in sync with
# COVAR_BASE in covar_regmap.py.
create_bd_cell -type module -reference axi_covar_ctrl axi_covar_ctrl
ad_cpu_interconnect 0x79090000 axi_covar_ctrl
ad_connect axi_covar_ctrl/enable_raw                 covar_bank_0/enable_raw
ad_connect axi_covar_ctrl/rd_bank_sel                covar_bank_0/rd_bank_sel
ad_connect axi_covar_ctrl/rd_word_sel                covar_bank_0/rd_word_sel
ad_connect axi_covar_ctrl/rd_data                    covar_bank_0/rd_data
# Status crossing back to the AXI domain: block_done_toggle is a single bit
# (2-FF synchronized inside axi_covar_ctrl); latest_complete_bank/block_seq
# are captured there only on a detected toggle edge, by which point they
# have been stable in the sample-clock domain for many up_clk cycles.
ad_connect axi_covar_ctrl/block_done_toggle          covar_bank_0/block_done_toggle
ad_connect axi_covar_ctrl/latest_complete_bank_async covar_bank_0/latest_complete_bank
ad_connect axi_covar_ctrl/block_seq_async            covar_bank_0/block_seq
## COVAR CTRL


# --- Runtime sample injector control (custom AXI-Lite slave) ---
create_bd_cell -type module -reference axi_iq_ctrl axi_iq_ctrl
ad_cpu_interconnect 0x79060000 axi_iq_ctrl

ad_connect axi_iq_ctrl/override_en  iq_override_0/override_en
ad_connect axi_iq_ctrl/pattern_mode iq_override_0/pattern_mode
ad_connect axi_iq_ctrl/pattern_hold iq_override_0/pattern_hold
ad_connect axi_iq_ctrl/dc_flat      iq_override_0/dc_flat
ad_connect axi_iq_ctrl/seed_flat    iq_override_0/seed_flat
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
