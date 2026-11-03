###############################################################################
## Copyright (C) 2017-2023, 2026 Analog Devices, Inc. All rights reserved.
## Short identifier: ADIBSD
##
## Redistribution and use in source and binary forms, with or without modification,
## are permitted provided that the following conditions are met:
##     - Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.
##     - Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in
##       the documentation and/or other materials provided with the
##       distribution.
##     - Neither the name of Analog Devices, Inc. nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without specific prior written permission.
##     - The use of this software may or may not infringe the patent rights
##       of one or more patent holders. This license does not release you
##       from the requirement that you obtain separate licenses from these
##       patent holders to use this software.
##     - Use of the software either in source or binary form, must be run
##       on or directly connected to an Analog Devices Inc. component.
##
## THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
## INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
## PARTICULAR PURPOSE ARE DISCLAIMED.
##
## IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
## RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
## BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
## STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
## THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

source $ad_hdl_dir/projects/common/zcu102/zcu102_system_bd.tcl

# RnP P2: PS-GTR lanes 0-3 -> PCIe Gen2 x4 Root Port. Frees DP (lane 1), USB3_0 (lane 2), SATA (lane 3).
# USB0 stays as USB 2.0 on MIO 52..63. Property names/values from ps8_gtr_props.txt (P2.0).
set_property -dict [list \
  CONFIG.PSU__DISPLAYPORT__PERIPHERAL__ENABLE {0} \
  CONFIG.PSU__DPAUX__PERIPHERAL__ENABLE {0} \
  CONFIG.PSU__SATA__PERIPHERAL__ENABLE {0} \
  CONFIG.PSU__USB3_0__PERIPHERAL__ENABLE {0} \
  CONFIG.PSU__PCIE__MAXIMUM_LINK_WIDTH {x4} \
  CONFIG.PSU__PCIE__LANE1__ENABLE {1} CONFIG.PSU__PCIE__LANE1__IO {GT Lane1} \
  CONFIG.PSU__PCIE__LANE2__ENABLE {1} CONFIG.PSU__PCIE__LANE2__IO {GT Lane2} \
  CONFIG.PSU__PCIE__LANE3__ENABLE {1} CONFIG.PSU__PCIE__LANE3__IO {GT Lane3} \
] [get_bd_cells sys_ps8]

source ../common/fmcomms5_bd.tcl
source $ad_hdl_dir/projects/scripts/adi_pd.tcl

#system ID
ad_ip_parameter axi_sysid_0 CONFIG.ROM_ADDR_BITS 9
ad_ip_parameter rom_sys_0 CONFIG.PATH_TO_FILE "$mem_init_sys_file_path/mem_init_sys.txt"
ad_ip_parameter rom_sys_0 CONFIG.ROM_ADDR_BITS 9

sysid_gen_sys_init_file

ad_ip_parameter axi_ad9361_0 CONFIG.ADC_INIT_DELAY 8
ad_ip_parameter axi_ad9361_0 CONFIG.DELAY_REFCLK_FREQUENCY 500
ad_ip_parameter axi_ad9361_1 CONFIG.ADC_INIT_DELAY 8
ad_ip_parameter axi_ad9361_1 CONFIG.DELAY_REFCLK_FREQUENCY 500

ad_ip_parameter util_ad9361_divclk CONFIG.SIM_DEVICE ULTRASCALE
