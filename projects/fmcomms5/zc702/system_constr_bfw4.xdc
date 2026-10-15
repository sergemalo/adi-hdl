
# BFW4 (MVDR Part 4): first stage of every BFW4 synchroniser, in axi_bf_ctrl,
# bf_dac_if and bf_capture (registers named bfx_m1_*). Each carries a single
# bit or a gray-coded FIFO pointer, so only the cell-to-cell delay matters.
# AXI <-> divclk is already asynchronous via the clock group above; this also
# covers AXI <-> l_clk (tx_src, FIFO status, core dac_enable) and the FIFO
# pointers between divclk and l_clk. Expect 38 cells (check_bf_build.tcl).
set_false_path -to [get_cells -hier -filter {NAME =~ *bfx_m1_* && IS_SEQUENTIAL}]

# BFW4 FIFO memory -> read register rd_word (bf_dac_if rev 2). The FIFO is
# distributed RAM written in divclk and read combinationally into rd_word,
# clocked by l_clk; rd_word is the ONLY l_clk endpoint of the memory.
# divclk reaches its loads ~4.9 ns AFTER l_clk (BUFR + BUFGCTRL vs. BUFG
# insertion delay), so against the 5 ns rx_0_clk constraint this path cannot
# be met as a synchronous path (first build: WNS -2.217 ns, skew -4.882 ns).
# It does not have to be: a location is read only after its write pointer has
# crossed the 2-FF gray synchroniser (>= 2 l_clk after the write), and is not
# rewritten until the read pointer has crossed back. Only the wire delay must
# be bounded, so: datapath-only, one rx_0_clk period. Same treatment as any
# asynchronous FIFO built on distributed RAM.
set_max_delay -datapath_only 5.000 \
  -from [get_pins -hier -filter {NAME =~ *bf_dac_if_0/inst/mem_reg*/CLK}] \
  -to   [get_pins -hier -filter {NAME =~ *bf_dac_if_0/inst/rd_word_reg*/D}]
