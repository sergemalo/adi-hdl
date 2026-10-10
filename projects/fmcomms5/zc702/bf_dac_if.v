`timescale 1ns/1ps
// ============================================================================
// bf_dac_if — BFW4 output to the AD9361 TX2 DMA input (chip A, TX2A_A)
//
// bf_bank produces one DAC word pair per sample in the divided sample clock
// (divclk). axi_ad9361_0 asks for one per dac_valid_i1 pulse in l_clk. Both
// rates derive from the same l_clk, so they are equal by construction; a
// small FIFO absorbs the unknown, fixed phase between them and the shift the
// driver's start_sync causes (<= 3 l_clk).
//
//   write side (clk_s = divclk): push {q, i} on every valid sample
//   read side  (clk_d = l_clk):  pop on dac_valid once PRIMED
//
// PRIMED: the read side does not pop until the FIFO holds PRIME words, so it
// starts half full. If it ever finds the FIFO empty when asked, it outputs 0,
// counts an UNDERFLOW and re-primes. If the write side finds it full, the
// sample is dropped and an OVERFLOW is counted. In normal operation neither
// can happen (equal rates); both are hard invariants for argus.
//
// The output register follows util_rfifo: it updates on the dac_valid pulse
// and holds between pulses. TX_SRC (async from AXI) selects BF (1) or zero
// (0) at that register; the FIFO keeps running either way, so its level and
// flags do not depend on TX_SRC.
//
// Event outputs are TOGGLES (flip once per event) for axi_bf_ctrl to count:
// sat (per clipped output sample), ovf, unf. At most one event per sample
// period, so a 100 MHz AXI clock sees every flip.
//
// CDC: gray-coded pointers through 2-FF synchronisers; data through a LUT RAM
// whose read address is in clk_d. clk_s and clk_d are related (BUFR divide of
// l_clk), so Vivado times these paths as synchronous. Synchroniser first
// stages are named bfx_m1_* for the XDC.
// No reset port: power-up state via `initial`.
// ============================================================================
module bf_dac_if #(
  parameter integer AW    = 4,      // FIFO depth 2^AW = 16
  parameter integer PRIME = 8
)(
  // write side, divclk
  input  wire        clk_s,
  input  wire        valid_s,
  input  wire [15:0] din_i,
  input  wire [15:0] din_q,
  input  wire        sat_s,          // bf_bank sat_out (already gated by valid)
  output reg         sat_toggle = 1'b0,
  output reg         ovf_toggle = 1'b0,

  // read side, l_clk
  input  wire        clk_d,
  input  wire        dac_valid,      // axi_ad9361_0/dac_valid_i1
  input  wire        tx_src,         // async (AXI): 1 = BF, 0 = zero
  output reg  [15:0] dac_data_i = 16'd0,
  output reg  [15:0] dac_data_q = 16'd0,
  output reg         unf_toggle = 1'b0,
  output reg         primed     = 1'b0
);
  localparam integer DEPTH = 1 << AW;

  function [AW:0] bin2gray; input [AW:0] b; bin2gray = b ^ (b >> 1); endfunction
  function [AW:0] gray2bin;
    input [AW:0] g; integer k;
    begin
      gray2bin[AW] = g[AW];
      for (k = AW - 1; k >= 0; k = k - 1) gray2bin[k] = gray2bin[k+1] ^ g[k];
    end
  endfunction

  reg [31:0] mem [0:DEPTH-1];
  integer m;
  initial for (m = 0; m < DEPTH; m = m + 1) mem[m] = 32'd0;

  // ------------------------------------------------------------- write side
  reg [AW:0] wbin = 0, wgray = 0;
  reg [AW:0] rgray = 0;                                   // read side, below
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m1_rgray = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m2_rgray = 0;
  wire [AW:0] rbin_s = gray2bin(bfx_m2_rgray);
  wire        full   = ((wbin - rbin_s) == DEPTH);

  always @(posedge clk_s) begin
    bfx_m1_rgray <= rgray;
    bfx_m2_rgray <= bfx_m1_rgray;
    if (valid_s) begin
      if (!full) begin
`ifdef MUT_FIFO_IQ
        mem[wbin[AW-1:0]] <= {din_i, din_q};     // fault: I/Q swapped in the FIFO
`else
        mem[wbin[AW-1:0]] <= {din_q, din_i};
`endif
        wbin  <= wbin + 1'b1;
        wgray <= bin2gray(wbin + 1'b1);
      end else
        ovf_toggle <= ~ovf_toggle;
      if (sat_s) sat_toggle <= ~sat_toggle;
    end
  end

  // -------------------------------------------------------------- read side
  reg [AW:0] rbin = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m1_wgray = 0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m2_wgray = 0;
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_src = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        bfx_m2_src = 1'b0;
  wire [AW:0] wbin_d = gray2bin(bfx_m2_wgray);
  wire [AW:0] level  = wbin_d - rbin;
  wire [31:0] rword  = mem[rbin[AW-1:0]];
`ifdef MUT_NO_PRIME
  wire        go     = 1'b1;                 // fault: pop from the first request
`else
  wire        go     = primed;
`endif
`ifdef MUT_SRC_IGNORED
  wire        src    = 1'b1;                 // fault: TX_SRC has no effect
`else
  wire        src    = bfx_m2_src;
`endif

  always @(posedge clk_d) begin
    bfx_m1_wgray <= wgray;
    bfx_m2_wgray <= bfx_m1_wgray;
    bfx_m1_src   <= tx_src;
    bfx_m2_src   <= bfx_m1_src;

    if (!primed && level >= PRIME) primed <= 1'b1;

    if (dac_valid) begin
      if (go && level != 0) begin
        dac_data_i <= src ? rword[15:0]  : 16'd0;
        dac_data_q <= src ? rword[31:16] : 16'd0;
        rbin  <= rbin + 1'b1;
        rgray <= bin2gray(rbin + 1'b1);
      end else begin
        dac_data_i <= 16'd0;
        dac_data_q <= 16'd0;
        if (go) begin                         // asked while empty
          unf_toggle <= ~unf_toggle;
          primed     <= 1'b0;
        end
      end
    end
  end
endmodule
