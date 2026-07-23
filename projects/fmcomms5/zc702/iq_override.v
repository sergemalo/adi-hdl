`timescale 1ns/1ps

// iq_override -- RX-path sample injector, per-lane.
//
// Sits between the ADC FIFO and the FIR bank. In passthrough it is transparent;
// when enabled it replaces each lane's sample with a KNOWN value, so the
// downstream FIR can be checked against a model with no RF involved.
//
// Widened from the original common-I/Q version (one 32-bit GPIO carried a
// single I/Q pair shared by all lanes): each lane now has its own value, which
// is what makes INPUT-side routing observable -- with a common value every lane
// looks alike, so a swapped RX->FIR input is invisible. Controls now come from
// axi_iq_ctrl (a real AXI-Lite slave) instead of an axi_gpio, because 8 lanes
// need 128 bits of values plus 128 bits of seeds.
//
// Two modes (pattern_mode):
//   0 = DC      : lane k emits dc[k], constant.
//   1 = PATTERN : lane k emits its own 16-bit Galois LFSR sequence, advancing
//                 one step per sample beat. Deterministic, so Python can
//                 regenerate the exact input stream and check the full
//                 convolution on all 8 lanes at once.
//
// LFSR: 16-bit Galois, polynomial mask 0xB400 (x^16+x^14+x^13+x^11+1), maximal
// length 65535. A zero seed would lock up, so a zero programs as 1 instead.
// pattern_hold (a level) holds every lane at its seed; releasing it starts all
// lanes from a known state on the same beat.
//
// Control inputs arrive from the AXI clock domain (clk_fpga_0) and are
// registered into the datapath domain (clk = util_ad9361_divclk/clk_out =
// clk_div_sel_1_s), so the crossing is a clean flop-to-flop CDC. The DATA path
// (din -> mux -> dout) stays combinational, so sample timing and the
// enable/fifo_wr_en handshake are unchanged.

module iq_override #(
  parameter DATA_WIDTH = 16,
  parameter NUM_LANES  = 8
) (
  input  wire                  clk,          // util_ad9361_divclk/clk_out
  input  wire                  valid,        // sample beat (adc_fifo dout_valid)

  input  wire [DATA_WIDTH-1:0] din_0, din_1, din_2, din_3,
  input  wire [DATA_WIDTH-1:0] din_4, din_5, din_6, din_7,

  // from axi_iq_ctrl (async / quasi-static, s_axi_aclk domain)
  input  wire                              override_en,   // 1=inject, 0=passthrough
  input  wire                              pattern_mode,  // 0=DC, 1=LFSR
  input  wire                              pattern_hold,  // 1=hold LFSRs at seed
  input  wire [(NUM_LANES*DATA_WIDTH)-1:0] dc_flat,
  input  wire [(NUM_LANES*DATA_WIDTH)-1:0] seed_flat,

  output wire [DATA_WIDTH-1:0] dout_0, dout_1, dout_2, dout_3,
  output wire [DATA_WIDTH-1:0] dout_4, dout_5, dout_6, dout_7
);

  localparam [15:0] LFSR_MASK = 16'hB400;

  // 2-FF synchronizers on the control levels.
  (* ASYNC_REG = "TRUE" *) reg en_meta   = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg en_sync   = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg mode_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg mode_sync = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg hold_meta = 1'b1;
  (* ASYNC_REG = "TRUE" *) reg hold_sync = 1'b1;

  // Quasi-static value/seed buses: one register stage (same treatment the
  // 32-bit ctrl_iq bus had before).
  reg [(NUM_LANES*DATA_WIDTH)-1:0] dc_r   = {(NUM_LANES*DATA_WIDTH){1'b0}};
  reg [(NUM_LANES*DATA_WIDTH)-1:0] seed_r = {(NUM_LANES*DATA_WIDTH){1'b0}};

  always @(posedge clk) begin
    en_meta   <= override_en;
    en_sync   <= en_meta;
    mode_meta <= pattern_mode;
    mode_sync <= mode_meta;
    hold_meta <= pattern_hold;
    hold_sync <= hold_meta;
    dc_r      <= dc_flat;
    seed_r    <= seed_flat;
  end

  // ---------------------------------------------------------------------
  // Per-lane LFSR. Advances once per SAMPLE (valid), so the sequence is in
  // lockstep with the samples the FIR consumes. Held at seed while
  // pattern_hold is asserted.
  // ---------------------------------------------------------------------
  reg  [DATA_WIDTH-1:0] lfsr [0:NUM_LANES-1];

  wire [DATA_WIDTH-1:0] dc_v   [0:NUM_LANES-1];
  wire [DATA_WIDTH-1:0] seed_v [0:NUM_LANES-1];
  wire [DATA_WIDTH-1:0] inj    [0:NUM_LANES-1];

  genvar g;
  generate
    for (g = 0; g < NUM_LANES; g = g + 1) begin: g_lane
      assign dc_v[g] = dc_r[g*DATA_WIDTH +: DATA_WIDTH];

      // a zero seed would lock the LFSR at zero forever -> substitute 1
      wire [DATA_WIDTH-1:0] seed_raw = seed_r[g*DATA_WIDTH +: DATA_WIDTH];
      assign seed_v[g] = (seed_raw == {DATA_WIDTH{1'b0}}) ? {{(DATA_WIDTH-1){1'b0}}, 1'b1}
                                                          : seed_raw;

      initial lfsr[g] = {{(DATA_WIDTH-1){1'b0}}, 1'b1};

      always @(posedge clk) begin
        if (hold_sync == 1'b1) begin
          lfsr[g] <= seed_v[g];
        end else if (valid == 1'b1) begin
          lfsr[g] <= lfsr[g][0] ? ((lfsr[g] >> 1) ^ LFSR_MASK)
                                : (lfsr[g] >> 1);
        end
      end

      // injected value for this lane: pattern or DC
      assign inj[g] = mode_sync ? lfsr[g] : dc_v[g];
    end
  endgenerate

  assign dout_0 = en_sync ? inj[0] : din_0;  // ch0 I
  assign dout_1 = en_sync ? inj[1] : din_1;  // ch0 Q
  assign dout_2 = en_sync ? inj[2] : din_2;  // ch1 I
  assign dout_3 = en_sync ? inj[3] : din_3;  // ch1 Q
  assign dout_4 = en_sync ? inj[4] : din_4;  // ch2 I
  assign dout_5 = en_sync ? inj[5] : din_5;  // ch2 Q
  assign dout_6 = en_sync ? inj[6] : din_6;  // ch3 I
  assign dout_7 = en_sync ? inj[7] : din_7;  // ch3 Q

endmodule
