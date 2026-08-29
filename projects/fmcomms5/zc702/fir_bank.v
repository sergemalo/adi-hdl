// fir_bank.v
//
// 8-lane FIR bank for the CRPA RX path. Instantiates one proven fir_i0 per
// lane and slices the wide coefficient buses from axi_fir_ctrl in HDL (channel
// c = the c-th NUM_COEFF*COEFF_WIDTH slice), so the block design connects whole
// buses -- no xlslice/xlconcat cells. This replaces the single golden fir_i0_0
// (and the Rung-2 xlslice scaffolding) as the production datapath.
//
//   dout_fir_c = FIR_c(din_c)     with channel c's own coefficients
//
// coeff_frac and active_sel are GLOBAL (one value fans out to all lanes), so a
// single CTRL write commits every lane's new coefficient set in the same sample
// beat -- the coherent swap the array needs. dout_ref is not used here (the
// 8-lane capture carries eight FIR outputs, no reference lane); tap-order
// correctness of the slice was proven per-lane in Rung 2.
//
// Lane map (matches iq_override dout_$i and the ADC pack fifo_wr_data_$i):
//   0,1 = AD9361_0 I0,Q0   2,3 = AD9361_0 I1,Q1
//   4,5 = AD9361_1 I0,Q0   6,7 = AD9361_1 I1,Q1
//
// CHANNEL-0 DSP RECLAIM (lanes 0,1 = rx0 I,Q):
//   rx0 is the delay/phase calibration reference channel. Its coefficients
//   are permanently a unit center tap (identity filter) -- the calibration
//   toolchain (fir_delay_coeffs / measure_time_offsets) never assigns rx0 a
//   nonzero residual delay, by construction (channel 1 in the math note's
//   1-indexed convention = channel 0 here is always the reference, delay=0).
//   So lanes 0,1 are wired to chan_delay instead of fir_i0: same din->dout
//   latency (CENTER_TAP + LATENCY sample beats, bit-exact), zero DSPs instead
//   of NUM_COEFF each. Reclaims ~2*NUM_COEFF DSPs (~40 for NUM_COEFF=21).
//
//   These two lanes no longer read coeff_flat0/1 or sel_sync at all -- their
//   slices of the coefficient bus are simply unused wires now. The
//   axi_fir_ctrl register map is left byte-identical (channels 0,1's tap
//   registers are still writable/readable there) for host-software address
//   compatibility; writes to those two channels' taps are hardware no-ops.
//   See axi_fir_ctrl.v's header for the corresponding register-map note.

`timescale 1ns/100ps

module fir_bank #(
  parameter NUM_COEFF   = 3,
  parameter DATA_WIDTH  = 16,
  parameter COEFF_WIDTH = 18,
  parameter FRAC_WIDTH  = 5
) (
  input                                       clk,

  // per-lane sample-valid (each from its ADC FIFO dout_valid_$i)
  input                                       valid_0,
  input                                       valid_1,
  input                                       valid_2,
  input                                       valid_3,
  input                                       valid_4,
  input                                       valid_5,
  input                                       valid_6,
  input                                       valid_7,

  // per-lane real input samples (from iq_override dout_$i)
  input      signed [DATA_WIDTH-1:0]          din_0,
  input      signed [DATA_WIDTH-1:0]          din_1,
  input      signed [DATA_WIDTH-1:0]          din_2,
  input      signed [DATA_WIDTH-1:0]          din_3,
  input      signed [DATA_WIDTH-1:0]          din_4,
  input      signed [DATA_WIDTH-1:0]          din_5,
  input      signed [DATA_WIDTH-1:0]          din_6,
  input      signed [DATA_WIDTH-1:0]          din_7,

  // wide coefficient banks from axi_fir_ctrl (8 channels, s_axi_aclk domain).
  // literal 8: this bank is fixed at the FMCOMMS5 lane count (4 RX x I/Q).
  // NOTE: the slices for lanes 0,1 (channel 0) are received but unused --
  // see the CHANNEL-0 DSP RECLAIM note above.
  input      [(8*NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat0,
  input      [(8*NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat1,
  input                                       active_sel,   // global: 0=bank0, 1=bank1
  input      [FRAC_WIDTH-1:0]                 coeff_frac,   // global Q-format shift

  // per-lane FIR outputs (to the ADC pack fifo_wr_data_$i)
  output     signed [DATA_WIDTH-1:0]          dout_fir_0,
  output     signed [DATA_WIDTH-1:0]          dout_fir_1,
  output     signed [DATA_WIDTH-1:0]          dout_fir_2,
  output     signed [DATA_WIDTH-1:0]          dout_fir_3,
  output     signed [DATA_WIDTH-1:0]          dout_fir_4,
  output     signed [DATA_WIDTH-1:0]          dout_fir_5,
  output     signed [DATA_WIDTH-1:0]          dout_fir_6,
  output     signed [DATA_WIDTH-1:0]          dout_fir_7
);

  localparam NUM_CHANNELS = 8;                       // FMCOMMS5 lanes (fixed)
  localparam SLICE        = NUM_COEFF * COEFF_WIDTH; // bits per channel per bank

  // Fan the per-lane ports into indexable arrays so the generate loop can carry
  // one clean instantiation for all lanes.
  wire signed [DATA_WIDTH-1:0] din  [0:NUM_CHANNELS-1];
  wire signed [DATA_WIDTH-1:0] dout [0:NUM_CHANNELS-1];
  wire                          vld [0:NUM_CHANNELS-1];

  assign din[0] = din_0; assign din[1] = din_1;
  assign din[2] = din_2; assign din[3] = din_3;
  assign din[4] = din_4; assign din[5] = din_5;
  assign din[6] = din_6; assign din[7] = din_7;

  assign vld[0] = valid_0; assign vld[1] = valid_1;
  assign vld[2] = valid_2; assign vld[3] = valid_3;
  assign vld[4] = valid_4; assign vld[5] = valid_5;
  assign vld[6] = valid_6; assign vld[7] = valid_7;

  assign dout_fir_0 = dout[0]; assign dout_fir_1 = dout[1];
  assign dout_fir_2 = dout[2]; assign dout_fir_3 = dout[3];
  assign dout_fir_4 = dout[4]; assign dout_fir_5 = dout[5];
  assign dout_fir_6 = dout[6]; assign dout_fir_7 = dout[7];

  // ---------------------------------------------------------------------
  // Shared active_sel synchronizer.
  //
  // active_sel is written by the CPU in the s_axi domain and is genuinely
  // asynchronous to the FIR clock. Resolving it ONCE here (a single 2-FF
  // synchronizer) and driving the settled sel_sync to all NUM_CHANNELS lanes
  // means every lane sees the identical select on the identical edge -- so a
  // coefficient-bank swap commits coherently across all 8 lanes in the same
  // sample beat, with no cross-lane tear even if the write lands in the
  // metastability window. (Per-lane synchronizers would each resolve the
  // crossing independently, allowing a rare 1-beat skew between lanes.)
  //
  // Note: lanes 0,1 (chan_delay) do not consume sel_sync at all -- it is
  // still generated here unconditionally since lanes 2-7 (fir_i0) need it,
  // and a partial synchronizer would be needless complexity for no DSP gain.
  //
  // MUST free-run (no valid enable): a synchronizer clocked by an intermittent
  // enable does not resolve metastability. The CDC (active_sel -> sel_meta) is
  // covered by the s_axi/FIR set_clock_groups -asynchronous, same as before,
  // now with a single crossing instead of eight.
  // ---------------------------------------------------------------------

  (* ASYNC_REG = "TRUE" *) reg sel_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg sel_sync = 1'b0;

  always @(posedge clk) begin
    sel_meta <= active_sel;
    sel_sync <= sel_meta;
  end

  genvar c;
  generate
    for (c = 0; c < NUM_CHANNELS; c = c + 1) begin: g_lane
      if (c < 2) begin: g_ch0_delay
        // Lanes 0,1 = AD9361_0 I0,Q0 = rx0 = the delay/phase reference
        // channel. Its FIR coefficients are permanently a unit center tap
        // (c_reg[(NUM_COEFF-1)/2] = 1.0 in Q2.16, all others 0), so the
        // 21-tap multiply-accumulate is pure overhead: chan_delay reproduces
        // the IDENTICAL din->dout_fir latency with a shift register,
        // reclaiming NUM_COEFF DSPs/lane. No coefficient bus, no bank
        // select, no axi_fir_ctrl dependency for this lane.
        chan_delay #(
          .NUM_COEFF  (NUM_COEFF),
          .DATA_WIDTH (DATA_WIDTH)
        ) u_delay (
          .clk      (clk),
          .valid    (vld[c]),
          .din      (din[c]),
          .dout_fir (dout[c]),
          .dout_ref (),                 // unused in the 8-lane production path
          .sat      ()                  // always 0 for a pure delay
        );
      end else begin: g_fir
        fir_i0 #(
          .NUM_COEFF   (NUM_COEFF),
          .DATA_WIDTH  (DATA_WIDTH),
          .COEFF_WIDTH (COEFF_WIDTH),
          .FRAC_WIDTH  (FRAC_WIDTH)
        ) u_fir (
          .clk         (clk),
          .valid       (vld[c]),
          .din         (din[c]),
          .coeff_flat0 (coeff_flat0[c*SLICE +: SLICE]),
          .coeff_flat1 (coeff_flat1[c*SLICE +: SLICE]),
          .sel_sync    (sel_sync),       // shared, already synchronized -> coherent swap
          .coeff_frac  (coeff_frac),
          .dout_fir    (dout[c]),
          .dout_ref    (),                 // unused in the 8-lane production path
          .sat         ()                  // per-lane saturation flag (available)
        );
      end
    end
  endgenerate

endmodule
