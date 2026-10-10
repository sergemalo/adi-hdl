`timescale 1ns/1ps
// ============================================================================
// bf_bank — BFW4 beamformer datapath (MVDR Part 4)
//
// Specification: py-examples/bf_golden.py. This module must match it bit for
// bit; nothing here is a design decision that file does not already state.
//
//   acc_i = sum_k cr_k*xi_k + ci_k*xq_k        (= Re/Im of sum_k conj(c_k)*x_k:
//   acc_q = sum_k cr_k*xq_k - ci_k*xi_k         the conjugate is applied HERE)
//   y     = (acc + R) >>> S,  R = S ? 2^(S-1) : 0
//   y12   = clamp(y, -2048, 2047)
//   dout  = {y12, 4'b0}                          (core takes dac_data[15:4])
//
// Pure datapath: no AXI, no reset port (power-up state via `initial`).
// Lanes: din_{2k} = I, din_{2k+1} = Q of channel k (phase_bank convention).
//
// Structure: two systolic DSP48 cascades (I and Q), 8 stages each, 16
// multipliers. Stage s uses weight field s (s = 2k: cr_k, s = 2k+1: ci_k):
//     chain I, stage s:  + w_s * din_s
//     chain Q, stage s:  + w_s * din_{s^1}  (even s),  - w_s * din_{s^1}  (odd s)
// Stage s sees its input delayed by s clocks so the partial sum and the data of
// one sample meet. Coefficients are quasi-static per bank (as in fir_bank /
// phase_bank), but ONE sample spans 12 clocks of pipeline, so a flip that
// switched every stage on the same clock would tear the samples in flight.
// Instead the synchronised bank select is captured WITH each sample (its
// "tag") and travels down the pipeline beside it: every stage, the rounding
// constant and the final shift all use the tag of the sample they are
// processing. One synchroniser, one decision per sample.
//
// Host contract (from the travelling tag): a bank may be rewritten only once no
// sample tagged with it remains in flight: 13 clocks after the select changes
// (measured in simulation; 12 tears a sample). Budget 2 (sync) + LATENCY = 15
// clocks, under 0.5 us at 30.72 MHz. The host rewrites a bank once per
// covariance block, 1 ms apart, so the margin is > 2000x.
// ============================================================================
module bf_bank #(
  parameter integer DATA_W  = 16,
  parameter integer W_W     = 18,
  parameter integer SHIFT_W = 5,
  parameter integer ACC_W   = 37,   // proven sufficient in bf_golden
  parameter integer OUT_W   = 12
)(
  input  wire                  clk,
  input  wire                  valid_in,
  input  wire [DATA_W-1:0]     din_0, din_1, din_2, din_3,
  input  wire [DATA_W-1:0]     din_4, din_5, din_6, din_7,

  input  wire                  active_sel,      // AXI domain (async)
  // Field f at [f*W_W +: W_W]: f = 2k -> W{k}_RE, f = 2k+1 -> W{k}_IM
  input  wire [8*W_W-1:0]      w_bank0, w_bank1,
  input  wire [SHIFT_W-1:0]    shift_bank0, shift_bank1,

  output wire                  valid_out,
  output wire [15:0]           dout_i, dout_q,  // DAC words, bits [3:0] = 0
  output wire                  sat_out          // with valid_out: I or Q clipped
);

  localparam integer NST     = 8;
  localparam integer P_W     = 48;              // DSP48E1 P width
  localparam integer M_W     = DATA_W + W_W;    // 34
  // Pipeline, counted from the din_* edge: 1 input reg, then stage s has
  // A/B (s+1), M (s+2), P (s+3) relative to the input reg; P of stage 7 at +10,
  // shift at +11, saturate at +12.
  localparam integer SB_LEN  = 13;              // sideband entries 0..12
  localparam integer LATENCY = SB_LEN;          // din edge -> dout, in clocks

  // ---------------------------------------------------------------- select sync
  (* ASYNC_REG = "TRUE" *) reg sel_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg sel_sync = 1'b0;
  always @(posedge clk) begin
    sel_meta <= active_sel;
    sel_sync <= sel_meta;
  end

  // ------------------------------------------------ input register + delay lines
  wire [DATA_W-1:0] din [0:7];
  assign din[0] = din_0; assign din[1] = din_1; assign din[2] = din_2; assign din[3] = din_3;
  assign din[4] = din_4; assign din[5] = din_5; assign din[6] = din_6; assign din[7] = din_7;

  // dl[l][d]: lane l delayed d clocks after the input register (d = 0 IS it)
  reg signed [DATA_W-1:0] dl [0:7][0:NST-1];
  integer li, di;
  initial for (li = 0; li < 8; li = li + 1) for (di = 0; di < NST; di = di + 1) dl[li][di] = 0;

  always @(posedge clk) begin
`ifdef MUT_CH_SWAP
    // fault: channels 0 and 3 swapped at the input
    dl[0][0] <= din[6]; dl[1][0] <= din[7];
    dl[2][0] <= din[2]; dl[3][0] <= din[3];
    dl[4][0] <= din[4]; dl[5][0] <= din[5];
    dl[6][0] <= din[0]; dl[7][0] <= din[1];
`else
    for (li = 0; li < 8; li = li + 1) dl[li][0] <= din[li];
`endif
    for (li = 0; li < 8; li = li + 1)
      for (di = 1; di < NST; di = di + 1) dl[li][di] <= dl[li][di-1];
  end

  // sideband: {valid, tag} travelling with each sample, sb[d] at input-reg + d
  reg sb_v [0:SB_LEN-1];
  reg sb_t [0:SB_LEN-1];
  integer bi;
  initial for (bi = 0; bi < SB_LEN; bi = bi + 1) begin sb_v[bi] = 1'b0; sb_t[bi] = 1'b0; end
  always @(posedge clk) begin
    sb_v[0] <= valid_in;
    sb_t[0] <= sel_sync;
    for (bi = 1; bi < SB_LEN; bi = bi + 1) begin
      sb_v[bi] <= sb_v[bi-1];
      sb_t[bi] <= sb_t[bi-1];
    end
  end

  // which tag a given stage/point uses
`ifdef MUT_FLAT_SEL
  // fault: every point uses the CURRENT select, not the sample's own tag
  `define TAG(d) sel_sync
`else
  `define TAG(d) sb_t[d]
`endif

  // rounding constant for a shift value
  function signed [P_W-1:0] rnd_of;
    input [SHIFT_W-1:0] s;
    begin
`ifdef MUT_NO_RND
      rnd_of = 0;
`else
      rnd_of = (s == 0) ? {P_W{1'b0}} : ({{(P_W-1){1'b0}}, 1'b1} << (s - 1));
`endif
    end
  endfunction

  // ------------------------------------------------------- the two DSP chains
  reg signed [W_W-1:0]    b_r [0:NST-1];   // muxed weight (B register)
  reg signed [DATA_W-1:0] ai_r [0:NST-1];  // A register, chain I
  reg signed [DATA_W-1:0] aq_r [0:NST-1];  // A register, chain Q
  (* use_dsp = "yes" *) reg signed [M_W-1:0] mi_r [0:NST-1];
  (* use_dsp = "yes" *) reg signed [M_W-1:0] mq_r [0:NST-1];
  reg signed [P_W-1:0]    pi_r [0:NST-1];
  reg signed [P_W-1:0]    pq_r [0:NST-1];

  integer si;
  initial for (si = 0; si < NST; si = si + 1) begin
    b_r[si] = 0; ai_r[si] = 0; aq_r[si] = 0;
    mi_r[si] = 0; mq_r[si] = 0; pi_r[si] = 0; pq_r[si] = 0;
  end

  genvar s;
  generate for (s = 0; s < NST; s = s + 1) begin : g_st
    wire signed [W_W-1:0] w0 = w_bank0[s*W_W +: W_W];
    wire signed [W_W-1:0] w1 = w_bank1[s*W_W +: W_W];
    always @(posedge clk) begin
      // A/B: data delayed s, weight chosen by the tag of that same sample
      ai_r[s] <= dl[s][s];
      aq_r[s] <= dl[s ^ 1][s];
      b_r[s]  <= `TAG(s) ? w1 : w0;
      // M
      mi_r[s] <= ai_r[s] * b_r[s];
      mq_r[s] <= aq_r[s] * b_r[s];
    end
  end endgenerate

  // P: stage 0 starts from the rounding constant of its sample's bank (tag at
  // input-reg + 2, aligned with M of stage 0); stage s adds to stage s-1.
  wire signed [P_W-1:0] rnd0 = `TAG(2) ? rnd_of(shift_bank1) : rnd_of(shift_bank0);

  always @(posedge clk) begin
    pi_r[0] <= rnd0 + mi_r[0];
    pq_r[0] <= rnd0 + mq_r[0];
    for (si = 1; si < NST; si = si + 1) begin
      pi_r[si] <= pi_r[si-1] + mi_r[si];
`ifdef MUT_NO_CONJ
      // fault: sum c*x instead of conj(c)*x (sign of the ci terms flipped)
      if (si & 1) begin
        pi_r[si] <= pi_r[si-1] - mi_r[si];
        pq_r[si] <= pq_r[si-1] + mq_r[si];
      end else
        pq_r[si] <= pq_r[si-1] + mq_r[si];
`else
      if (si & 1) pq_r[si] <= pq_r[si-1] - mq_r[si];
      else        pq_r[si] <= pq_r[si-1] + mq_r[si];
`endif
    end
  end

  // --------------------------------------------- shift (+11), saturate (+12)
  reg signed [ACC_W-1:0] sh_i = 0, sh_q = 0;
  wire [SHIFT_W-1:0] s_now = `TAG(10) ? shift_bank1 : shift_bank0;

  always @(posedge clk) begin
    sh_i <= $signed(pi_r[NST-1][ACC_W-1:0]) >>> s_now;
    sh_q <= $signed(pq_r[NST-1][ACC_W-1:0]) >>> s_now;
  end

  localparam signed [OUT_W-1:0] Y_MAX = {1'b0, {(OUT_W-1){1'b1}}};
  localparam signed [OUT_W-1:0] Y_MIN = {1'b1, {(OUT_W-1){1'b0}}};

  function [OUT_W:0] sat12;                    // {flag, value}
    input signed [ACC_W-1:0] v;
    reg ovf;
    begin
      ovf = (v[ACC_W-1:OUT_W-1] != {(ACC_W-OUT_W+1){1'b0}}) &&
            (v[ACC_W-1:OUT_W-1] != {(ACC_W-OUT_W+1){1'b1}});
`ifdef MUT_WRAP
      sat12 = {ovf, v[OUT_W-1:0]};
`else
      sat12 = ovf ? {1'b1, (v[ACC_W-1] ? Y_MIN : Y_MAX)} : {1'b0, v[OUT_W-1:0]};
`endif
    end
  endfunction

  reg [OUT_W-1:0] y_i = 0, y_q = 0;
  reg             sat_r = 1'b0;
  wire [OUT_W:0]  ri = sat12(sh_i);
  wire [OUT_W:0]  rq = sat12(sh_q);
  always @(posedge clk) begin
    y_i   <= ri[OUT_W-1:0];
    y_q   <= rq[OUT_W-1:0];
    sat_r <= ri[OUT_W] | rq[OUT_W];
  end

  // ------------------------------------------------------------------ outputs
  assign valid_out = sb_v[SB_LEN-1];
  assign sat_out   = sat_r & sb_v[SB_LEN-1];
`ifdef MUT_IQ_SWAP_OUT
  assign dout_i = {y_q, 4'b0};
  assign dout_q = {y_i, 4'b0};
`elsif MUT_NO_DAC_SHIFT
  assign dout_i = {{(16-OUT_W){y_i[OUT_W-1]}}, y_i};
  assign dout_q = {{(16-OUT_W){y_q[OUT_W-1]}}, y_q};
`else
  assign dout_i = {y_i, {(16-OUT_W){1'b0}}};
  assign dout_q = {y_q, {(16-OUT_W){1'b0}}};
`endif

  `undef TAG
endmodule
