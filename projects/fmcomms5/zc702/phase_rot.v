`timescale 1ns/1ps
// ----------------------------------------------------------------------------
// phase_rot.v -- per-channel 2x2 complex rotate + gain (phase/gain cal core)
//
//   I' = (a*I - b*Q + round) >>> COEFF_FRAC   (arithmetic), saturated to DATA_WIDTH
//   Q' = (b*I + a*Q + round) >>> COEFF_FRAC
//
//   (a,b) = g*(cos(phi), sin(phi)) in Q2.16 (COEFF_FRAC=16). IDENTITY is
//   a = (1<<COEFF_FRAC) = 65536, b = 0  ->  output == input, delayed by LATENCY.
//   Q2.16 is required (not Q1.17) so a=1.0 is representable: 1<<16=65536 fits an
//   18-bit signed field (max 131071); 1<<17 would overflow. Same reason as the FIR.
//
//   Coefficients are PORTS (driven later by the double-buffered bank). For first
//   bring-up, tie a_in=18'sd65536, b_in=18'sd0 to get a pass-through THROUGH the
//   real pipeline -- a broken pipeline then shows up immediately as corrupted or
//   time-shifted data against the known-good baseline.
//
//   LATENCY (localparam below) = clocks from a valid input to its valid output.
//   The reference lane (rx0) MUST be delayed by EXACTLY LATENCY to stay aligned
//   with the rotated channels (a plain shift register -- see the notes with this
//   file). One instance handles the I/Q pair of ONE complex channel.
//
//   The data pipeline free-runs every clock; `valid` is delay-matched. Coeffs are
//   quasi-static (change only on a bank swap) so they are not pipelined vs valid.
// ----------------------------------------------------------------------------
module phase_rot #(
  parameter integer DATA_WIDTH  = 16,   // sample lane width, signed
  parameter integer COEFF_WIDTH = 18,   // coeff width, signed (Q2.16 -> 18 bits)
  parameter integer COEFF_FRAC  = 16    // fractional bits of a,b
) (
  input  wire                          clk,
  input  wire                          rstn,      // active-low; clears valid only. Tie 1'b1 if unused.
  input  wire                          valid_in,
  input  wire signed [DATA_WIDTH-1:0]  i_in,
  input  wire signed [DATA_WIDTH-1:0]  q_in,
  input  wire signed [COEFF_WIDTH-1:0] a_in,      // Q2.16; identity = 1<<COEFF_FRAC
  input  wire signed [COEFF_WIDTH-1:0] b_in,      // Q2.16; identity = 0
  output reg                           valid_out,
  output reg  signed [DATA_WIDTH-1:0]  i_out,
  output reg  signed [DATA_WIDTH-1:0]  q_out
);

  // ---- fixed pipeline depth: rx0's delay line MUST match this ----
  localparam integer LATENCY = 4;

  localparam integer PROD_W = DATA_WIDTH + COEFF_WIDTH;   // 34: full product
  localparam integer ACC_W  = PROD_W + 2;                 // 36: two products + round bias
  localparam signed [ACC_W-1:0] ROUND =
             (COEFF_FRAC == 0) ? {ACC_W{1'b0}} : (1 <<< (COEFF_FRAC-1));
  localparam signed [ACC_W-1:0] SAT_MAX =  (1 <<< (DATA_WIDTH-1)) - 1;  //  32767
  localparam signed [ACC_W-1:0] SAT_MIN = -(1 <<< (DATA_WIDTH-1));      // -32768

  // stage 1: register inputs (and coeffs)
  reg signed [DATA_WIDTH-1:0]  i1, q1;
  reg signed [COEFF_WIDTH-1:0] a1, b1;
  reg                          v1;

  // stage 2: the four products (let synthesis map these to DSP48s)
  (* use_dsp = "yes" *) reg signed [PROD_W-1:0] ai2, bq2, bi2, aq2;
  reg                          v2;

  // stage 3: rotate sums with rounding bias
  reg signed [ACC_W-1:0] acc_i3, acc_q3;
  reg                    v3;

  // stage 4 (registered outputs): arithmetic shift + saturate
  wire signed [ACC_W-1:0] shr_i = acc_i3 >>> COEFF_FRAC;
  wire signed [ACC_W-1:0] shr_q = acc_q3 >>> COEFF_FRAC;

  function signed [DATA_WIDTH-1:0] sat;
    input signed [ACC_W-1:0] x;
    begin
      if      (x > SAT_MAX) sat = SAT_MAX[DATA_WIDTH-1:0];
      else if (x < SAT_MIN) sat = SAT_MIN[DATA_WIDTH-1:0];
      else                  sat = x[DATA_WIDTH-1:0];
    end
  endfunction

  always @(posedge clk) begin
    // stage 1
    i1 <= i_in;  q1 <= q_in;  a1 <= a_in;  b1 <= b_in;
    // stage 2
    ai2 <= a1 * i1;
    bq2 <= b1 * q1;
    bi2 <= b1 * i1;
    aq2 <= a1 * q1;
    // stage 3  (operands are signed -> sign-extended into ACC_W)
    acc_i3 <= ai2 - bq2 + ROUND;
    acc_q3 <= bi2 + aq2 + ROUND;
    // stage 4
    i_out <= sat(shr_i);
    q_out <= sat(shr_q);
  end

  // valid pipeline (reset so no spurious valid at power-up)
  always @(posedge clk) begin
    if (!rstn) begin
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; valid_out <= 1'b0;
    end else begin
      v1 <= valid_in; v2 <= v1; v3 <= v2; valid_out <= v3;
    end
  end

endmodule
