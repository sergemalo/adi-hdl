// fir_i0.v
//
// Single-lane direct-form FIR with a latency-matched raw bypass lane,
// for golden-model validation against a Python reference.
//
//   dout_fir  =  FIR(din)          delayed LATENCY sample beats
//   dout_ref  =  din               delayed LATENCY sample beats
//
// Both outputs are aligned, so in Python:
//
//   assert (np.convolve(ref, h)[:len(fir)] == fir).all()      # for COEFF_FRAC = 0
//
// EVERY register in this module is enabled by `valid`, so LATENCY counts
// sample beats, not clock cycles.  If you add or remove a pipeline stage,
// LATENCY must be updated to match or the two lanes will skew.
//
// Fixed point:
//   din          signed DATA_WIDTH
//   coefficients signed COEFF_WIDTH, coeff_frac fractional bits (Q(CW-F).F)
//   coeff_frac is a RUNTIME input (from axi_fir_ctrl), not a parameter.
//   coeff_frac = 0  ->  plain integer coefficients, no shift, no rounding
//
// The output stage rounds to nearest (not truncate-toward-minus-infinity,
// which would inject a DC bias) and saturates rather than wrapping.
//
// CDC: coeff_flat is written in the s_axi_aclk domain and read here in the
// `clk` domain.  Coefficients must be static while samples are streaming.
// Add: set_false_path -from [get_cells .../up_coeff_reg*] -to [get_cells ...]
// Double buffering removes this restriction; it is a later step.

`timescale 1ns/100ps

module fir_i0 #(
  parameter NUM_COEFF   = 3,
  parameter DATA_WIDTH  = 16,
  parameter COEFF_WIDTH = 18,
  parameter FRAC_WIDTH  = 5      // width of the runtime coeff_frac field
) (
  input                                     clk,
  input                                     valid,

  input      signed [DATA_WIDTH-1:0]        din,
  input      [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat,
  input      [FRAC_WIDTH-1:0]               coeff_frac,   // runtime Q-format shift

  output     signed [DATA_WIDTH-1:0]        dout_fir,
  output     signed [DATA_WIDTH-1:0]        dout_ref,
  output                                    sat        // 1 = output clamped
);

  // ---------------------------------------------------------------------
  // Widths
  //   product     = DATA_WIDTH + COEFF_WIDTH
  //   accumulator = product + ceil(log2(NUM_COEFF))   worst-case bit growth
  // ---------------------------------------------------------------------

  localparam PROD_WIDTH = DATA_WIDTH + COEFF_WIDTH;
  localparam ACC_WIDTH  = PROD_WIDTH + $clog2(NUM_COEFF);

  // stages: x -> p -> acc -> dout_fir_r
  localparam LATENCY = 5;

  localparam signed [ACC_WIDTH-1:0] MAXV =  (1 <<< (DATA_WIDTH-1)) - 1;
  localparam signed [ACC_WIDTH-1:0] MINV = -(1 <<< (DATA_WIDTH-1));

  integer i;
  genvar  n;

  // ---------------------------------------------------------------------
  // Unpack coefficients.  coeff_flat[k] occupies bits [(k+1)*CW-1 : k*CW]
  // ---------------------------------------------------------------------

  wire signed [COEFF_WIDTH-1:0] c [0:NUM_COEFF-1];

  generate
    for (n = 0; n < NUM_COEFF; n = n + 1) begin: g_unpack
      assign c[n] = coeff_flat[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH];
    end
  endgenerate

  // ---------------------------------------------------------------------
  // Stage 1: tapped delay line.  x[0] is the newest sample.
  // Shifts once per SAMPLE, not once per clock.
  // ---------------------------------------------------------------------

  reg signed [DATA_WIDTH-1:0] x [0:NUM_COEFF-1];

  always @(posedge clk) begin
    if (valid == 1'b1) begin
      x[0] <= din;
      for (i = 1; i < NUM_COEFF; i = i + 1) begin
        x[i] <= x[i-1];
      end
    end
  end

  // ---------------------------------------------------------------------
  // Stage 2: multiply.  One DSP48E1 per tap; the register here maps to the
  // DSP's internal M register, so synthesis keeps the multiply pipelined.
  // ---------------------------------------------------------------------

  reg signed [PROD_WIDTH-1:0] p [0:NUM_COEFF-1];

  always @(posedge clk) begin
    if (valid == 1'b1) begin
      for (i = 0; i < NUM_COEFF; i = i + 1) begin
        p[i] <= x[i] * c[i];
      end
    end
  end

  // ---------------------------------------------------------------------
  // Stage 3: accumulate.
  //
  // A flat adder tree is fine at NUM_COEFF = 3.  At 21 taps this becomes
  // the critical path; the fix is a DSP48E1 PCOUT->PCIN cascade (one add
  // per tap, pipelined), which changes LATENCY.
  // ---------------------------------------------------------------------

  reg signed [ACC_WIDTH-1:0] acc_comb;
  reg signed [ACC_WIDTH-1:0] acc;

  always @(*) begin
    acc_comb = {ACC_WIDTH{1'b0}};
    for (i = 0; i < NUM_COEFF; i = i + 1) begin
      acc_comb = acc_comb + p[i];   // p[i] is signed -> sign-extended
    end
  end

  always @(posedge clk) begin
    if (valid == 1'b1) begin
      acc <= acc_comb;
    end
  end

  // ---------------------------------------------------------------------
  // Stage 4: round, rescale, saturate.
  // With COEFF_FRAC = 0 this is a pure saturating truncation to DATA_WIDTH.
  // ---------------------------------------------------------------------

  // Round-half-up then arithmetic shift, both driven by the runtime
  // coeff_frac register:  rnd = 2^(coeff_frac - 1), or 0 when coeff_frac == 0.
  // The variable >>> is a barrel shifter (LUTs, no DSP).
  wire [ACC_WIDTH-1:0] one   = {{(ACC_WIDTH-1){1'b0}}, 1'b1};
  wire [ACC_WIDTH-1:0] rnd_u = (coeff_frac == 0)
                             ? {ACC_WIDTH{1'b0}}
                             : (one << (coeff_frac - 1'b1));

  wire signed [ACC_WIDTH-1:0] acc_rnd = acc + $signed(rnd_u);
  wire signed [ACC_WIDTH-1:0] acc_scl = acc_rnd >>> coeff_frac;

  wire hi = (acc_scl > MAXV);
  wire lo = (acc_scl < MINV);

  reg signed [DATA_WIDTH-1:0] dout_fir_r;
  reg                         sat_r;

  // stage 4a: round + rescale (registered)
  reg signed [ACC_WIDTH-1:0] acc_scl_r;
  always @(posedge clk) if (valid) acc_scl_r <= (acc + $signed(rnd_u)) >>> coeff_frac;

  // stage 4b: saturate (registered)
  wire hi = (acc_scl_r > MAXV);
  wire lo = (acc_scl_r < MINV);
  always @(posedge clk) if (valid) begin
    sat_r <= hi | lo;
    dout_fir_r <= hi ? MAXV[DATA_WIDTH-1:0] : lo ? MINV[DATA_WIDTH-1:0] : acc_scl_r[DATA_WIDTH-1:0];
  end

  // ---------------------------------------------------------------------
  // Reference lane: din delayed by exactly LATENCY sample beats.
  //
  // Functionally identical to running a second FIR with h = [1,0,0], but
  // with no DSPs and no dependence on the coefficient bus -- so a bug in
  // the coefficient path cannot corrupt the reference the FIR is checked
  // against.  Enabled by the same `valid`, which is the whole point: if
  // `valid` is wrong, the FIR's taps get misspaced and the reference does
  // not, and Python sees the mismatch.
  // ---------------------------------------------------------------------

  reg signed [DATA_WIDTH-1:0] r [0:LATENCY-1];

  always @(posedge clk) begin
    if (valid == 1'b1) begin
      r[0] <= din;
      for (i = 1; i < LATENCY; i = i + 1) begin
        r[i] <= r[i-1];
      end
    end
  end

  assign dout_fir = dout_fir_r;
  assign dout_ref = r[LATENCY-1];
  assign sat      = sat_r;

endmodule
