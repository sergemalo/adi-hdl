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
// sample beats, not clock cycles -- EXCEPT the active_sel synchronizer, which
// must free-run (see below).  If you add or remove a pipeline stage in the
// datapath, LATENCY must be updated to match or the two lanes will skew.
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
// DOUBLE BUFFERING / CDC:
//   Two coefficient banks (coeff_flat0/1) arrive from axi_fir_ctrl in the
//   s_axi_aclk domain.  active_sel (also s_axi_aclk) chooses which bank feeds
//   the multipliers.  Only active_sel crosses into `clk`, through a 2-FF
//   synchronizer; the wide banks are quasi-static (software only writes the
//   INACTIVE bank, then flips active_sel) and cross via a false_path.
//
//   The bank mux is COMBINATIONAL, so LATENCY is unchanged and the reference
//   lane stays aligned.  A single synchronized select drives all taps, so a
//   given output sample uses an all-bank0 or all-bank1 set -- never a mix.
//   Worst case a metastable select resolves one clock early/late, shifting
//   WHEN the swap lands by one sample; it can never corrupt a coefficient.
//
//   Constraints (match your instance hierarchy):
//     set_false_path -from [get_cells .../up_coeff0_reg* .../up_coeff1_reg*] \
//                    -to   [get_cells .../g_unpack*]
//     set_false_path -from [get_cells .../up_active_sel_reg*] \
//                    -to   [get_cells .../sel_meta_reg*]

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
  input      [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat0,  // bank 0 (s_axi_aclk)
  input      [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat1,  // bank 1 (s_axi_aclk)
  input                                     active_sel,   // async: 0=bank0, 1=bank1
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

  // Balanced adder-tree accumulator (stage 3): ceil(log2(NUM_COEFF)) registered
  // levels, built as a perfect binary tree over the products zero-padded up to
  // the next power of two (NP2).  Padding leaves are constant 0, so the "+0"
  // adders fold away -- real adder count is NUM_COEFF-1, all in fabric (no DSP).
  localparam TREE_LEVELS = (NUM_COEFF <= 1) ? 1 : $clog2(NUM_COEFF);
  localparam NP2         = (1 << TREE_LEVELS);   // >= NUM_COEFF
  localparam NP2H        = (NP2 >> 1);           // nodes at level 1 (widest)

  // stages: x -> p -> [adder tree: TREE_LEVELS] -> acc_scl_r -> dout_fir_r
  localparam LATENCY = 4 + TREE_LEVELS;

  localparam signed [ACC_WIDTH-1:0] MAXV =  (1 <<< (DATA_WIDTH-1)) - 1;
  localparam signed [ACC_WIDTH-1:0] MINV = -(1 <<< (DATA_WIDTH-1));

  integer i;
  genvar  n;

  // ---------------------------------------------------------------------
  // active_sel synchronizer.  MUST free-run (no `valid` enable): a
  // synchronizer clocked by an intermittent enable does not resolve
  // metastability.  The mux it drives is combinational, so this adds no
  // datapath latency.
  // ---------------------------------------------------------------------

  (* ASYNC_REG = "TRUE" *) reg sel_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg sel_sync = 1'b0;

  always @(posedge clk) begin
    sel_meta <= active_sel;
    sel_sync <= sel_meta;
  end

  // ---------------------------------------------------------------------
  // Unpack both banks and select.  coeff_flatX[k] occupies bits
  // [(k+1)*CW-1 : k*CW].  One shared sel_sync -> all taps switch together.
  // ---------------------------------------------------------------------

  wire signed [COEFF_WIDTH-1:0] c0 [0:NUM_COEFF-1];
  wire signed [COEFF_WIDTH-1:0] c1 [0:NUM_COEFF-1];
  wire signed [COEFF_WIDTH-1:0] c  [0:NUM_COEFF-1];

  generate
    for (n = 0; n < NUM_COEFF; n = n + 1) begin: g_unpack
      assign c0[n] = coeff_flat0[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH];
      assign c1[n] = coeff_flat1[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH];
      assign c[n]  = sel_sync ? c1[n] : c0[n];
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
  // Stage 3: accumulate via a pipelined balanced adder tree.
  //
  // Products p[] are the tree leaves, sign-extended to ACC_WIDTH and zero-
  // padded to NP2.  Each level registers pairwise sums, so the whole product
  // vector for one sample advances together and the result is uniformly
  // delayed by TREE_LEVELS beats (folded into LATENCY, so the reference lane
  // stays aligned).
  //
  // Bit-exact to the old flat left-fold sum: two's-complement add is
  // associative modulo 2^ACC_WIDTH, so grouping does not matter; ACC_WIDTH is
  // sized so no partial sum overflows in the first place.  All adds are fabric
  // (LUT/CARRY4) -- the multiplies stay in the DSPs, the accumulate does not.
  // ---------------------------------------------------------------------

  // Level 0: products as tree leaves, sign-extended and zero-padded to NP2.
  wire signed [ACC_WIDTH-1:0] leaf [0:NP2-1];
  generate
    for (n = 0; n < NP2; n = n + 1) begin: g_leaf
      if (n < NUM_COEFF) begin: g_leaf_prod
        assign leaf[n] = p[n];                 // signed narrower -> sign-extends
      end else begin: g_leaf_zero
        assign leaf[n] = {ACC_WIDTH{1'b0}};    // padding -> "+0", folds away
      end
    end
  endgenerate

  // Levels 1..TREE_LEVELS: registered pairwise adds.  node[L][j] is the j-th
  // partial sum at level L; the final sum is node[TREE_LEVELS][0].
  //
  // use_dsp="no": keep the accumulate in fabric (LUT/CARRY4).  Without this,
  // Vivado's default DSP inference pulls the wide (ACC_WIDTH) tree adders into
  // spare DSP48 ALUs -- burning ~8 extra DSPs per channel on top of the 21
  // multipliers, which does not fit the 8-channel budget.  The multiplies (p)
  // stay in the DSPs; only the adds are forced out.
  (* use_dsp = "no" *)
  reg signed [ACC_WIDTH-1:0] node [1:TREE_LEVELS][0:NP2H-1];

  genvar lvl, nd;
  generate
    for (lvl = 1; lvl <= TREE_LEVELS; lvl = lvl + 1) begin: g_tree_lvl
      for (nd = 0; nd < (NP2 >> lvl); nd = nd + 1) begin: g_tree_node
        if (lvl == 1) begin: g_from_leaf
          always @(posedge clk) if (valid)
            node[lvl][nd] <= leaf[2*nd] + leaf[(2*nd)+1];
        end else begin: g_from_node
          always @(posedge clk) if (valid)
            node[lvl][nd] <= node[lvl-1][2*nd] + node[lvl-1][(2*nd)+1];
        end
      end
    end
  endgenerate

  wire signed [ACC_WIDTH-1:0] acc = node[TREE_LEVELS][0];

  // ---------------------------------------------------------------------
  // Stage 4: round, rescale, saturate.
  // With COEFF_FRAC = 0 this is a pure saturating truncation to DATA_WIDTH.
  //
  // Round-half-up then arithmetic shift, both driven by the runtime
  // coeff_frac register:  rnd = 2^(coeff_frac - 1), or 0 when coeff_frac == 0.
  // The variable >>> is a barrel shifter (LUTs, no DSP).
  // ---------------------------------------------------------------------

  wire [ACC_WIDTH-1:0] one   = {{(ACC_WIDTH-1){1'b0}}, 1'b1};
  wire [ACC_WIDTH-1:0] rnd_u = (coeff_frac == 0)
                             ? {ACC_WIDTH{1'b0}}
                             : (one << (coeff_frac - 1'b1));

  reg signed [DATA_WIDTH-1:0] dout_fir_r;
  reg                         sat_r;

  // stage 4a: round + rescale (registered).  use_dsp="no": the (acc + rnd_u)
  // add is fabric, not a DSP ALU (see the tree note above).
  (* use_dsp = "no" *)
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
