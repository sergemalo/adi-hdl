// chan_delay.v
//
// Latency-matched pure delay line, replacing fir_i0 for a lane whose
// coefficients are permanently a unit center tap (i.e. an identity filter).
// Reproduces fir_i0's exact din->dout_fir latency for that specific
// coefficient set -- CENTER_TAP + LATENCY sample beats -- with a plain
// shift register: no multipliers, no coefficient bus, no bank select.
//
// Latency derivation (must track fir_i0.v's pipeline exactly):
//   fir_i0 stages: x[tap line] -> p[mult reg] -> adder tree -> acc_scl_r -> dout_fir_r
//   A coefficient at tap index i (all other taps 0) produces output delayed by
//     (i + 1)               tap-line propagation to x[i]
//     + 1                   mult register (p[i])
//     + TREE_LEVELS          adder tree
//     + 1                   round/rescale register (acc_scl_r)
//     + 1                   saturate/output register (dout_fir_r)
//     = i + 4 + TREE_LEVELS = i + fir_i0's LATENCY
//
// rx0's tap is the unit CENTER tap, i = (NUM_COEFF-1)/2, not tap 0 -- so the
// delay this module reproduces is CENTER_TAP + LATENCY, not LATENCY alone.
// Verified bit-exact for the identity case: with coeff_frac=16 (Q2.16) and
// c_reg[CENTER_TAP] = 65536 (1.0), fir_i0's round/rescale stage computes
// (x*65536 + 32768) >>> 16 = x exactly, so this is a true integer delay, not
// an approximation.
//
// If fir_i0's pipeline ever changes (extra stage, different tree structure),
// this module's DELAY must be re-derived to match, or rx0 will silently
// drift out of sample alignment with the other 6 lanes -- a bug that will
// not show up until the array is coherently combined downstream.
//
// Every register here is valid-enabled, exactly like fir_i0, so DELAY counts
// sample beats, not clock cycles -- consistent with the rest of the bank.
//
// NUM_COEFF is a parameter (not a hardcoded 21) purely so this module can be
// instantiated against whatever tap count fir_bank.v is built with; in this
// design it is always driven to $fir_num_coeff (21) from fir_bank's own
// NUM_COEFF parameter, in lock-step with the fir_i0 lanes it sits beside.

`timescale 1ns/100ps

module chan_delay #(
  parameter NUM_COEFF  = 21,     // must match the fir_i0 instances in this bank
  parameter DATA_WIDTH = 16
) (
  input                              clk,
  input                              valid,
  input      signed [DATA_WIDTH-1:0] din,
  output     signed [DATA_WIDTH-1:0] dout_fir,
  output     signed [DATA_WIDTH-1:0] dout_ref,   // = dout_fir; kept for interface parity
  output                             sat         // always 0: a pure delay cannot saturate
);

  generate
    if ((NUM_COEFF % 2) == 0) begin: g_even_guard
      // Center-tap identity is only well-defined for odd-length FIRs (the
      // 21-tap bank's fixed design point per the fractional-delay math note,
      // section 4: "each channel uses a length-L FIR filter with odd length").
      // Halt elaboration loudly rather than silently pick a wrong tap.
      NUM_COEFF_must_be_odd_for_center_tap_delay _bad_num_coeff ();
    end
  endgenerate

  localparam CENTER_TAP  = (NUM_COEFF - 1) / 2;
  localparam TREE_LEVELS = (NUM_COEFF <= 1) ? 1 : $clog2(NUM_COEFF);
  localparam FIR_LATENCY = 4 + TREE_LEVELS;          // must equal fir_i0's LATENCY
  localparam DELAY       = CENTER_TAP + FIR_LATENCY;

  reg signed [DATA_WIDTH-1:0] r [0:DELAY-1];
  integer i;

  always @(posedge clk) begin
    if (valid == 1'b1) begin
      r[0] <= din;
      for (i = 1; i < DELAY; i = i + 1) begin
        r[i] <= r[i-1];
      end
    end
  end

  assign dout_fir = r[DELAY-1];
  assign dout_ref = r[DELAY-1];
  assign sat      = 1'b0;

endmodule
