`timescale 1ns/1ps
// ============================================================================
// bf_capture — debug mux on the RX DMA path (D7)
//
// Sits between phase_bank_0/dout_* and util_ad9361_adc_pack/fifo_wr_data_*,
// AFTER the covar tap, so covar_bank never sees it. With CAPTURE_SEL = 0 it
// is transparent (one LUT per bit, no register: the pack path timing and
// latency are unchanged). With CAPTURE_SEL = c+1 (c = 0..3) the two lanes of
// channel c carry bf_bank's DAC words instead, so an ordinary RX capture
// records the exact BF output beside the other three inputs. Values 5..7
// behave as 0.
//
// The BF words lag their inputs by bf_bank's LATENCY (13 samples); the host
// aligns on that (axi_bf_ctrl INFO register).
//
// capture_sel comes from the AXI domain: 2-FF synchronised per bit. During a
// change the bits may disagree for one sample; harmless for a debug mux.
// ============================================================================
module bf_capture (
  input  wire        clk,
  input  wire [15:0] din_0, din_1, din_2, din_3,
  input  wire [15:0] din_4, din_5, din_6, din_7,
  input  wire [15:0] bf_i,
  input  wire [15:0] bf_q,
  input  wire [2:0]  capture_sel,     // async
  output wire [15:0] dout_0, dout_1, dout_2, dout_3,
  output wire [15:0] dout_4, dout_5, dout_6, dout_7
);
  (* ASYNC_REG = "TRUE" *) reg [2:0] bfx_m1_cap = 3'd0;
  (* ASYNC_REG = "TRUE" *) reg [2:0] bfx_m2_cap = 3'd0;
  always @(posedge clk) begin
    bfx_m1_cap <= capture_sel;
    bfx_m2_cap <= bfx_m1_cap;
  end

  wire c0 = (bfx_m2_cap == 3'd1);
  wire c1 = (bfx_m2_cap == 3'd2);
  wire c2 = (bfx_m2_cap == 3'd3);
  wire c3 = (bfx_m2_cap == 3'd4);

  assign dout_0 = c0 ? bf_i : din_0;
  assign dout_1 = c0 ? bf_q : din_1;
  assign dout_2 = c1 ? bf_i : din_2;
  assign dout_3 = c1 ? bf_q : din_3;
  assign dout_4 = c2 ? bf_i : din_4;
  assign dout_5 = c2 ? bf_q : din_5;
  assign dout_6 = c3 ? bf_i : din_6;
  assign dout_7 = c3 ? bf_q : din_7;
endmodule
