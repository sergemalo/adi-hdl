`timescale 1ns/1ps

// Control inputs arrive from the AXI clock domain (clk_fpga_0) and are
// registered into the datapath domain (clk = util_ad9361_divclk/clk_out =
// clk_div_sel_1_s). This (a) removes the placement pull stretching the
// FIFO->cpack route and (b) makes the crossing a clean flop-to-flop CDC.
// The DATA path (din -> mux -> dout) stays combinational, so sample timing
// and the enable/fifo_wr_en handshake are unchanged.
module iq_override #(
  parameter DATA_WIDTH = 16
) (
  input  wire                  clk,          // util_ad9361_divclk/clk_out

  input  wire [DATA_WIDTH-1:0] din_0, din_1, din_2, din_3,
  input  wire [DATA_WIDTH-1:0] din_4, din_5, din_6, din_7,

  input  wire [31:0]           ctrl_iq,      // async: [15:0]=I, [31:16]=Q
  input  wire                  override_en,  // async: 1=inject, 0=passthrough

  output wire [DATA_WIDTH-1:0] dout_0, dout_1, dout_2, dout_3,
  output wire [DATA_WIDTH-1:0] dout_4, dout_5, dout_6, dout_7
);

  (* ASYNC_REG = "TRUE" *) reg        en_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        en_sync = 1'b0;
  reg [31:0] ctrl_iq_r = 32'h0;

  always @(posedge clk) begin
    en_meta   <= override_en;   // 2-FF synchronizer on the enable
    en_sync   <= en_meta;
    ctrl_iq_r <= ctrl_iq;       // quasi-static bus, one register stage
  end

  wire [DATA_WIDTH-1:0] i_val = ctrl_iq_r[15:0];
  wire [DATA_WIDTH-1:0] q_val = ctrl_iq_r[31:16];

  assign dout_0 = en_sync ? i_val : din_0;  // ch0 I
  assign dout_1 = en_sync ? q_val : din_1;  // ch0 Q
  assign dout_2 = en_sync ? i_val : din_2;  // ch1 I
  assign dout_3 = en_sync ? q_val : din_3;  // ch1 Q
  assign dout_4 = en_sync ? i_val : din_4;  // ch2 I
  assign dout_5 = en_sync ? q_val : din_5;  // ch2 Q
  assign dout_6 = en_sync ? i_val : din_6;  // ch3 I
  assign dout_7 = en_sync ? q_val : din_7;  // ch3 Q
endmodule