`timescale 1ns/1ps

// Sits between util_ad9361_adc_fifo (dout) and util_cpack2 (fifo_wr_data).
// Point 1: force I = 0x7FCC, Q = 0 on all four channels.
// Even lanes = I, odd lanes = Q (verified from fmcomms5_bd.tcl).
module iq_override #(
  parameter DATA_WIDTH = 16
) (
  input  wire [DATA_WIDTH-1:0] din_0, din_1, din_2, din_3,
  input  wire [DATA_WIDTH-1:0] din_4, din_5, din_6, din_7,
  output wire [DATA_WIDTH-1:0] dout_0, dout_1, dout_2, dout_3,
  output wire [DATA_WIDTH-1:0] dout_4, dout_5, dout_6, dout_7
);

  localparam [DATA_WIDTH-1:0] I_CONST = 16'h7FCC;
  localparam [DATA_WIDTH-1:0] Q_CONST = 16'h0000;
  localparam                  OVERRIDE = 1'b1;   // Point 2: driven by a register

  assign dout_0 = OVERRIDE ? I_CONST : din_0;  // ch0 I
  assign dout_1 = OVERRIDE ? Q_CONST : din_1;  // ch0 Q
  assign dout_2 = OVERRIDE ? I_CONST : din_2;  // ch1 I
  assign dout_3 = OVERRIDE ? Q_CONST : din_3;  // ch1 Q
  assign dout_4 = OVERRIDE ? I_CONST : din_4;  // ch2 I
  assign dout_5 = OVERRIDE ? Q_CONST : din_5;  // ch2 Q
  assign dout_6 = OVERRIDE ? I_CONST : din_6;  // ch3 I
  assign dout_7 = OVERRIDE ? Q_CONST : din_7;  // ch3 Q

endmodule