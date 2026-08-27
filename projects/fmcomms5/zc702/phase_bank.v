`timescale 1ns/1ps
// ----------------------------------------------------------------------------
// phase_bank.v -- 8-lane (4 complex channel) phase/gain stage. Instantiates four
// phase_rot cores, muxes each channel's {a,b} between the two coefficient banks,
// and synchronizes active_sel ONCE (2-FF ASYNC_REG) fanned to all channels so the
// bank swap is coherent -- every channel adopts the new coefficients on the
// identical sample beat. Mirrors fir_bank.
//
// Lane map: channel c uses din_{2c} = I, din_{2c+1} = Q  (interleaved).
// Option (a): rx0 is a normal rotator; leaving its coefficients at the identity
// reset value makes it transparent. (Swap to a DSP-free delay line later.)
//
// coeff_frac is accepted for interface symmetry but the datapath shift is fixed
// at the build-time COEFF_FRAC (Q2.16); phase_rot is unchanged.
// ----------------------------------------------------------------------------
module phase_bank #(
  parameter integer DATA_WIDTH  = 16,
  parameter integer COEFF_WIDTH = 18,
  parameter integer COEFF_FRAC  = 16
)(
  input  wire clk,
  input  wire rstn,
  input  wire valid_in,
  input  wire [DATA_WIDTH-1:0] din_0, din_1, din_2, din_3,
  input  wire [DATA_WIDTH-1:0] din_4, din_5, din_6, din_7,

  input  wire                     active_sel,     // from AXI domain (async)
  input  wire [4:0]               coeff_frac,     // reserved (shift fixed at COEFF_FRAC)
  input  wire [4*COEFF_WIDTH-1:0] a_bank0, b_bank0, a_bank1, b_bank1,

  output wire valid_out,
  output wire [DATA_WIDTH-1:0] dout_0, dout_1, dout_2, dout_3,
  output wire [DATA_WIDTH-1:0] dout_4, dout_5, dout_6, dout_7
);
  // ---- coherent active_sel CDC: one 2-FF synchronizer, fanned to all channels ----
  (* ASYNC_REG = "TRUE" *) reg sel_meta, sel_sync;
  always @(posedge clk) begin
    if (!rstn) begin sel_meta <= 1'b0; sel_sync <= 1'b0; end
    else       begin sel_meta <= active_sel; sel_sync <= sel_meta; end
  end

  wire [DATA_WIDTH-1:0] din  [0:7];
  wire [DATA_WIDTH-1:0] dout [0:7];
  assign din[0]=din_0; assign din[1]=din_1; assign din[2]=din_2; assign din[3]=din_3;
  assign din[4]=din_4; assign din[5]=din_5; assign din[6]=din_6; assign din[7]=din_7;
  assign dout_0=dout[0]; assign dout_1=dout[1]; assign dout_2=dout[2]; assign dout_3=dout[3];
  assign dout_4=dout[4]; assign dout_5=dout[5]; assign dout_6=dout[6]; assign dout_7=dout[7];

  wire [3:0] vo;
  genvar c;
  generate for (c=0; c<4; c=c+1) begin : ch
    wire signed [COEFF_WIDTH-1:0] a0 = a_bank0[c*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] b0 = b_bank0[c*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] a1 = a_bank1[c*COEFF_WIDTH +: COEFF_WIDTH];
    wire signed [COEFF_WIDTH-1:0] b1 = b_bank1[c*COEFF_WIDTH +: COEFF_WIDTH];
    // one shared sel_sync -> all channels switch banks on the same beat
    wire signed [COEFF_WIDTH-1:0] a_sel = sel_sync ? a1 : a0;
    wire signed [COEFF_WIDTH-1:0] b_sel = sel_sync ? b1 : b0;

    phase_rot #(
      .DATA_WIDTH(DATA_WIDTH), .COEFF_WIDTH(COEFF_WIDTH), .COEFF_FRAC(COEFF_FRAC)
    ) u_rot (
      .clk(clk), .rstn(rstn), .valid_in(valid_in),
      .i_in(din[2*c]), .q_in(din[2*c+1]),
      .a_in(a_sel),    .b_in(b_sel),
      .valid_out(vo[c]), .i_out(dout[2*c]), .q_out(dout[2*c+1])
    );
  end endgenerate

  assign valid_out = vo[0];   // all channels share identical latency
endmodule
