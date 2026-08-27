`timescale 1ns/1ps
module tb_phase_bank;
  localparam DW=16, CW=18, FR=16;
  reg clk=0, rstn=0, vin=0, asel=0;
  reg [DW-1:0] din=16'sd1000;
  reg [4*CW-1:0] a0,b0,a1,b1;
  wire vout;
  wire [DW-1:0] d[0:7];
  integer cyc=0, armed=0, l, tc[0:7], vv[0:7], errors=0;

  phase_bank #(.DATA_WIDTH(DW),.COEFF_WIDTH(CW),.COEFF_FRAC(FR)) dut(
    .clk(clk),.rstn(rstn),.valid_in(vin),
    .din_0(din),.din_1(din),.din_2(din),.din_3(din),
    .din_4(din),.din_5(din),.din_6(din),.din_7(din),
    .active_sel(asel),.coeff_frac(5'd16),
    .a_bank0(a0),.b_bank0(b0),.a_bank1(a1),.b_bank1(b1),
    .valid_out(vout),
    .dout_0(d[0]),.dout_1(d[1]),.dout_2(d[2]),.dout_3(d[3]),
    .dout_4(d[4]),.dout_5(d[5]),.dout_6(d[6]),.dout_7(d[7]));

  always #5 clk=~clk;
  always @(posedge clk) cyc<=cyc+1;

  // expected bank1 outputs per channel: a1_c = 32768>>c, din=1000, b=0
  // ch0=500 ch1=250 ch2=125 ch3=63
  reg [DW-1:0] exp[0:7];
  initial begin exp[0]=500;exp[1]=500;exp[2]=250;exp[3]=250;exp[4]=125;exp[5]=125;exp[6]=63;exp[7]=63; end

  // record first post-arm deviation from the bank0 (identity=1000) value, per lane
  always @(posedge clk) if (armed) for (l=0;l<8;l=l+1)
    if (tc[l]==-1 && d[l]!==16'sd1000) begin tc[l]=cyc; vv[l]=d[l]; end

  integer j;
  initial begin
    for (j=0;j<8;j=j+1) begin tc[j]=-1; vv[j]=0; end
    // bank0 = identity all channels; bank1 = distinct gains
    a0={4{18'sd65536}}; b0={4{18'sd0}}; b1={4{18'sd0}};
    a1[0*CW+:CW]=18'sd32768; a1[1*CW+:CW]=18'sd16384;
    a1[2*CW+:CW]=18'sd8192;  a1[3*CW+:CW]=18'sd4096;

    rstn=0; vin=0; repeat(3) @(posedge clk); rstn=1; vin=1;
    repeat(10) @(posedge clk);           // bank0 active: all lanes should read 1000
    for (l=0;l<8;l=l+1) if (d[l]!==16'sd1000) begin errors=errors+1;
      $display("  bank0 lane %0d = %0d (exp 1000)", l, d[l]); end
    $display("[bank0] all lanes identity-passthrough checked");

    @(negedge clk); asel=1; armed=1;     // COHERENT SWAP -> bank1
    repeat(12) @(posedge clk);
    // all lanes must have transitioned on the SAME cycle, to their own bank1 value
    for (l=0;l<8;l=l+1) begin
      if (tc[l]==-1) begin errors=errors+1; $display("  lane %0d never switched", l); end
      else if (vv[l]!==exp[l]) begin errors=errors+1;
        $display("  lane %0d switched to %0d (exp %0d)", l, vv[l], exp[l]); end
    end
    for (l=1;l<8;l=l+1) if (tc[l]!==tc[0]) begin errors=errors+1;
      $display("  TEAR: lane %0d switched at cyc %0d but lane0 at %0d", l, tc[l], tc[0]); end
    $display("[swap] all lanes switched on cyc %0d (lane0 ref); per-lane values checked", tc[0]);

    // swap back to bank0 -- should return to 1000 coherently
    for (j=0;j<8;j=j+1) begin tc[j]=-1; vv[j]=0; end
    // re-arm: record first deviation from bank1 value
    @(negedge clk); asel=0;
    repeat(12) @(posedge clk);
    for (l=0;l<8;l=l+1) if (d[l]!==16'sd1000) begin errors=errors+1;
      $display("  after swap-back lane %0d = %0d (exp 1000)", l, d[l]); end
    $display("[swap-back] all lanes returned to identity");

    if (errors==0) $display("RESULT: PASS (coherent single-beat swap across all 8 lanes, no tear, correct per-channel values)");
    else           $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
