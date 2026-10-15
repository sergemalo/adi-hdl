`timescale 1ns/1ps
// ============================================================================
// tb_bf_bank — replays a stimulus file into bf_bank and records every valid
// output. It does NOT judge anything: py-examples/check_bf_sim.py compares the
// record against bf_golden. Plain Verilog-2001 file I/O, so it runs in Icarus
// and in Vivado xsim alike.
//
//   +stim=<file>   one line per clock, 28 hex fields:
//                  valid sel din0..din7 b0w0..b0w7 b0shift b1w0..b1w7 b1shift
//   +out=<file>    "# LATENCY n", "# LINE0_EDGE e", then per valid output:
//                  "<edge> <dout_i> <dout_q> <sat>"   (edge = posedge count
//                  after which the output was observed)
// ============================================================================
module tb_bf_bank;
  reg clk = 1'b0;
  always #5 clk = ~clk;

  reg         valid = 1'b0, sel = 1'b0;
  reg  [15:0] d0 = 0, d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0, d6 = 0, d7 = 0;
  reg  [8*18-1:0] wb0 = 0, wb1 = 0;
  reg  [4:0]  sh0 = 0, sh1 = 0;
  wire        vo, sat;
  wire [15:0] oi, oq;

`ifdef ACC_W_OVERRIDE
  defparam dut.ACC_W = `ACC_W_OVERRIDE;     // mutation: accumulator too narrow
`endif
  bf_bank dut (
    .clk(clk), .valid_in(valid),
    .din_0(d0), .din_1(d1), .din_2(d2), .din_3(d3),
    .din_4(d4), .din_5(d5), .din_6(d6), .din_7(d7),
    .active_sel(sel),
    .w_bank0(wb0), .w_bank1(wb1), .shift_bank0(sh0), .shift_bank1(sh1),
    .valid_out(vo), .dout_i(oi), .dout_q(oq), .sat_out(sat)
  );

  integer pe = 0;                       // posedges so far
  always @(posedge clk) pe <= pe + 1;

  reg [1023:0] stim_path, out_path;
  integer fin, fout, r, line, k, xbad;
  reg [31:0] f [0:27];

  task record;                          // called at a negedge
    begin
      if (vo === 1'b1) begin
        if ((^oi === 1'bx) || (^oq === 1'bx) || (sat === 1'bx)) xbad = xbad + 1;
        $fwrite(fout, "%0d %h %h %0d\n", pe, oi, oq, sat);
      end else if (vo !== 1'b0) xbad = xbad + 1;
    end
  endtask

  initial begin
    xbad = 0;
    if (!$value$plusargs("stim=%s", stim_path)) begin $display("need +stim="); $finish; end
    if (!$value$plusargs("out=%s",  out_path))  begin $display("need +out=");  $finish; end
    fin  = $fopen(stim_path, "r");
    fout = $fopen(out_path,  "w");
    if (fin == 0 || fout == 0) begin $display("cannot open files"); $finish; end
    $fwrite(fout, "# LATENCY %0d\n", dut.LATENCY);

    line = 0;
    @(negedge clk);
    $fwrite(fout, "# LINE0_EDGE %0d\n", pe + 1);
    while (!$feof(fin)) begin
      r = $fscanf(fin, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h\n",
                  f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9],
                  f[10], f[11], f[12], f[13], f[14], f[15], f[16], f[17], f[18], f[19],
                  f[20], f[21], f[22], f[23], f[24], f[25], f[26], f[27]);
      if (r == 28) begin
        if (line > 0) @(negedge clk);
        record;
        valid = f[0][0];  sel = f[1][0];
        d0 = f[2]; d1 = f[3]; d2 = f[4]; d3 = f[5];
        d4 = f[6]; d5 = f[7]; d6 = f[8]; d7 = f[9];
        for (k = 0; k < 8; k = k + 1) begin
          wb0[k*18 +: 18] = f[10 + k][17:0];
          wb1[k*18 +: 18] = f[19 + k][17:0];
        end
        sh0 = f[18][4:0];
        sh1 = f[27][4:0];
        line = line + 1;
      end else if (r > 0) begin
        $display("malformed stimulus line %0d (%0d fields)", line, r);
        $finish;
      end
    end
    // let the posedge capture the last line before dropping valid
    repeat (40) begin @(negedge clk); record; valid = 1'b0; end
    $fwrite(fout, "# LINES %0d\n# XBAD %0d\n", line, xbad);
    $fclose(fout);
    $display("tb_bf_bank: %0d stimulus lines, X-violations %0d", line, xbad);
    $finish;
  end
endmodule
