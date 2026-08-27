`timescale 1ns/1ps
module tb_axi;
  localparam CW=18;
  reg clk=0, rstn=0;
  reg  [15:0] awaddr=0, araddr=0; reg awvalid=0, wvalid=0, bready=0, arvalid=0, rready=0;
  reg  [31:0] wdata=0; reg [3:0] wstrb=0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp;
  wire [31:0] rdata; wire active_sel; wire [4:0] cfrac;
  wire [4*CW-1:0] a0,b0,a1,b1;
  integer errors=0; reg [31:0] rd;

  axi_phase_ctrl #(.COEFF_WIDTH(CW),.COEFF_FRAC(16)) dut(
    .active_sel(active_sel),.coeff_frac_o(cfrac),
    .a_bank0(a0),.b_bank0(b0),.a_bank1(a1),.b_bank1(b1),
    .s_axi_aclk(clk),.s_axi_aresetn(rstn),
    .s_axi_awaddr(awaddr),.s_axi_awprot(3'd0),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
    .s_axi_wdata(wdata),.s_axi_wstrb(wstrb),.s_axi_wvalid(wvalid),.s_axi_wready(wready),
    .s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
    .s_axi_araddr(araddr),.s_axi_arprot(3'd0),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
    .s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready));
  always #5 clk=~clk;
  initial begin #50000; $display("WATCHDOG"); $finish; end

  task aw; input [15:0] addr; input [31:0] data; begin           // negedge-driven, race-free
    @(negedge clk); awaddr=addr; wdata=data; wstrb=4'hF; awvalid=1; wvalid=1; bready=1;
    @(negedge clk); while(!bvalid) @(negedge clk);
    awvalid=0; wvalid=0; @(negedge clk); bready=0; end endtask

  task ar; input [15:0] addr; begin
    @(negedge clk); araddr=addr; arvalid=1; rready=1;
    @(negedge clk); while(!rvalid) @(negedge clk);
    rd=rdata; arvalid=0; @(negedge clk); rready=0; end endtask

  task chk; input [31:0] got, exp; input [255:0] name; begin
    if (got!==exp) begin errors=errors+1; $display("  MISMATCH %0s got=0x%08x exp=0x%08x",name,got,exp); end
    else $display("  ok %0s = 0x%08x", name, got); end endtask

  initial begin
    rstn=0; repeat(4) @(posedge clk); rstn=1; repeat(2) @(posedge clk);
    ar(16'h0000); chk(rd, 32'h50484734, "ID");
    ar(16'h8000); chk(rd, 32'd65536, "ch0.b0.a identity reset");
    ar(16'h8180); chk(rd, 32'd65536, "ch1.b1.a identity reset");
    ar(16'h8004); chk(rd, 32'd0,     "ch0.b0.b identity reset");
    aw(16'h8180, 32'd12345); ar(16'h8180); chk(rd, 32'd12345, "ch1.b1.a write/read");
    if (a1[1*CW+:CW]!==18'sd12345) begin errors=errors+1; $display("  a_bank1[ch1] bus != 12345"); end
    else $display("  ok a_bank1[ch1] bus reflects 12345");
    aw(16'h8204, 32'hFFFFFFCE); ar(16'h8204); chk(rd, 32'hFFFFFFCE, "ch2.b0.b = -50 signed r/w");
    aw(16'h000C, 32'd1); repeat(2) @(posedge clk);
    if (active_sel!==1'b1) begin errors=errors+1; $display("  active_sel not set"); end
    else $display("  ok active_sel=1 after CTRL write");
    ar(16'h000C); chk(rd, 32'd1,  "CTRL readback");
    ar(16'h0008); chk(rd, 32'd16, "CFG coeff_frac reset=16");
    if (errors==0) $display("RESULT: PASS (ID, identity reset, signed coeff r/w at mapped addrs, bus + active_sel)");
    else           $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
