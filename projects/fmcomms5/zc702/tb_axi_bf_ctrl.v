`timescale 1ns/1ps
// ============================================================================
// tb_axi_bf_ctrl — B3 self-checking test of the BFW4 register file.
// Needs $ad_hdl_dir/library/common/up_axi.v. Prints RESULT: PASS / FAIL.
// ============================================================================
module tb_axi_bf_ctrl;
  reg clk = 0, rstn = 0;
  always #5 clk = ~clk;                  // 100 MHz AXI
  reg sclk = 0;
  always #16.276 sclk = ~sclk;           // ~30.72 MHz sample clock (async)

  reg  [15:0] awaddr = 0, araddr = 0;
  reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg  [31:0] wdata = 0;
  wire        awready, wready, bvalid, arready, rvalid;
  wire [1:0]  bresp, rresp;
  wire [31:0] rdata;

  wire [8*18-1:0] wb0, wb1;
  wire [4:0]      sh0, sh1;
  wire            asel, txsrc;
  wire [2:0]      capsel;
  reg sat_t = 0, ovf_t = 0, unf_t = 0, primed = 0, en_i = 0, en_q = 0;

  axi_bf_ctrl dut (
    .w_bank0(wb0), .w_bank1(wb1), .shift_bank0(sh0), .shift_bank1(sh1),
    .active_sel(asel), .tx_src(txsrc), .capture_sel(capsel),
    .sat_toggle(sat_t), .ovf_toggle(ovf_t), .unf_toggle(unf_t),
    .fifo_primed(primed), .dac_enable_i(en_i), .dac_enable_q(en_q),
    .s_axi_aclk(clk), .s_axi_aresetn(rstn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  integer errors = 0, k, n;
  reg [31:0] rd;
  initial begin #2000000; $display("WATCHDOG"); $display("RESULT: FAIL"); $finish; end

  task aw(input [15:0] a, input [31:0] d); begin
    @(negedge clk); awaddr = a; wdata = d; awvalid = 1; wvalid = 1; bready = 1;
    @(negedge clk); while (!bvalid) @(negedge clk);
    awvalid = 0; wvalid = 0; @(negedge clk); bready = 0;
  end endtask

  task ar(input [15:0] a); begin
    @(negedge clk); araddr = a; arvalid = 1; rready = 1;
    @(negedge clk); while (!rvalid) @(negedge clk);
    rd = rdata; arvalid = 0; @(negedge clk); rready = 0;
  end endtask

  task chk(input [31:0] got, input [31:0] exp, input [8*40-1:0] name); begin
    if (got !== exp) begin errors = errors + 1;
      $display("  MISMATCH %0s got=0x%08x exp=0x%08x", name, got, exp); end
  end endtask

  // bank b word w byte address
  function [15:0] BA(input integer b, input integer w); BA = 16'h100 + 16'h40*b + 4*w; endfunction

  initial begin
    repeat (4) @(posedge clk); rstn = 1; repeat (2) @(posedge clk);

    // ---- identity and IDLE declaration ----
    ar(16'h000); chk(rd, 32'h42465734, "ID");
    ar(16'h024); chk(rd, {8'd0, 8'd16, 8'd8, 8'd13}, "INFO");
    for (k = 2; k <= 8; k = k + 1) begin ar(4*k); chk(rd, 32'd0, "reset value ctrl/status/count"); end
    for (k = 0; k < 2; k = k + 1) for (n = 0; n < 16; n = n + 1) begin
      ar(BA(k, n)); chk(rd, 32'd0, "bank reset = 0");
    end
    if (wb0 !== 0 || wb1 !== 0 || sh0 !== 0 || sh1 !== 0 || asel !== 0 || txsrc !== 0 || capsel !== 0) begin
      errors = errors + 1; $display("  outputs not at IDLE declaration"); end

    // ---- bank write/readback, sign extension, bus layout (BANK_FIELD_ORDER) ----
    for (n = 0; n < 8; n = n + 1) aw(BA(1, n), (n & 1) ? -(n*1000 + 7) : (n*1000 + 3));
    aw(BA(1, 8), 32'h000000FF);                   // SHIFT keeps [4:0]
    aw(BA(1, 9), 32'hDEADBEEF);
    for (n = 0; n < 8; n = n + 1) begin
      ar(BA(1, n)); chk(rd, (n & 1) ? -(n*1000 + 7) : (n*1000 + 3), "bank1 weight r/w");
      if ($signed(wb1[n*18 +: 18]) !== ((n & 1) ? -(n*1000 + 7) : (n*1000 + 3))) begin
        errors = errors + 1; $display("  w_bank1 field %0d wrong on the bus", n); end
    end
    ar(BA(1, 8)); chk(rd, 32'h1F, "SHIFT masked to [4:0]");
    if (sh1 !== 5'h1F) begin errors = errors + 1; $display("  shift_bank1 bus"); end
    ar(BA(1, 9)); chk(rd, 32'hDEADBEEF, "WEIGHT_SEQ");
    if (wb0 !== 0) begin errors = errors + 1; $display("  bank0 disturbed by bank1 writes"); end
    aw(BA(0, 0), 32'h00020000);                   // not a sign-extended s18
    ar(BA(0, 0)); chk(rd, 32'hFFFE0000, "18-bit storage, sign-extended readback");
    aw(BA(0, 0), 32'h0001FFFF); ar(BA(0, 0)); chk(rd, 32'h0001FFFF, "W_MAX");
    aw(BA(0, 1), 32'hFFFE0000); ar(BA(0, 1)); chk(rd, 32'hFFFE0000, "W_MIN");

    // ---- no aliasing ----
    aw(BA(0, 10), 32'h12345); ar(BA(0, 10)); chk(rd, 32'd0, "bank word 10 unmapped");
    aw(16'h0180, 32'h12345);  ar(16'h0180);  chk(rd, 32'd0, "beyond bank region");
    aw(16'h4100, 32'h00777);  ar(BA(0, 0));  chk(rd, 32'h0001FFFF, "0x4100 must not alias bank0 W0_RE");
    aw(16'h1140, 32'h00555);  ar(BA(1, 0));  chk(rd, 32'd3, "0x1140 must not alias bank1 W0_RE");
    ar(16'h0FFC); chk(rd, 32'd0, "unmapped reads 0");

    // ---- controls ----
    aw(16'h008, 32'hFFFFFFFF); ar(16'h008); chk(rd, 32'd1, "ACTIVE_SEL [0] only");
    aw(16'h00C, 32'd1);        ar(16'h00C); chk(rd, 32'd1, "TX_SRC");
    aw(16'h010, 32'd3);        ar(16'h010); chk(rd, 32'd3, "CAPTURE_SEL");
    if (asel !== 1 || txsrc !== 1 || capsel !== 3) begin errors = errors + 1; $display("  control outputs"); end
    aw(16'h004, 32'hA5A5F00D); ar(16'h004); chk(rd, 32'hA5A5F00D, "SCRATCH");

    // ---- live status ----
    primed = 1; en_i = 1; en_q = 0; repeat (6) @(posedge clk);
    ar(16'h014); chk(rd, 32'h3, "STATUS live: primed, dma_i");
    en_q = 1; repeat (6) @(posedge clk);
    ar(16'h014); chk(rd, 32'h7, "STATUS live: + dma_q");

    // ---- events: every toggle counted, from a ~30.72 MHz domain ----
    for (n = 0; n < 1000; n = n + 1) begin @(posedge sclk); sat_t = ~sat_t; end
    for (n = 0; n < 37; n = n + 1)   begin @(posedge sclk); ovf_t = ~ovf_t; end
    for (n = 0; n < 5; n = n + 1)    begin @(posedge sclk); unf_t = ~unf_t; end
    repeat (8) @(posedge clk);
    ar(16'h018); chk(rd, 32'd1000, "SAT_COUNT exact at full sample rate");
    ar(16'h020); chk(rd, 32'd37, "OVF_COUNT");
    ar(16'h01C); chk(rd, 32'd5, "UNF_COUNT");
    ar(16'h014); chk(rd, 32'h707, "STATUS sticky set");
    aw(16'h014, 32'h00000200);                    // clear OVF only
    ar(16'h014); chk(rd, 32'h507, "W1C clears only OVF");
    ar(16'h020); chk(rd, 32'd37, "W1C does not clear the counter");
    aw(16'h014, 32'h00000500);
    ar(16'h014); chk(rd, 32'h007, "W1C clears UNF+SAT");
    @(posedge sclk); unf_t = ~unf_t; repeat (8) @(posedge clk);
    ar(16'h014); chk(rd, 32'h107, "new event sets sticky again");

    // ---- reset returns to IDLE declaration ----
    rstn = 0; repeat (3) @(posedge clk); rstn = 1; repeat (2) @(posedge clk);
    if (wb0 !== 0 || wb1 !== 0 || sh1 !== 0 || asel !== 0 || txsrc !== 0 || capsel !== 0) begin
      errors = errors + 1; $display("  reset did not restore IDLE outputs"); end
    ar(16'h018); chk(rd, 32'd0, "reset clears SAT_COUNT");

    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
