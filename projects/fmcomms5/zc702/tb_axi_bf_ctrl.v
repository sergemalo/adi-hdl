`timescale 1ns/1ps
// ============================================================================
// tb_axi_bf_ctrl — self-checking test of the BFW4 register file (rev 2).
// Needs $ad_hdl_dir/library/common/up_axi.v. Prints RESULT: PASS / FAIL.
// The datapath is replaced by plain regs, so the EVENT RULES can be tested
// exactly: events while OUT_EN = 0 or while FAULTED must change nothing.
// ============================================================================
module tb_axi_bf_ctrl;
  reg clk = 0, rstn = 0;
  always #5 clk = ~clk;                  // 100 MHz AXI
  reg sclk = 0;
  always #16.276 sclk = ~sclk;           // ~30.72 MHz event source (async)

  reg  [15:0] awaddr = 0, araddr = 0;
  reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg  [31:0] wdata = 0;
  wire        awready, wready, bvalid, arready, rvalid;
  wire [1:0]  bresp, rresp;
  wire [31:0] rdata;

  wire [8*18-1:0] wb0, wb1;
  wire [4:0]      sh0, sh1;
  wire            asel, oen;
  wire [2:0]      capsel;
  reg sat_t = 0, unf_t = 0, slr_t = 0, ovf_t = 0, slw_t = 0;
  reg running = 0, arm_r = 0, arm_w = 0, en_i = 0, en_q = 0;
  reg [4:0] lmin = 0, lmax = 0;

  axi_bf_ctrl dut (
    .w_bank0(wb0), .w_bank1(wb1), .shift_bank0(sh0), .shift_bank1(sh1),
    .active_sel(asel), .out_en(oen), .capture_sel(capsel),
    .sat_toggle(sat_t), .unf_toggle(unf_t), .slip_r_toggle(slr_t),
    .ovf_toggle(ovf_t), .slip_w_toggle(slw_t),
    .running(running), .armed_r(arm_r), .armed_w(arm_w),
    .level_min(lmin), .level_max(lmax),
    .dac_enable_i(en_i), .dac_enable_q(en_q),
    .s_axi_aclk(clk), .s_axi_aresetn(rstn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  integer errors = 0, k, n;
  reg [31:0] rd;
  initial begin #3000000; $display("WATCHDOG"); $display("RESULT: FAIL"); $finish; end

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
  task chk(input [31:0] got, input [31:0] exp, input [8*48-1:0] name); begin
    if (got !== exp) begin errors = errors + 1;
      $display("  MISMATCH %0s got=0x%08x exp=0x%08x", name, got, exp); end
  end endtask
  task settle; begin repeat (8) @(posedge clk); end endtask
  // n events of one kind from the ~30.72 MHz domain
  task pulse(input integer which, input integer cnt); integer i; begin
    for (i = 0; i < cnt; i = i + 1) begin
      @(posedge sclk);
      case (which)
        0: sat_t = ~sat_t;  1: unf_t = ~unf_t;  2: slr_t = ~slr_t;
        3: ovf_t = ~ovf_t;  4: slw_t = ~slw_t;
      endcase
    end
    settle;
  end endtask

  function [15:0] BA(input integer b, input integer w); BA = 16'h100 + 16'h40*b + 4*w; endfunction

  localparam [15:0] A_ACT = 16'h008, A_OEN = 16'h00C, A_CAP = 16'h010, A_ST = 16'h014,
                    A_SAT = 16'h018, A_UNF = 16'h01C, A_OVF = 16'h020, A_INFO = 16'h024,
                    A_SLIP = 16'h028, A_LVL = 16'h02C;

  initial begin
    repeat (4) @(posedge clk); rstn = 1; repeat (2) @(posedge clk);

    // ---- identity and IDLE declaration ----
    ar(16'h000); chk(rd, 32'h42465734, "ID");
    ar(A_INFO);  chk(rd, 32'h0210080D, "INFO");
    for (k = 2; k <= 11; k = k + 1) if (k != 9) begin ar(4*k); chk(rd, 0, "reset value"); end
    for (k = 0; k < 2; k = k + 1) for (n = 0; n < 16; n = n + 1) begin
      ar(BA(k, n)); chk(rd, 0, "bank reset = 0");
    end
    if (wb0 !== 0 || wb1 !== 0 || sh0 !== 0 || sh1 !== 0 || asel !== 0 || oen !== 0 || capsel !== 0) begin
      errors = errors + 1; $display("  outputs not at IDLE declaration"); end

    // ---- banks: layout, sign extension, 18-bit storage ----
    for (n = 0; n < 8; n = n + 1) aw(BA(1, n), (n & 1) ? -(n*1000 + 7) : (n*1000 + 3));
    aw(BA(1, 8), 32'hFF); aw(BA(1, 9), 32'hDEADBEEF);
    for (n = 0; n < 8; n = n + 1) begin
      ar(BA(1, n)); chk(rd, (n & 1) ? -(n*1000 + 7) : (n*1000 + 3), "bank1 weight r/w");
      if ($signed(wb1[n*18 +: 18]) !== ((n & 1) ? -(n*1000 + 7) : (n*1000 + 3))) begin
        errors = errors + 1; $display("  w_bank1 field %0d wrong on the bus", n); end
    end
    ar(BA(1, 8)); chk(rd, 32'h1F, "SHIFT masked");
    ar(BA(1, 9)); chk(rd, 32'hDEADBEEF, "WEIGHT_SEQ");
    aw(BA(0, 0), 32'h00020000); ar(BA(0, 0)); chk(rd, 32'hFFFE0000, "18-bit storage");
    aw(BA(0, 0), 32'h0001FFFF); ar(BA(0, 0)); chk(rd, 32'h0001FFFF, "W_MAX");

    // ---- no aliasing ----
    aw(BA(0, 10), 32'h12345); ar(BA(0, 10)); chk(rd, 0, "bank word 10 unmapped");
    aw(16'h0180, 32'h12345);  ar(16'h0180);  chk(rd, 0, "beyond bank region");
    aw(16'h4100, 32'h00777);  ar(BA(0, 0));  chk(rd, 32'h0001FFFF, "0x4100 must not alias");
    aw(16'h1140, 32'h00555);  ar(BA(1, 0));  chk(rd, 32'd3, "0x1140 must not alias");

    // ---- controls ----
    aw(A_ACT, 32'hFFFFFFFF); ar(A_ACT); chk(rd, 1, "ACTIVE_SEL [0]");
    aw(A_CAP, 32'hFFFFFFFB); ar(A_CAP); chk(rd, 3, "CAPTURE_SEL [2:0]");
    aw(A_CAP, 0);
    aw(16'h004, 32'hA5A5F00D); ar(16'h004); chk(rd, 32'hA5A5F00D, "SCRATCH");

    // ---- live status ----
    running = 1; arm_r = 1; arm_w = 0; en_i = 1; en_q = 0; lmin = 7; lmax = 9; settle;
    ar(A_ST);  chk(rd, 32'h0000000B, "STATUS live: running, fabric_i, armed");
    ar(A_LVL); chk(rd, 32'h00000907, "LEVEL min 7 max 9");
    arm_r = 0; arm_w = 1; en_q = 1; settle;
    ar(A_ST);  chk(rd, 32'h0000000F, "ARMED from the write side, fabric_q");
    running = 0; arm_w = 0; en_i = 0; en_q = 0; settle;

    // ---- RULE 1: OUT_EN = 0 -> events change nothing ----
    pulse(0, 50); pulse(1, 3); pulse(2, 3); pulse(3, 3); pulse(4, 3);
    ar(A_ST); chk(rd, 0, "OUT_EN=0: no sticky, no FAULTED");
    for (k = 6; k <= 10; k = k + 1) if (k != 9) begin ar(4*k); chk(rd, 0, "OUT_EN=0: counters frozen"); end

    // ---- RULE 2: while live, SAT counts exactly; first fault latches ----
    aw(A_OEN, 1); if (oen !== 1) begin errors = errors + 1; $display("  out_en not 1"); end
    pulse(0, 1000);
    ar(A_SAT); chk(rd, 1000, "SAT_COUNT exact at full sample rate");
    pulse(2, 1);                                   // slip from the read side
    ar(A_ST); chk(rd & 32'hF10, 32'hC10, "SLIP + SAT sticky, FAULTED");
    if (oen !== 0) begin errors = errors + 1; $display("  FAULTED did not force out_en low"); end
    ar(A_OEN); chk(rd, 1, "OUT_EN register itself unchanged (it does not lie)");
    ar(A_SLIP); chk(rd, 1, "SLIP_COUNT = 1");
    // further events while FAULTED: nothing changes
    pulse(1, 2); pulse(3, 2); pulse(4, 2); pulse(0, 20);
    ar(A_ST); chk(rd & 32'hF10, 32'hC10, "while FAULTED: no new sticky");
    ar(A_UNF); chk(rd, 0, "while FAULTED: UNF frozen");
    ar(A_OVF); chk(rd, 0, "while FAULTED: OVF frozen");
    ar(A_SLIP); chk(rd, 1, "while FAULTED: SLIP frozen");
    ar(A_SAT); chk(rd, 1000, "while FAULTED: SAT frozen");

    // ---- RULE 3: FAULTED clears only while OUT_EN = 0 ----
    aw(A_ST, 32'h10); ar(A_ST); chk(rd & 32'h10, 32'h10, "W1C FAULTED refused while OUT_EN=1");
    aw(A_OEN, 0);
    aw(A_ST, 32'h10); ar(A_ST); chk(rd & 32'h10, 0, "W1C FAULTED accepted while OUT_EN=0");
    chk(rd & 32'hF00, 32'hC00, "sticky flags survive a FAULTED clear");
    aw(A_ST, 32'hF00); ar(A_ST); chk(rd, 0, "W1C clears the sticky flags");
    ar(A_SAT); chk(rd, 1000, "W1C does not clear counters");

    // ---- each fault source latches FAULTED and its own counter ----
    aw(A_OEN, 1); pulse(1, 1); ar(A_ST); chk(rd & 32'hF10, 32'h110, "UNF -> FAULTED");
    aw(A_OEN, 0); aw(A_ST, 32'hF10);
    aw(A_OEN, 1); pulse(3, 1); ar(A_ST); chk(rd & 32'hF10, 32'h210, "OVF -> FAULTED");
    aw(A_OEN, 0); aw(A_ST, 32'hF10);
    aw(A_OEN, 1); pulse(4, 1); ar(A_ST); chk(rd & 32'hF10, 32'h810, "write-side SLIP -> FAULTED");
    aw(A_OEN, 0); aw(A_ST, 32'hF10);
    ar(A_UNF); chk(rd, 1, "UNF_COUNT"); ar(A_OVF); chk(rd, 1, "OVF_COUNT");
    ar(A_SLIP); chk(rd, 2, "SLIP_COUNT (read + write side)");

    // ---- reset returns to IDLE declaration ----
    aw(A_OEN, 1); pulse(1, 1);
    rstn = 0; repeat (3) @(posedge clk); rstn = 1; repeat (2) @(posedge clk);
    if (wb0 !== 0 || wb1 !== 0 || sh1 !== 0 || asel !== 0 || oen !== 0 || capsel !== 0) begin
      errors = errors + 1; $display("  reset did not restore IDLE outputs"); end
    ar(A_ST); chk(rd & 32'hF10, 0, "reset clears sticky + FAULTED");

    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
