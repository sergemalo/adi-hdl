`timescale 1ns/1ps
// ============================================================================
// tb_bf_chain — rev 2: axi_bf_ctrl + bf_bank + bf_dac_if + bf_capture with
// three clocks, as in the BD:
//   lclk   125 MHz stand-in for axi_ad9361_0/l_clk
//   sclk   lclk/4, derived synchronously (BUFR stand-in) = divclk
//   aclk   ~94 MHz AXI, unrelated
// A behavioural "core" raises dac_valid every 4 lclk; its rate counter can be
// jumped (driver start_sync) or stalled.
//
// Scenarios (agreed event rules, 2026-10-11):
//   S1  OUT_EN = 0: RX gaps, cadence jumps, DAC stall  -> nothing counted
//   S2  start: ARMING -> RUNNING, steady state, data identity, level band
//   S3  stop: OFF reached, then every disturbance      -> nothing counted
//   S4  restart works
//   S5  1-sample RX gap while RUNNING  -> SLIP, FAULTED, fail-stop, re-arm rules
//   S6  DAC cadence jump while RUNNING -> SLIP
//   S7  DAC stall while RUNNING        -> SLIP
//   S8  SAT_COUNT == clipped samples actually popped to the DAC
//   S9  OUT_EN = 1 with no RX stream   -> ARMING forever, nothing counted
// Prints RESULT: PASS / FAIL.
// ============================================================================
module tb_bf_chain;
  reg lclk = 0; always #4 lclk = ~lclk;
  reg aclk = 0; always #5.3 aclk = ~aclk;
  reg [1:0] dc = 0; always @(posedge lclk) dc <= dc + 1'b1;
  // divclk insertion delay vs l_clk: ~4.9 ns on the board (timing report).
`ifndef SCLK_SKEW
  `define SCLK_SKEW 4.9
`endif
  wire #(`SCLK_SKEW) sclk = dc[1];

  // ------------------------------------------------------------ AXI master
  reg  rstn = 0;
  reg  [15:0] awaddr = 0, araddr = 0;
  reg  awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg  [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid;
  wire [1:0] bresp, rresp;
  wire [31:0] rdata;
  reg  [31:0] rd;
  task aw(input [15:0] a, input [31:0] d); begin
    @(negedge aclk); awaddr = a; wdata = d; awvalid = 1; wvalid = 1; bready = 1;
    @(negedge aclk); while (!bvalid) @(negedge aclk);
    awvalid = 0; wvalid = 0; @(negedge aclk); bready = 0;
  end endtask
  task ar(input [15:0] a); begin
    @(negedge aclk); araddr = a; arvalid = 1; rready = 1;
    @(negedge aclk); while (!rvalid) @(negedge aclk);
    rd = rdata; arvalid = 0; @(negedge aclk); rready = 0;
  end endtask

  localparam [15:0] A_ACT = 16'h008, A_OEN = 16'h00C, A_CAP = 16'h010, A_ST = 16'h014,
                    A_SAT = 16'h018, A_UNF = 16'h01C, A_OVF = 16'h020,
                    A_SLIP = 16'h028, A_LVL = 16'h02C;

  // ---------------------------------------------------------------- DUTs
  wire [8*18-1:0] wb0, wb1; wire [4:0] sh0, sh1;
  wire asel, oen; wire [2:0] capsel;
  wire sat_t, unf_t, slr_t, ovf_t, slw_t, running, arm_r, arm_w;
  wire [4:0] lmin, lmax;

  axi_bf_ctrl u_ctrl (
    .w_bank0(wb0), .w_bank1(wb1), .shift_bank0(sh0), .shift_bank1(sh1),
    .active_sel(asel), .out_en(oen), .capture_sel(capsel),
    .sat_toggle(sat_t), .unf_toggle(unf_t), .slip_r_toggle(slr_t),
    .ovf_toggle(ovf_t), .slip_w_toggle(slw_t),
    .running(running), .armed_r(arm_r), .armed_w(arm_w),
    .level_min(lmin), .level_max(lmax),
    .dac_enable_i(1'b0), .dac_enable_q(1'b0),   // DATA_SEL is argus' business
    .s_axi_aclk(aclk), .s_axi_aresetn(rstn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  reg         vin = 0;
  reg  [15:0] x [0:7];
  wire        bv, bsat;
  wire [15:0] bi, bq;
  bf_bank u_bank (
    .clk(sclk), .valid_in(vin),
    .din_0(x[0]), .din_1(x[1]), .din_2(x[2]), .din_3(x[3]),
    .din_4(x[4]), .din_5(x[5]), .din_6(x[6]), .din_7(x[7]),
    .active_sel(asel), .w_bank0(wb0), .w_bank1(wb1),
    .shift_bank0(sh0), .shift_bank1(sh1),
    .valid_out(bv), .dout_i(bi), .dout_q(bq), .sat_out(bsat));

  reg  dac_valid = 0;
  wire [15:0] di, dq;
  bf_dac_if u_if (
    .clk_s(sclk), .valid_s(bv), .din_i(bi), .din_q(bq), .sat_s(bsat),
    .ovf_toggle(ovf_t), .slip_w_toggle(slw_t), .armed_w(arm_w),
    .clk_d(lclk), .dac_valid(dac_valid), .out_en(oen),
    .dac_data_i(di), .dac_data_q(dq),
    .unf_toggle(unf_t), .slip_r_toggle(slr_t), .sat_toggle(sat_t),
    .running(running), .armed_r(arm_r), .level_min(lmin), .level_max(lmax));

  wire [15:0] co [0:7];
  bf_capture u_cap (
    .clk(sclk),
    .din_0(x[0]), .din_1(x[1]), .din_2(x[2]), .din_3(x[3]),
    .din_4(x[4]), .din_5(x[5]), .din_6(x[6]), .din_7(x[7]),
    .bf_i(bi), .bf_q(bq), .capture_sel(capsel),
    .dout_0(co[0]), .dout_1(co[1]), .dout_2(co[2]), .dout_3(co[3]),
    .dout_4(co[4]), .dout_5(co[5]), .dout_6(co[6]), .dout_7(co[7]));

  // ---------------------------------------------- stimulus: RX samples
  integer seed = 7, l;
  reg big = 0;                        // larger inputs for the SAT scenario
  initial for (l = 0; l < 8; l = l + 1) x[l] = 0;
  always @(negedge sclk) if (vin)
    for (l = 0; l < 8; l = l + 1) x[l] <= big ? ($random(seed) % 30000) : ($random(seed) % 1000);

  // --------------------------------------------- behavioural core: dac_valid
  reg [1:0] rate = 0;
  integer r0;
  initial if ($value$plusargs("rate0=%d", r0)) rate = r0;   // DAC cadence phase
  reg       stall = 0;
  always @(posedge lclk) begin
    rate      <= rate + 1'b1;
    dac_valid <= (rate == 2'd0) && !stall;
  end
  task jump(input [1:0] by); begin @(negedge lclk); rate = rate + by; end endtask

  // ---------------------------------------- scoreboard: FIFO in -> FIFO out
  reg [32:0] q [0:65535];
  integer qw = 0, qr = 0, errors = 0, pops = 0, sat_pops = 0, zero_bad = 0;
  always @(posedge sclk) begin
    if (!u_if.en_s) qw <= 0;
    else if (u_if.do_write) begin q[qw[15:0]] <= {bsat, bq, bi}; qw <= qw + 1; end
  end
  reg [4:0] rbin_before;
  always @(posedge lclk) begin
    rbin_before = u_if.rbin;
    #1;
    if (!u_if.en_d) qr = 0;
    else if (u_if.rbin != rbin_before) begin                    // a pop happened
      if ({dq, di} !== q[qr[15:0]][31:0]) begin
        errors = errors + 1;
        if (errors < 10) $display("  FIFO word %0d: got %h exp %h", qr, {dq, di}, q[qr[15:0]][31:0]);
      end
      if (q[qr[15:0]][32]) sat_pops = sat_pops + 1;
      qr = qr + 1; pops = pops + 1;
    end
    if (!running && {dq, di} !== 32'd0) zero_bad = zero_bad + 1;   // output must be 0 unless RUNNING
  end

  task chk(input [31:0] got, input [31:0] exp, input [8*56-1:0] name); begin
    if (got !== exp) begin errors = errors + 1;
      $display("  MISMATCH %0s got=0x%08x exp=0x%08x", name, got, exp); end
  end endtask
  task samples(input integer n); begin repeat (n) @(posedge sclk); end endtask
  task rx_gap(input integer n); begin @(negedge sclk); vin = 0; samples(n); @(negedge sclk); vin = 1; end endtask
  // n samples' worth of l_clk plus one: always removes at least n dac_valid pulses
  task dac_stall(input integer n); begin @(negedge lclk); stall = 1; repeat (4*n + 1) @(negedge lclk); stall = 0; end endtask
  task disturb; begin rx_gap(1); samples(50); rx_gap(20); samples(50);
                      jump(1); samples(50); jump(2); samples(50); dac_stall(20); samples(50); end endtask
  task expect_counts(input [31:0] sat, input [31:0] unf, input [31:0] ovf, input [31:0] slip,
                     input [8*40-1:0] when); begin
    ar(A_SAT); chk(rd, sat, when); ar(A_UNF); chk(rd, unf, when);
    ar(A_OVF); chk(rd, ovf, when); ar(A_SLIP); chk(rd, slip, when);
  end endtask
  // the agreed STOP / START sequences (TX attenuation and DATA_SEL live in argus)
  integer polls;
  task stop_seq; begin
    aw(A_OEN, 0); polls = 0;
    ar(A_ST); while (rd[3] || rd[0]) begin polls = polls + 1; ar(A_ST); end
  end endtask
  task start_seq; begin
    aw(A_ST, 32'hF10); aw(A_OEN, 1); polls = 0;
    ar(A_ST); while (!rd[0] && !rd[4] && polls < 400) begin polls = polls + 1; ar(A_ST); end
  end endtask

  integer sat0, slip0, p0;
  initial begin #6000000; $display("WATCHDOG"); $display("RESULT: FAIL"); $finish; end

  initial begin
    repeat (4) @(posedge aclk); rstn = 1;
    // bank0: moderate complex weights (S = 17); bank1: saturating (S = 0)
    aw(16'h100, 30000);  aw(16'h104, -12000); aw(16'h108, -25000); aw(16'h10C, 8000);
    aw(16'h110, 17000);  aw(16'h114, 21000);  aw(16'h118, -9000);  aw(16'h11C, -30000);
    aw(16'h120, 17);
    aw(16'h140, 131071); aw(16'h148, -131072); aw(16'h160, 0);
    samples(20);
    @(negedge sclk); vin = 1;

    // S1 -----------------------------------------------------------------
    samples(300); disturb;
    ar(A_ST); chk(rd, 0, "S1 OUT_EN=0: STATUS all clear");
    expect_counts(0, 0, 0, 0, "S1 OUT_EN=0: nothing counted");

    // S2 -----------------------------------------------------------------
    start_seq;
    chk(rd & 32'h19, 32'h09, "S2 start: RUNNING + ARMED, not FAULTED");
    $display("  S2: RUNNING after %0d STATUS polls", polls);
    p0 = pops; samples(3000);
    ar(A_ST); chk(rd, 32'h09, "S2 steady: RUNNING, no sticky");
    expect_counts(0, 0, 0, 0, "S2 steady: nothing counted");
    ar(A_LVL); $display("  S2: level band min %0d max %0d", rd[4:0], rd[12:8]);
    if (rd[4:0] < 6 || rd[12:8] > 10 || rd[12:8] - rd[4:0] > 1) begin
      errors = errors + 1; $display("  level band not within {PRIME-2..PRIME+2} or wider than 1"); end
    if (pops - p0 < 2990) begin errors = errors + 1; $display("  only %0d pops in 3000 samples", pops - p0); end

    // S3 -----------------------------------------------------------------
    stop_seq;
    $display("  S3: OFF after %0d STATUS polls", polls);
    disturb;
    ar(A_ST); chk(rd, 0, "S3 after stop: STATUS all clear");
    expect_counts(0, 0, 0, 0, "S3 after stop: nothing counted");

    // S4 -----------------------------------------------------------------
    start_seq; chk(rd & 32'h19, 32'h09, "S4 restart reaches RUNNING");
    samples(500); ar(A_ST); chk(rd, 32'h09, "S4 restart steady");

    // S5 -----------------------------------------------------------------
    rx_gap(1); samples(20);
    ar(A_ST); chk(rd, 32'h810, "S5 1-sample RX gap: SLIP + FAULTED; fail-stop -> not ARMED, not RUNNING");
    expect_counts(0, 0, 0, 1, "S5 exactly one SLIP");
    disturb; dac_stall(30); samples(50);
    expect_counts(0, 0, 0, 1, "S5 while FAULTED: nothing more counted");
    aw(A_ST, 32'h10); ar(A_ST); chk(rd & 32'h10, 32'h10, "S5 FAULTED not clearable while OUT_EN=1");
    ar(A_OEN); chk(rd, 1, "S5 OUT_EN register unchanged");
    stop_seq; aw(A_ST, 32'h10); ar(A_ST); chk(rd, 32'h800, "S5 after stop + W1C FAULTED: only SLIP sticky");
    start_seq; chk(rd & 32'h19, 32'h09, "S5 re-arm after acknowledge");
    samples(300); ar(A_ST); chk(rd, 32'h09, "S5 running again, clean");

    // S6 -----------------------------------------------------------------
    jump(1); samples(20);
    ar(A_ST); chk(rd, 32'h810, "S6 DAC cadence jump: SLIP + FAULTED");
    expect_counts(0, 0, 0, 2, "S6 SLIP_COUNT = 2");
    stop_seq; aw(A_ST, 32'hF10); start_seq; samples(300);
    ar(A_ST); chk(rd, 32'h09, "S6 recovered");

    // S7 -----------------------------------------------------------------
    dac_stall(1); samples(20);
    ar(A_ST); chk(rd, 32'h810, "S7 DAC stall: SLIP + FAULTED");
    expect_counts(0, 0, 0, 3, "S7 SLIP_COUNT = 3");
    stop_seq; aw(A_ST, 32'hF10); start_seq; samples(300);
    ar(A_ST); chk(rd, 32'h09, "S7 recovered");

    // S8 -----------------------------------------------------------------
    ar(A_SAT); sat0 = rd; p0 = sat_pops;
    big = 1; aw(A_ACT, 1); samples(400); aw(A_ACT, 0); samples(60); big = 0; samples(40);
    ar(A_SAT);
    if (sat_pops - p0 < 50) begin errors = errors + 1; $display("  too few clipped pops (%0d)", sat_pops - p0); end
    chk(rd - sat0, sat_pops - p0, "S8 SAT_COUNT == clipped samples popped");
    ar(A_ST); chk(rd, 32'h409, "S8 SAT sticky, still RUNNING");
    stop_seq; ar(A_SAT); sat0 = rd;
    big = 1; aw(A_ACT, 1); samples(300); aw(A_ACT, 0); big = 0;
    ar(A_SAT); chk(rd, sat0, "S8 SAT frozen while OFF");
    aw(A_ST, 32'hF10);

    // S9 -----------------------------------------------------------------
    @(negedge sclk); vin = 0; samples(30);
    aw(A_OEN, 1); samples(200);
    ar(A_ST); chk(rd, 32'h08, "S9 no RX stream: ARMED only, forever");
    expect_counts(sat0, 0, 0, 3, "S9 nothing counted while ARMING");
    @(negedge sclk); vin = 1; polls = 0;
    ar(A_ST); while (!rd[0] && polls < 400) begin polls = polls + 1; ar(A_ST); end
    chk(rd, 32'h09, "S9 RX starts -> RUNNING");
    samples(300); stop_seq;

    // -------------------------------------------------------------------
    if (zero_bad != 0) begin errors = errors + 1; $display("  %0d non-zero outputs while not RUNNING", zero_bad); end
    $display("  words checked through the FIFO: %0d (clipped %0d)", pops, sat_pops);
    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
