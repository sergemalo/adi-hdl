`timescale 1ns/1ps
// ============================================================================
// tb_bf_chain — B3 integration test: axi_bf_ctrl + bf_bank + bf_dac_if +
// bf_capture with three clocks, as in the BD:
//   lclk   125 MHz stand-in for axi_ad9361_0/l_clk
//   sclk   lclk/4, derived synchronously (BUFR stand-in) = divclk
//   aclk   100 MHz AXI, unrelated
// A behavioural "core" raises dac_valid every 4 lclk (its rate counter can be
// jumped, as the driver's start_sync does).
//
// Checks: priming without flags; every popped word equals the next word
// bf_bank produced (no loss, duplication or reordering through the FIFO);
// TX_SRC = 0 gives zeros while the FIFO keeps running; cadence jumps raise no
// flag; capture mux transparent / substituting; valid loss -> UNF and
// re-prime; DAC stall -> OVF; SAT_COUNT equals the sat pulses exactly.
// Prints RESULT: PASS / FAIL.
// ============================================================================
module tb_bf_chain;
  reg lclk = 0; always #4 lclk = ~lclk;
  reg aclk = 0; always #5.3 aclk = ~aclk;
  reg [1:0] dc = 0; always @(posedge lclk) dc <= dc + 1'b1;
  wire sclk = dc[1];

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

  // ---------------------------------------------------------------- DUTs
  wire [8*18-1:0] wb0, wb1; wire [4:0] sh0, sh1;
  wire asel, txsrc; wire [2:0] capsel;
  wire sat_t, ovf_t, unf_t, primed;
  reg  en_i = 1, en_q = 1;

  axi_bf_ctrl u_ctrl (
    .w_bank0(wb0), .w_bank1(wb1), .shift_bank0(sh0), .shift_bank1(sh1),
    .active_sel(asel), .tx_src(txsrc), .capture_sel(capsel),
    .sat_toggle(sat_t), .ovf_toggle(ovf_t), .unf_toggle(unf_t),
    .fifo_primed(primed), .dac_enable_i(en_i), .dac_enable_q(en_q),
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
    .sat_toggle(sat_t), .ovf_toggle(ovf_t),
    .clk_d(lclk), .dac_valid(dac_valid), .tx_src(txsrc),
    .dac_data_i(di), .dac_data_q(dq), .unf_toggle(unf_t), .primed(primed));

  wire [15:0] co [0:7];
  bf_capture u_cap (
    .clk(sclk),
    .din_0(x[0]), .din_1(x[1]), .din_2(x[2]), .din_3(x[3]),
    .din_4(x[4]), .din_5(x[5]), .din_6(x[6]), .din_7(x[7]),
    .bf_i(bi), .bf_q(bq), .capture_sel(capsel),
    .dout_0(co[0]), .dout_1(co[1]), .dout_2(co[2]), .dout_3(co[3]),
    .dout_4(co[4]), .dout_5(co[5]), .dout_6(co[6]), .dout_7(co[7]));

  // ------------------------------------------------------- stimulus: samples
  integer seed = 7, l;
  initial for (l = 0; l < 8; l = l + 1) x[l] = 0;
  always @(negedge sclk) if (vin) for (l = 0; l < 8; l = l + 1) x[l] <= $random(seed) % 1000;

  // --------------------------------------------- behavioural core: dac_valid
  reg [1:0] rate = 0;
  reg       stall = 0;
  always @(posedge lclk) begin
    rate      <= rate + 1'b1;
    dac_valid <= (rate == 2'd0) && !stall;
  end
  task jump(input [1:0] by); begin @(negedge lclk); rate = rate + by; end endtask

  // ---------------------------------------- scoreboard: FIFO in -> FIFO out
  reg [31:0] q [0:65535];
  integer qw = 0, qr = 0, errors = 0, sat_pulses = 0, popped_n = 0;
  always @(posedge sclk) if (bv) begin
    if (!u_if.full) begin q[qw[15:0]] <= {bq, bi}; qw <= qw + 1; end
    if (bsat) sat_pulses <= sat_pulses + 1;
  end
  // decide at the edge what the FIFO must do, check right after it
  reg pop_exp, src_exp; reg [31:0] word_exp;
  always @(posedge lclk) begin
    pop_exp  = dac_valid && u_if.go && (u_if.level != 0);
    src_exp  = u_if.src;
    word_exp = q[qr[15:0]];
    if (dac_valid) begin
      #1;
      if (pop_exp) begin
        if ({dq, di} !== (src_exp ? word_exp : 32'd0)) begin
          errors = errors + 1;
          if (errors < 10) $display("  FIFO word %0d: got %h exp %h (src %0d)", qr, {dq, di}, word_exp, src_exp);
        end
        qr = qr + 1; popped_n = popped_n + 1;
      end else if ({dq, di} !== 32'd0) begin
        errors = errors + 1; $display("  non-zero output without a pop");
      end
    end
  end

  task chk(input [31:0] got, input [31:0] exp, input [8*48-1:0] name); begin
    if (got !== exp) begin errors = errors + 1;
      $display("  MISMATCH %0s got=0x%08x exp=0x%08x", name, got, exp); end
  end endtask
  task samples(input integer n); begin repeat (n) @(posedge sclk); end endtask

  integer k, sat0, zeros, nz;
  initial begin #3000000; $display("WATCHDOG"); $display("RESULT: FAIL"); $finish; end

  initial begin
    repeat (4) @(posedge aclk); rstn = 1;
    // bank0: moderate complex weights (S = 17), all four channels used
    aw(16'h100, 30000);  aw(16'h104, -12000); aw(16'h108, -25000); aw(16'h10C, 8000);
    aw(16'h110, 17000);  aw(16'h114, 21000);  aw(16'h118, -9000);  aw(16'h11C, -30000);
    aw(16'h120, 17);
    aw(16'h00C, 1);                                     // TX_SRC = BF
    samples(20);
    @(negedge sclk); vin = 1;

    // 1. priming, no flags, data identity
    samples(400);
    ar(16'h014); chk(rd & 32'h701, 32'h001, "primed, no sticky flags after start");
    ar(16'h01C); chk(rd, 0, "UNF_COUNT = 0 after start");
    ar(16'h020); chk(rd, 0, "OVF_COUNT = 0 after start");
    if (popped_n < 300) begin errors = errors + 1; $display("  only %0d words popped", popped_n); end

    // 2. TX_SRC = 0: zeros, FIFO still running
    aw(16'h00C, 0); samples(10);
    zeros = 0; nz = 0;
    for (k = 0; k < 200; k = k + 1) begin @(posedge dac_valid); #2; if ({dq,di} == 0) zeros = zeros + 1; else nz = nz + 1; end
    if (nz != 0) begin errors = errors + 1; $display("  TX_SRC=0 but %0d non-zero outputs", nz); end
    aw(16'h00C, 1); samples(10);
    nz = 0;
    for (k = 0; k < 100; k = k + 1) begin @(posedge dac_valid); #2; if ({dq,di} != 0) nz = nz + 1; end
    if (nz < 90) begin errors = errors + 1; $display("  TX_SRC=1 again but only %0d non-zero", nz); end

    // 3. cadence jumps (driver start_sync) raise no flag
    for (k = 1; k <= 3; k = k + 1) begin jump(k); samples(100); end
    ar(16'h014); chk(rd & 32'h700, 0, "no sticky after cadence jumps");

    // 4. capture mux
    aw(16'h010, 3); samples(6);
    for (k = 0; k < 50; k = k + 1) begin
      @(negedge sclk);
      if (co[4] !== bi || co[5] !== bq || co[0] !== x[0] || co[3] !== x[3] || co[7] !== x[7]) begin
        errors = errors + 1; $display("  capture_sel=3 lanes wrong"); k = 50; end
    end
    aw(16'h010, 0); samples(6);
    for (k = 0; k < 50; k = k + 1) begin
      @(negedge sclk);
      if (co[4] !== x[4] || co[5] !== x[5]) begin errors = errors + 1; $display("  capture_sel=0 not transparent"); k = 50; end
    end

    // 5. valid loss -> underflow, re-prime
    @(negedge sclk); vin = 0; samples(40); @(negedge sclk); vin = 1; samples(200);
    ar(16'h014); chk(rd & 32'h101, 32'h101, "UNF sticky set, primed again");
    ar(16'h01C); if (rd == 0) begin errors = errors + 1; $display("  UNF_COUNT not incremented"); end
    aw(16'h014, 32'h700);

    // 6. DAC stall -> overflow
    @(negedge lclk); stall = 1; samples(40); @(negedge lclk); stall = 0; samples(200);
    ar(16'h014); chk(rd & 32'h200, 32'h200, "OVF sticky set");
    ar(16'h020); if (rd == 0) begin errors = errors + 1; $display("  OVF_COUNT not incremented"); end
    aw(16'h014, 32'h700);

    // 7. saturation count is exact
    ar(16'h018); sat0 = rd; k = sat_pulses;
    aw(16'h140, 131071); aw(16'h148, -131072); aw(16'h160, 0);   // bank1: huge, S = 0
    aw(16'h008, 1); samples(300); aw(16'h008, 0); samples(40);
    ar(16'h018);
    if (sat_pulses - k < 100) begin errors = errors + 1; $display("  too few sat pulses (%0d)", sat_pulses - k); end
    chk(rd - sat0, sat_pulses - k, "SAT_COUNT == sat pulses");
    aw(16'h014, 32'h700);

    // 8. long steady state: nothing may fire
    samples(3000);
    ar(16'h014); chk(rd & 32'h701, 32'h001, "steady state: primed, no flags");

    $display("  words checked through the FIFO: %0d", popped_n);
    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
