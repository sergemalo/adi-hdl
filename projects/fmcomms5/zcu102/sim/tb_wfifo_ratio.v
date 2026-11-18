// ARGUS-91 step 6 (b), RX: does util_wfifo lose samples at the 2R2T clock ratio?
//
// The block design runs util_wfifo with din_clk = l_clk and dout_clk = divclk,
// divclk = l_clk / 4 through util_clkdiv (BUFGCE_DIV on UltraScale+), and one
// sample per channel every 4 l_clk (2R2T). Its read bandwidth then equals its
// write bandwidth, which its own comment says "will NOT work".
//
// Stimulus: 8 lanes, each sample tagged {lane + 1, k}; k counts samples
// (12 bits, wraps). Checker on divclk: every dout_valid must carry k + 1 on all
// 8 lanes. A lost, repeated or reordered sample is a MISMATCH.
//
// Sweeps (plusargs, one elaboration): +skew=<ns> delays divclk after the
// BUFGCE_DIV (routing skew), +phase=<0..PERIOD-1> moves the sample cadence
// relative to the divider.
//
// Negative controls (xvlog -d):
//   NEG_FAST   one sample every 3 l_clk: writes outrun reads, must FAIL
//   NEG_SKIP   source skips one k once: checker must FAIL
//   NEG_DUP    source repeats one k once: checker must FAIL

`timescale 1ns/100ps

module tb_wfifo_ratio;

`ifdef NEG_FAST
  localparam PERIOD = 3;
`else
  localparam PERIOD = 4;
`endif
  localparam N_CHECK = 20000;
  localparam K_BAD = 12'd1000;

  real    skew = 0.0;
  integer phase = 0;

  // l_clk: 245.76 MHz at 61.44 MSPS; 4 ns is close enough, only the ratio counts.
  reg l_clk = 1'b0;
  always #2 l_clk = ~l_clk;

  wire div_raw;
  reg  divclk = 1'b0;

  // SEL_*_DIV as integers: the module defaults are strings ("4"), which the
  // BUFGCE_DIV sim model reads as 52. Same ratios as the IP (/4, /2).
  util_clkdiv #(.SIM_DEVICE("ULTRASCALE"), .SEL_0_DIV(4), .SEL_1_DIV(2)) i_clkdiv (
    .clk (l_clk),
    .clk_sel (1'b0),      // 2R2T: all *_r1_mode = 0, divide by 4
    .clk_out (div_raw));

  always @(div_raw) divclk <= #(skew) div_raw;

  // resets: axi_ad9361 rst (l_clk, active high), proc_sys_reset (divclk, active low)
  reg din_rst = 1'b1;
  reg dout_rstn = 1'b0;
  initial begin
    if (!$value$plusargs("skew=%f", skew)) skew = 0.0;
    if (!$value$plusargs("phase=%d", phase)) phase = 0;
    #300; @(posedge l_clk) din_rst <= 1'b0;
    #40;  @(posedge divclk) dout_rstn <= 1'b1;
  end

  // source: one sample every PERIOD l_clk, as axi_ad9361 adc_valid in 2R2T
  reg [3:0]  cnt = 4'd0;
  reg        din_valid = 1'b0;
  reg [11:0] k = 12'd0;
  reg        bad_done = 1'b0;

  always @(posedge l_clk) begin
    if (din_rst) begin
      cnt <= phase;
      din_valid <= 1'b0;
    end else begin
      cnt <= (cnt == PERIOD - 1) ? 4'd0 : cnt + 4'd1;
      din_valid <= (cnt == 0);
    end
    if (din_valid) begin
`ifdef NEG_SKIP
      if (k == K_BAD && !bad_done) begin k <= k + 12'd2; bad_done <= 1'b1; end else
`endif
`ifdef NEG_DUP
      if (k == K_BAD && !bad_done) begin k <= k; bad_done <= 1'b1; end else
`endif
      k <= k + 12'd1;
    end
  end

  function [15:0] tag(input integer lane, input [11:0] kk);
    tag = {lane[3:0] + 4'd1, kk};
  endfunction

  wire [15:0] dout_data [0:7];
  wire [7:0]  dout_valid;
  wire [7:0]  dout_enable;
  wire        din_ovf;

  util_wfifo #(
    .NUM_OF_CHANNELS (8),
    .DIN_DATA_WIDTH (16),
    .DOUT_DATA_WIDTH (16),
    .DIN_ADDRESS_WIDTH (4)
  ) dut (
    .din_rst (din_rst), .din_clk (l_clk),
    .din_enable_0 (1'b1), .din_valid_0 (din_valid), .din_data_0 (tag(0, k)),
    .din_enable_1 (1'b1), .din_valid_1 (din_valid), .din_data_1 (tag(1, k)),
    .din_enable_2 (1'b1), .din_valid_2 (din_valid), .din_data_2 (tag(2, k)),
    .din_enable_3 (1'b1), .din_valid_3 (din_valid), .din_data_3 (tag(3, k)),
    .din_enable_4 (1'b1), .din_valid_4 (din_valid), .din_data_4 (tag(4, k)),
    .din_enable_5 (1'b1), .din_valid_5 (din_valid), .din_data_5 (tag(5, k)),
    .din_enable_6 (1'b1), .din_valid_6 (din_valid), .din_data_6 (tag(6, k)),
    .din_enable_7 (1'b1), .din_valid_7 (din_valid), .din_data_7 (tag(7, k)),
    .din_ovf (din_ovf),
    .dout_rstn (dout_rstn), .dout_clk (divclk),
    .dout_enable_0 (dout_enable[0]), .dout_valid_0 (dout_valid[0]), .dout_data_0 (dout_data[0]),
    .dout_enable_1 (dout_enable[1]), .dout_valid_1 (dout_valid[1]), .dout_data_1 (dout_data[1]),
    .dout_enable_2 (dout_enable[2]), .dout_valid_2 (dout_valid[2]), .dout_data_2 (dout_data[2]),
    .dout_enable_3 (dout_enable[3]), .dout_valid_3 (dout_valid[3]), .dout_data_3 (dout_data[3]),
    .dout_enable_4 (dout_enable[4]), .dout_valid_4 (dout_valid[4]), .dout_data_4 (dout_data[4]),
    .dout_enable_5 (dout_enable[5]), .dout_valid_5 (dout_valid[5]), .dout_data_5 (dout_data[5]),
    .dout_enable_6 (dout_enable[6]), .dout_valid_6 (dout_valid[6]), .dout_data_6 (dout_data[6]),
    .dout_enable_7 (dout_enable[7]), .dout_valid_7 (dout_valid[7]), .dout_data_7 (dout_data[7]),
    .dout_ovf (1'b0));

  // checker on divclk
  integer    n_checked = 0;
  integer    n_err = 0;
  reg        locked = 1'b0;
  reg [11:0] exp_k = 12'd0;
  reg [11:0] first_k = 12'd0;
  integer    lane;

  always @(posedge divclk) begin
    if (dout_rstn && dout_valid[0]) begin
      if (!locked) begin
        locked = 1'b1;
        first_k = dout_data[0][11:0];
        exp_k = dout_data[0][11:0];
      end
      for (lane = 0; lane < 8; lane = lane + 1)
        if (dout_data[lane] !== tag(lane, exp_k)) begin
          n_err = n_err + 1;
          if (n_err <= 5)
            $display("MISMATCH t=%0t sample %0d lane %0d: got %h, expected %h",
                     $time, n_checked, lane, dout_data[lane], tag(lane, exp_k));
        end
      exp_k = exp_k + 12'd1;
      n_checked = n_checked + 1;
      if (n_checked == N_CHECK) begin
        $display("skew %0.1f ns, phase %0d, period %0d l_clk: %0d samples checked, first k %0d, %0d lane errors",
                 skew, phase, PERIOD, n_checked, first_k, n_err);
        $display("RESULT: %s", (n_err == 0) ? "PASS" : "FAIL");
        $finish;
      end
    end
  end

  // watchdog: the stream must deliver N_CHECK samples
  initial begin
    #(1000 + N_CHECK * 4 * PERIOD * 2);
    $display("STALLED: %0d of %0d samples", n_checked, N_CHECK);
    $display("RESULT: FAIL");
    $finish;
  end

endmodule
