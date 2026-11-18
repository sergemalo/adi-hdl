// ARGUS-91 step 6 (b), TX: do util_upack2 + util_rfifo lose or insert samples
// at the 2R2T clock ratio?
//
// As in fmcomms5_bd.tcl: the TX DMA's m_axis (divclk) feeds util_upack2
// (8 lanes x 16 bits), which util_rfifo pulls from (din_clk = divclk) to feed
// the DAC (dout_clk = l_clk, one sample every 4 l_clk in 2R2T). Read bandwidth
// equals write bandwidth here too.
//
// Stimulus: an always-valid m_axis (an ideal DMA), each beat 8 lanes tagged
// {lane + 1, k}, k counting beats (12 bits, wraps). The DAC channels are
// enabled (DATA_SEL = DMA) after reset, as a buffer enable does. Checker on
// l_clk: at every dac_valid, rfifo's output must carry k + 1 on all 8 lanes.
// A lost, repeated, stale or zero sample is a MISMATCH. After lock,
// util_upack2's underflow must stay low: PASS needs both.
//
// util_rfifo writes only words upack2 marks valid, and pulls at exactly the
// DAC rate, so a DMA stall delays the writer for good. Its 32-word memory
// absorbs up to 15 beats in total over a run (underflow set, DAC stream
// intact); from 16, the DAC replays words 32 samples old and the stream slips.
//
// Sweeps (plusargs): +skew=<ns> delays divclk, +phase=<0..PERIOD-1> moves the
// DAC cadence relative to the divider.
//
// Negative controls (xvlog -d):
//   NEG_FAST   one DAC read every 3 l_clk: reads outrun the refill, must FAIL
//   NEG_STALL  the source drops valid for STALL_LEN divclk once (default 3):
//              must FAIL; below 16 only through the underflow count
//   NEG_STALL2 two such stalls, 1000 beats apart: their lengths add up
//   NEG_SKIP   the source skips one k once: checker must FAIL

`timescale 1ns/100ps

module tb_rfifo_ratio;

`ifdef NEG_FAST
  localparam PERIOD = 3;
`else
  localparam PERIOD = 4;
`endif
  localparam N_CHECK = 20000;
  localparam K_BAD = 12'd1000;
`ifndef STALL_LEN
  `define STALL_LEN 3
`endif

  real    skew = 0.0;
  integer phase = 0;

  reg l_clk = 1'b0;
  always #2 l_clk = ~l_clk;

  wire div_raw;
  reg  divclk = 1'b0;

  // SEL_*_DIV as integers: the module defaults are strings ("4"), which the
  // BUFGCE_DIV sim model reads as 52. Same ratios as the IP (/4, /2).
  util_clkdiv #(.SIM_DEVICE("ULTRASCALE"), .SEL_0_DIV(4), .SEL_1_DIV(2)) i_clkdiv (
    .clk (l_clk),
    .clk_sel (1'b0),
    .clk_out (div_raw));

  always @(div_raw) divclk <= #(skew) div_raw;

  reg dout_rst = 1'b1;     // axi_ad9361 rst (l_clk)
  reg div_rstn = 1'b0;     // proc_sys_reset (divclk)
  reg dac_en = 1'b0;       // DATA_SEL = DMA on all channels (l_clk)
  initial begin
    if (!$value$plusargs("skew=%f", skew)) skew = 0.0;
    if (!$value$plusargs("phase=%d", phase)) phase = 0;
    #300; @(posedge l_clk) dout_rst <= 1'b0;
    #40;  @(posedge divclk) div_rstn <= 1'b1;
    #400; @(posedge l_clk) dac_en <= 1'b1;
  end

  // DAC cadence: axi_ad9361_tx dac_valid, one every PERIOD l_clk
  reg [3:0] cnt = 4'd0;
  reg       dac_valid = 1'b0;
  always @(posedge l_clk) begin
    if (dout_rst) begin
      cnt <= phase;
      dac_valid <= 1'b0;
    end else begin
      cnt <= (cnt == PERIOD - 1) ? 4'd0 : cnt + 4'd1;
      dac_valid <= (cnt == 0);
    end
  end

  function [15:0] tag(input integer lane, input [11:0] kk);
    tag = {lane[3:0] + 4'd1, kk};
  endfunction

  // ideal DMA on m_axis
  reg  [11:0]  k = 12'd0;
  reg          bad_done = 1'b0;
  integer      beat = 0;           // accepted beats, no wrap
  integer      n_stalls = 0;
  integer      stall = 0;
  wire         s_axis_ready;
  wire         s_axis_valid = (stall == 0);
  wire [127:0] s_axis_data = {tag(7, k), tag(6, k), tag(5, k), tag(4, k),
                              tag(3, k), tag(2, k), tag(1, k), tag(0, k)};

  always @(posedge divclk) begin
    if (stall > 0) stall <= stall - 1;
    if (s_axis_valid && s_axis_ready) begin
      beat <= beat + 1;
`ifdef NEG_STALL
      if (beat == 1000 && n_stalls == 0) begin stall <= `STALL_LEN; n_stalls <= 1; end
`endif
`ifdef NEG_STALL2
      if ((beat == 1000 && n_stalls == 0) || (beat == 2000 && n_stalls == 1)) begin
        stall <= `STALL_LEN; n_stalls <= n_stalls + 1;
      end
`endif
`ifdef NEG_SKIP
      if (k == K_BAD && !bad_done) begin k <= k + 12'd2; bad_done <= 1'b1; end else
`endif
      k <= k + 12'd1;
    end
  end

  // util_upack2 <-> util_rfifo
  wire [7:0]  up_enable;
  wire        fifo_rd_en;
  wire        fifo_rd_valid;
  wire        fifo_rd_underflow;
  wire [15:0] fifo_rd_data [0:7];
  wire [15:0] dout_data [0:7];
  wire [7:0]  dout_valid_out;
  wire        dout_unf;

  util_upack2 #(
    .NUM_OF_CHANNELS (8),
    .SAMPLE_DATA_WIDTH (16)
  ) i_upack (
    .clk (divclk),
    .reset (~div_rstn),
    .enable_0 (up_enable[0]), .enable_1 (up_enable[1]), .enable_2 (up_enable[2]), .enable_3 (up_enable[3]),
    .enable_4 (up_enable[4]), .enable_5 (up_enable[5]), .enable_6 (up_enable[6]), .enable_7 (up_enable[7]),
    .fifo_rd_en (fifo_rd_en),
    .fifo_rd_valid (fifo_rd_valid),
    .fifo_rd_underflow (fifo_rd_underflow),
    .fifo_rd_data_0 (fifo_rd_data[0]), .fifo_rd_data_1 (fifo_rd_data[1]),
    .fifo_rd_data_2 (fifo_rd_data[2]), .fifo_rd_data_3 (fifo_rd_data[3]),
    .fifo_rd_data_4 (fifo_rd_data[4]), .fifo_rd_data_5 (fifo_rd_data[5]),
    .fifo_rd_data_6 (fifo_rd_data[6]), .fifo_rd_data_7 (fifo_rd_data[7]),
    .s_axis_valid (s_axis_valid),
    .s_axis_ready (s_axis_ready),
    .s_axis_data (s_axis_data));

  util_rfifo #(
    .NUM_OF_CHANNELS (8),
    .DIN_DATA_WIDTH (16),
    .DOUT_DATA_WIDTH (16),
    .DIN_ADDRESS_WIDTH (4)
  ) dut (
    .din_rstn (div_rstn), .din_clk (divclk),
    .din_enable_0 (up_enable[0]), .din_valid_0 (fifo_rd_en),  .din_valid_in_0 (fifo_rd_valid), .din_data_0 (fifo_rd_data[0]),
    .din_enable_1 (up_enable[1]), .din_valid_1 (),            .din_valid_in_1 (fifo_rd_valid), .din_data_1 (fifo_rd_data[1]),
    .din_enable_2 (up_enable[2]), .din_valid_2 (),            .din_valid_in_2 (fifo_rd_valid), .din_data_2 (fifo_rd_data[2]),
    .din_enable_3 (up_enable[3]), .din_valid_3 (),            .din_valid_in_3 (fifo_rd_valid), .din_data_3 (fifo_rd_data[3]),
    .din_enable_4 (up_enable[4]), .din_valid_4 (),            .din_valid_in_4 (fifo_rd_valid), .din_data_4 (fifo_rd_data[4]),
    .din_enable_5 (up_enable[5]), .din_valid_5 (),            .din_valid_in_5 (fifo_rd_valid), .din_data_5 (fifo_rd_data[5]),
    .din_enable_6 (up_enable[6]), .din_valid_6 (),            .din_valid_in_6 (fifo_rd_valid), .din_data_6 (fifo_rd_data[6]),
    .din_enable_7 (up_enable[7]), .din_valid_7 (),            .din_valid_in_7 (fifo_rd_valid), .din_data_7 (fifo_rd_data[7]),
    .din_unf (fifo_rd_underflow),
    .dout_rst (dout_rst), .dout_clk (l_clk),
    .dout_enable_0 (dac_en), .dout_valid_0 (dac_valid), .dout_valid_out_0 (dout_valid_out[0]), .dout_data_0 (dout_data[0]),
    .dout_enable_1 (dac_en), .dout_valid_1 (dac_valid), .dout_valid_out_1 (dout_valid_out[1]), .dout_data_1 (dout_data[1]),
    .dout_enable_2 (dac_en), .dout_valid_2 (dac_valid), .dout_valid_out_2 (dout_valid_out[2]), .dout_data_2 (dout_data[2]),
    .dout_enable_3 (dac_en), .dout_valid_3 (dac_valid), .dout_valid_out_3 (dout_valid_out[3]), .dout_data_3 (dout_data[3]),
    .dout_enable_4 (dac_en), .dout_valid_4 (dac_valid), .dout_valid_out_4 (dout_valid_out[4]), .dout_data_4 (dout_data[4]),
    .dout_enable_5 (dac_en), .dout_valid_5 (dac_valid), .dout_valid_out_5 (dout_valid_out[5]), .dout_data_5 (dout_data[5]),
    .dout_enable_6 (dac_en), .dout_valid_6 (dac_valid), .dout_valid_out_6 (dout_valid_out[6]), .dout_data_6 (dout_data[6]),
    .dout_enable_7 (dac_en), .dout_valid_7 (dac_valid), .dout_valid_out_7 (dout_valid_out[7]), .dout_data_7 (dout_data[7]),
    .dout_unf (dout_unf));

  // checker on l_clk. rfifo registers its output on dac_valid, so the value
  // seen at a dac_valid is the one the DAC takes; it is checked for continuity.
  integer    n_checked = 0;
  integer    n_err = 0;
  integer    n_unf = 0;
  integer    n_lead_zero = 0;
  reg        locked = 1'b0;
  reg [11:0] exp_k = 12'd0;
  reg [11:0] first_k = 12'd0;
  integer    lane;

  always @(posedge l_clk) begin
    if (!dout_rst && dac_en && dac_valid) begin
      if (!locked && dout_data[0] == 16'd0) begin
        n_lead_zero = n_lead_zero + 1;          // before the first DMA word
      end else begin
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
          $display("skew %0.1f ns, phase %0d, period %0d l_clk: %0d samples checked, first k %0d after %0d zeros, %0d lane errors, %0d underflow cycles",
                   skew, phase, PERIOD, n_checked, first_k, n_lead_zero, n_err, n_unf);
          $display("RESULT: %s", (n_err == 0 && n_unf == 0) ? "PASS" : "FAIL");
          $finish;
        end
      end
    end
  end

  // underflow after lock (divclk domain)
  always @(posedge divclk)
    if (locked && fifo_rd_underflow) n_unf = n_unf + 1;

  initial begin
    #(2000 + N_CHECK * 4 * PERIOD * 2);
    $display("STALLED: %0d of %0d samples", n_checked, N_CHECK);
    $display("RESULT: FAIL");
    $finish;
  end

endmodule
