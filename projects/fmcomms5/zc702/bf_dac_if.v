`timescale 1ns/1ps
// ============================================================================
// bf_dac_if — BFW4 output to the AD9361 TX2 input (chip A, TX2A_A), rev 2
//
// A 16-deep FIFO between bf_bank (divclk) and axi_ad9361_0's dac_valid_i1
// requests (l_clk), with an explicit, fail-stop output state machine.
//
//   OFF       out_en = 0. Both pointers held at 0, nothing written, nothing
//             read, output 0. NO EVENT CAN BE GENERATED.
//   ARMING    out_en = 1. Write side pushes every valid sample. Read side pops
//             nothing, outputs 0, and learns the dac_valid period. No events.
//   RUNNING   entered once the FIFO holds PRIME (8) words AND the dac_valid
//             period has been seen twice in a row. One pop per dac_valid.
//   FAULTED   any violation below while RUNNING. Output 0, no more pops or
//             pushes, no more events. Left only through out_en = 0 (OFF).
//
// WHY RUNNING IS DETERMINISTIC (design note §11). Both sides derive from the
// same l_clk: divclk = l_clk/4, dac_valid = every 4th l_clk. Rates are equal
// exactly, so the level cannot drift. The invariants are checked DIRECTLY in
// each clock domain, with no CDC in the check itself:
//   write side   one valid sample on EVERY divclk cycle once RUNNING
//                (util_wfifo hands over 8-word bursts back to back) -> a gap
//                is an RX-stream interruption                       => SLIP
//   read side    dac_valid exactly every PERIOD l_clk (learned while
//                ARMING) -> a shorter/longer spacing is a cadence change
//                (driver start_sync, rate change)                    => SLIP
//   backstops    level outside [PRIME-WIN, PRIME+WIN]               => SLIP
//                empty at a pop                                      => UNF
//                full at a push                                      => OVF
//
// Events are TOGGLES (one flip per event) for axi_bf_ctrl, which counts and
// latches them only while its OUT_EN register is 1 and FAULTED is clear.
// SAT is counted at POP time from a bit stored with each word: it counts
// clipped samples actually handed to the DAC.
//
// CDC: gray pointers, enables and fault bits through 2-FF synchronisers named
// bfx_m1_* / bfx_m2_* (XDC false-paths the m1 stage). Memory is distributed
// RAM read into rd_word in l_clk (XDC set_max_delay -datapath_only on
// mem -> rd_word; divclk arrives ~4.9 ns after l_clk, see note §10).
// No reset port: power-up state via `initial`.
// ============================================================================
module bf_dac_if #(
  parameter integer AW    = 4,        // depth 2^AW = 16
  parameter integer PRIME = 8,
  parameter integer WIN   = 2
)(
  // write side, divclk
  input  wire        clk_s,
  input  wire        valid_s,
  input  wire [15:0] din_i,
  input  wire [15:0] din_q,
  input  wire        sat_s,           // bf_bank sat_out (gated by valid)
  output reg         ovf_toggle    = 1'b0,
  output reg         slip_w_toggle = 1'b0,
  output wire        armed_w,

  // read side, l_clk
  input  wire        clk_d,
  input  wire        dac_valid,       // axi_ad9361_0/dac_valid_i1
  input  wire        out_en,          // async, from axi_bf_ctrl (OUT_EN & ~FAULTED)
  output wire [15:0] dac_data_i,
  output wire [15:0] dac_data_q,
  output reg         unf_toggle    = 1'b0,
  output reg         slip_r_toggle = 1'b0,
  output reg         sat_toggle    = 1'b0,
  output wire        running,
  output wire        armed_r,
  output reg  [AW:0] level_min = 0,   // since entering RUNNING
  output reg  [AW:0] level_max = 0
);
  localparam integer DEPTH = 1 << AW;
  localparam integer W     = 33;      // {sat, q[15:0], i[15:0]}

  function [AW:0] bin2gray; input [AW:0] b; bin2gray = b ^ (b >> 1); endfunction
  function [AW:0] gray2bin;
    input [AW:0] g; integer k;
    begin
      gray2bin[AW] = g[AW];
      for (k = AW - 1; k >= 0; k = k - 1) gray2bin[k] = gray2bin[k+1] ^ g[k];
    end
  endfunction

  reg [W-1:0] mem [0:DEPTH-1];
  integer m;
  initial for (m = 0; m < DEPTH; m = m + 1) mem[m] = {W{1'b0}};

  // read-side state, declared early (the write side synchronises some of it)
  localparam [1:0] ST_OFF = 2'd0, ST_ARM = 2'd1, ST_RUN = 2'd2, ST_FLT = 2'd3;
  reg  [1:0]  st      = ST_OFF;
  reg  [AW:0] rbin    = 0, rgray = 0;
  reg         fault_r = 1'b0;

  // ================================================================ write side
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_en_s  = 1'b0, bfx_m2_en_s  = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_run_s = 1'b0, bfx_m2_run_s = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_fr_s  = 1'b0, bfx_m2_fr_s  = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m1_rgray = 0,    bfx_m2_rgray = 0;

  reg  [AW:0] wbin = 0, wgray = 0;
  reg         fault_w = 1'b0;
  wire        en_s    = bfx_m2_en_s;
  wire [AW:0] rbin_s  = gray2bin(bfx_m2_rgray);
  wire        full    = ((wbin - rbin_s) == DEPTH);
  wire        stop_w  = fault_w | bfx_m2_fr_s;
  wire        do_write = en_s & valid_s & ~stop_w & ~full;

  always @(posedge clk_s) begin
    bfx_m1_en_s  <= out_en;        bfx_m2_en_s  <= bfx_m1_en_s;
    bfx_m1_run_s <= (st == ST_RUN); bfx_m2_run_s <= bfx_m1_run_s;
    bfx_m1_fr_s  <= fault_r;       bfx_m2_fr_s  <= bfx_m1_fr_s;
    bfx_m1_rgray <= rgray;         bfx_m2_rgray <= bfx_m1_rgray;

    if (!en_s) begin
      wbin <= 0; wgray <= 0; fault_w <= 1'b0;
    end else begin
      if (do_write) begin
`ifdef MUT_FIFO_IQ
        mem[wbin[AW-1:0]] <= {sat_s, din_i, din_q};   // fault: I/Q swapped
`else
        mem[wbin[AW-1:0]] <= {sat_s, din_q, din_i};
`endif
        wbin  <= wbin + 1'b1;
        wgray <= bin2gray(wbin + 1'b1);
      end
      if (!stop_w) begin
        if (valid_s && full) begin
          ovf_toggle <= ~ovf_toggle;  fault_w <= 1'b1;
        end
`ifndef MUT_NO_GAP_CHECK
        else if (bfx_m2_run_s && !valid_s) begin       // RX stream gap
          slip_w_toggle <= ~slip_w_toggle;  fault_w <= 1'b1;
        end
`endif
      end
    end
  end
  assign armed_w = en_s;

  // ================================================================= read side
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_en_d  = 1'b0, bfx_m2_en_d  = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        bfx_m1_fw_d  = 1'b0, bfx_m2_fw_d  = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg [AW:0] bfx_m1_wgray = 0,    bfx_m2_wgray = 0;

  wire        en_d   = bfx_m2_en_d;
  wire [AW:0] wbin_d = gray2bin(bfx_m2_wgray);
  wire [AW:0] level  = wbin_d - rbin;
  wire [W-1:0] rword = mem[rbin[AW-1:0]];

  reg  [5:0]  since  = 6'd0;           // l_clk since the last dac_valid
  reg  [5:0]  period = 6'd0;           // learned dac_valid spacing
  reg  [1:0]  nper   = 2'd0;           // 0 none, 1 one pulse, 2 one spacing, 3 confirmed
  reg  [W-1:0] rd_word = {W{1'b0}};
  reg         pop_d  = 1'b0;
  wire [5:0]  spacing = since + 1'b1;

  localparam [AW:0] LO = PRIME - WIN;
  localparam [AW:0] HI = PRIME + WIN;
`ifdef MUT_NO_PRIME
  wire primed_ok = 1'b1;                                // fault: no priming
`else
  wire primed_ok = (level >= PRIME);
`endif
`ifdef MUT_NO_PERIOD_CHECK
  wire bad_cadence = 1'b0;                              // fault: cadence unchecked
`else
  wire bad_cadence = dac_valid ? (spacing != period) : (spacing == period);
`endif

  always @(posedge clk_d) begin
    bfx_m1_en_d  <= out_en;   bfx_m2_en_d  <= bfx_m1_en_d;
    bfx_m1_fw_d  <= fault_w;  bfx_m2_fw_d  <= bfx_m1_fw_d;
    bfx_m1_wgray <= wgray;    bfx_m2_wgray <= bfx_m1_wgray;

    since <= dac_valid ? 6'd0 : ((since == 6'd63) ? since : since + 1'b1);
    pop_d <= 1'b0;

    if (!en_d) begin
      st <= ST_OFF; rbin <= 0; rgray <= 0; rd_word <= {W{1'b0}};
      nper <= 2'd0; period <= 6'd0; fault_r <= 1'b0;
    end else begin
      case (st)
        ST_OFF: begin
          st <= ST_ARM; nper <= 2'd0;
        end
        ST_ARM: begin
          rd_word <= {W{1'b0}};
          if (dac_valid) begin
            if (nper == 2'd0)                          nper <= 2'd1;
            else if (nper == 2'd1) begin period <= spacing; nper <= 2'd2; end
            else if (spacing == period)                nper <= 2'd3;
            else begin period <= spacing; nper <= 2'd2; end
          end
          if (bfx_m2_fw_d) begin
            st <= ST_FLT; fault_r <= 1'b1;
          end else if (nper == 2'd3 && primed_ok) begin
            st <= ST_RUN; level_min <= level; level_max <= level;
          end
        end
        ST_RUN: begin
          if (level < level_min) level_min <= level;
          if (level > level_max) level_max <= level;
          if (bfx_m2_fw_d) begin                       // write side has counted it
            st <= ST_FLT; fault_r <= 1'b1; rd_word <= {W{1'b0}};
          end else if (level < LO || level > HI || bad_cadence) begin
            slip_r_toggle <= ~slip_r_toggle;
            st <= ST_FLT; fault_r <= 1'b1; rd_word <= {W{1'b0}};
          end else if (dac_valid) begin
            if (level == 0) begin                      // backstop: band check fires first
              unf_toggle <= ~unf_toggle;
              st <= ST_FLT; fault_r <= 1'b1; rd_word <= {W{1'b0}};
            end else begin
              rd_word <= rword;
              rbin    <= rbin + 1'b1;
              rgray   <= bin2gray(rbin + 1'b1);
              pop_d   <= 1'b1;
            end
          end
        end
        default: begin                                 // ST_FLT
          rd_word <= {W{1'b0}};
        end
      endcase
    end

    if (pop_d && rd_word[W-1]) sat_toggle <= ~sat_toggle;
  end

  assign running    = (st == ST_RUN);
  assign armed_r    = en_d;
  assign dac_data_i = rd_word[15:0];
  assign dac_data_q = rd_word[31:16];
endmodule
