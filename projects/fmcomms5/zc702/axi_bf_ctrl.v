// ============================================================================
// axi_bf_ctrl.v — AXI4-Lite register file for the BFW4 beamformer, rev 2
//
// Pure register/status block, wired to bf_bank / bf_dac_if / bf_capture in the
// BD (the axi_covar_ctrl <-> covar_bank split). Uses ADI's up_axi shim.
//
// =====================================================================
// Register map, base 0x790A0000 (byte offsets). argus/regmap.py must match.
// =====================================================================
//   0x000  ID           RO   0x42465734 ("BFW4")
//   0x004  SCRATCH      RW
//   0x008  ACTIVE_SEL   RW   [0] bank applied to the datapath
//   0x00C  OUT_EN       RW   [0] 1 = arm/run the TX2 output path
//                            (effective enable = OUT_EN & ~FAULTED)
//   0x010  CAPTURE_SEL  RW   [2:0] 0 = off, c+1 = BF output replaces channel c
//                            in the RX DMA capture (c = 0..3); 5..7 = off
//   0x014  STATUS       RO   [0] RUNNING        output path is popping to the DAC
//                            [1] CORE_FABRIC_I  axi_ad9361_0 TX2 I DATA_SEL == 2
//                            [2] CORE_FABRIC_Q  axi_ad9361_0 TX2 Q DATA_SEL == 2
//                            [3] ARMED          either FIFO side still sees enable
//                       W1C  [4] FAULTED        fail-stop latch; clearable ONLY
//                                               while OUT_EN = 0
//                       W1C  [8] UNF  [9] OVF  [10] SAT  [11] SLIP   sticky
//   0x018  SAT_COUNT    RO   clipped samples handed to the DAC (wraps)
//   0x01C  UNF_COUNT    RO   underflow faults
//   0x020  OVF_COUNT    RO   overflow faults
//   0x024  INFO         RO   [7:0] LATENCY 13, [15:8] PRIME 8, [23:16] DEPTH 16,
//                            [31:24] WIN 2           -> 0x0210080D
//   0x028  SLIP_COUNT   RO   slip faults (RX gap, DAC cadence, level band)
//   0x02C  LEVEL        RO   [4:0] LEVEL_MIN, [12:8] LEVEL_MAX since RUNNING
//                            (bits synchronised individually: read twice)
//
//   Banks: byte 0x100 + 0x40*b + 4*w   (b = 0..1, w = 0..9)
//     w = 0..7  W0_RE, W0_IM, W1_RE, W1_IM, W2_RE, W2_IM, W3_RE, W3_IM
//               s18 stored as 18 bits, read back sign-extended
//     w = 8     SHIFT [4:0]     w = 9  WEIGHT_SEQ     w = 10..15 read 0
//   Range-compare decode (axi_covar_ctrl lesson).
//
// EVENT RULES (agreed 2026-10-11)
//   * Sticky flags, counters and FAULTED change only while the OUT_EN register
//     is 1 and FAULTED is 0. From the AXI write of OUT_EN = 0 on, they are
//     frozen; only the host's W1C changes them.
//   * The first UNF/OVF/SLIP sets its sticky flag, its counter and FAULTED.
//     FAULTED forces the effective enable to 0 (fail-stop). Re-arming needs
//     OUT_EN = 0, then W1C FAULTED, then OUT_EN = 1.
//   * SAT counts only while live (the datapath emits it only while RUNNING).
//
// Reset values = the IDLE declaration (all zero).
// CDC into this domain: 20 bits through 2-FF synchronisers bfx_m1_* / bfx_m2_*.
// Toggles need s_axi_aclk >= ~2.5x the sample rate (100 MHz vs 30.72 MSPS).
// ============================================================================

`timescale 1ns/100ps

module axi_bf_ctrl #(
  parameter integer W_W      = 18,
  parameter integer SHIFT_W  = 5,
  parameter integer LATENCY  = 13,
  parameter integer PRIME    = 8,
  parameter integer DEPTH    = 16,
  parameter integer WIN      = 2
) (
  // to bf_bank
  output wire [8*W_W-1:0]   w_bank0,
  output wire [8*W_W-1:0]   w_bank1,
  output wire [SHIFT_W-1:0] shift_bank0,
  output wire [SHIFT_W-1:0] shift_bank1,
  output wire               active_sel,
  // to bf_dac_if / bf_capture
  output wire               out_en,
  output wire [2:0]         capture_sel,
  // status in (async)
  input  wire               sat_toggle,      // l_clk
  input  wire               unf_toggle,      // l_clk
  input  wire               slip_r_toggle,   // l_clk
  input  wire               ovf_toggle,      // divclk
  input  wire               slip_w_toggle,   // divclk
  input  wire               running,         // l_clk
  input  wire               armed_r,         // l_clk
  input  wire               armed_w,         // divclk
  input  wire [4:0]         level_min,       // l_clk
  input  wire [4:0]         level_max,       // l_clk
  input  wire               dac_enable_i,    // l_clk, axi_ad9361_0/dac_enable_i1
  input  wire               dac_enable_q,    // l_clk, axi_ad9361_0/dac_enable_q1

  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input                     s_axi_aclk,
  input                     s_axi_aresetn,
  input                     s_axi_awvalid,
  input       [15:0]        s_axi_awaddr,
  input       [ 2:0]        s_axi_awprot,
  output                    s_axi_awready,
  input                     s_axi_wvalid,
  input       [31:0]        s_axi_wdata,
  input       [ 3:0]        s_axi_wstrb,
  output                    s_axi_wready,
  output                    s_axi_bvalid,
  output      [ 1:0]        s_axi_bresp,
  input                     s_axi_bready,
  input                     s_axi_arvalid,
  input       [15:0]        s_axi_araddr,
  input       [ 2:0]        s_axi_arprot,
  output                    s_axi_arready,
  output                    s_axi_rvalid,
  output      [ 1:0]        s_axi_rresp,
  output      [31:0]        s_axi_rdata,
  input                     s_axi_rready
);

  localparam [31:0] ID_VALUE   = 32'h42465734;   // "BFW4"
  localparam [7:0]  INFO_LAT   = LATENCY;
  localparam [7:0]  INFO_PRIME = PRIME;
  localparam [7:0]  INFO_DEPTH = DEPTH;
  localparam [7:0]  INFO_WIN   = WIN;

  localparam [13:0] WORD_ID          = 14'h000;
  localparam [13:0] WORD_SCRATCH     = 14'h001;
  localparam [13:0] WORD_ACTIVE_SEL  = 14'h002;
  localparam [13:0] WORD_OUT_EN      = 14'h003;
  localparam [13:0] WORD_CAPTURE_SEL = 14'h004;
  localparam [13:0] WORD_STATUS      = 14'h005;
  localparam [13:0] WORD_SAT_COUNT   = 14'h006;
  localparam [13:0] WORD_UNF_COUNT   = 14'h007;
  localparam [13:0] WORD_OVF_COUNT   = 14'h008;
  localparam [13:0] WORD_INFO        = 14'h009;
  localparam [13:0] WORD_SLIP_COUNT  = 14'h00A;
  localparam [13:0] WORD_LEVEL       = 14'h00B;

  localparam [13:0] BANK_BASE_WORD   = 14'h040;   // byte 0x100
  localparam [13:0] BANK_STRIDE_WORD = 14'h010;   // byte 0x40
  localparam integer NUM_BANKS       = 2;

  wire        up_clk  = s_axi_aclk;
  wire        up_rstn = s_axi_aresetn;
  wire        up_wreq;
  wire [13:0] up_waddr;
  wire [31:0] up_wdata;
  reg         up_wack  = 1'b0;
  wire        up_rreq;
  wire [13:0] up_raddr;
  reg  [31:0] up_rdata = 32'd0;
  reg         up_rack  = 1'b0;

  // ------------------------------------------------------------ bank decode
`ifdef MUT_ALIAS
  wire        wr_bank = up_waddr[6];               // fault: bit-slice decode
  wire        rd_bank = up_raddr[6];
`else
  wire        wr_bank = (up_waddr >= BANK_BASE_WORD) &&
                        (up_waddr <  BANK_BASE_WORD + NUM_BANKS * BANK_STRIDE_WORD);
  wire        rd_bank = (up_raddr >= BANK_BASE_WORD) &&
                        (up_raddr <  BANK_BASE_WORD + NUM_BANKS * BANK_STRIDE_WORD);
`endif
  wire [13:0] wr_off  = up_waddr - BANK_BASE_WORD;
  wire [13:0] rd_off  = up_raddr - BANK_BASE_WORD;
  wire        wr_b    = wr_off[4];
  wire [3:0]  wr_w    = wr_off[3:0];
  wire        rd_b    = rd_off[4];
  wire [3:0]  rd_w    = rd_off[3:0];

  // ---------------------------------------------------------------- storage
  reg [W_W-1:0]     wt  [0:1][0:7];
  reg [SHIFT_W-1:0] sh  [0:1];
  reg [31:0]        seq [0:1];
  reg [31:0]        up_scratch     = 32'd0;
  reg               up_active_sel  = 1'b0;
  reg               up_out_en      = 1'b0;
  reg [2:0]         up_capture_sel = 3'd0;

  integer b, f;
  initial begin
    for (b = 0; b < 2; b = b + 1) begin
      for (f = 0; f < 8; f = f + 1) wt[b][f] = {W_W{1'b0}};
      sh[b] = {SHIFT_W{1'b0}};
      seq[b] = 32'd0;
    end
  end

  // ------------------------------------------------------ status synchronisers
  // [4:0] toggles {slip_w, ovf, slip_r, unf, sat}, [9:5] live
  // {dma_q, dma_i, armed_w, armed_r, running}, [14:10] level_min, [19:15] level_max
  wire [19:0] st_in = {level_max, level_min,
                       dac_enable_q, dac_enable_i, armed_w, armed_r, running,
                       slip_w_toggle, ovf_toggle, slip_r_toggle, unf_toggle, sat_toggle};
  (* ASYNC_REG = "TRUE" *) reg [19:0] bfx_m1_st = 20'd0;
  (* ASYNC_REG = "TRUE" *) reg [19:0] bfx_m2_st = 20'd0;
  reg [4:0] tog_d = 5'd0;
  always @(posedge up_clk) begin
    bfx_m1_st <= st_in;
    bfx_m2_st <= bfx_m1_st;
    tog_d     <= bfx_m2_st[4:0];
  end
  wire [4:0] ev      = bfx_m2_st[4:0] ^ tog_d;
  wire       ev_sat  = ev[0];
  wire       ev_unf  = ev[1];
  wire       ev_slip = ev[2] | ev[4];
  wire       ev_ovf  = ev[3];
  wire       s_running = bfx_m2_st[5];
  wire       s_armed   = bfx_m2_st[6] | bfx_m2_st[7];
  wire       s_dma_i   = bfx_m2_st[8];
  wire       s_dma_q   = bfx_m2_st[9];
  wire [4:0] s_lmin    = bfx_m2_st[14:10];
  wire [4:0] s_lmax    = bfx_m2_st[19:15];

  reg [31:0] sat_cnt = 32'd0, unf_cnt = 32'd0, ovf_cnt = 32'd0, slip_cnt = 32'd0;
  reg        st_unf = 1'b0, st_ovf = 1'b0, st_sat = 1'b0, st_slip = 1'b0;
  reg        st_faulted = 1'b0;

`ifdef MUT_NO_GATE
  wire live = 1'b1;                                  // fault: events counted always
`else
  wire live = up_out_en & ~st_faulted;
`endif
  wire fault_ev = ev_unf | ev_ovf | ev_slip;
  wire w1c      = up_wreq && (up_waddr == WORD_STATUS);
`ifdef MUT_W1C_ANYTIME
  wire clr_faulted = w1c & up_wdata[4];              // fault: clearable while enabled
`else
  wire clr_faulted = w1c & up_wdata[4] & ~up_out_en;
`endif

  assign out_en = up_out_en & ~st_faulted;

  // ------------------------------------------------------------------ writes
  always @(posedge up_clk) begin
    if (!up_rstn) begin
      up_wack        <= 1'b0;
      up_scratch     <= 32'd0;
      up_active_sel  <= 1'b0;
      up_out_en      <= 1'b0;
      up_capture_sel <= 3'd0;
      for (b = 0; b < 2; b = b + 1) begin
        for (f = 0; f < 8; f = f + 1) wt[b][f] <= {W_W{1'b0}};
        sh[b]  <= {SHIFT_W{1'b0}};
        seq[b] <= 32'd0;
      end
    end else begin
      up_wack <= up_wreq;
      if (up_wreq) begin
        if (wr_bank) begin
          if (wr_w < 4'd8)       wt[wr_b][wr_w[2:0]] <= up_wdata[W_W-1:0];
          else if (wr_w == 4'd8) sh[wr_b]  <= up_wdata[SHIFT_W-1:0];
          else if (wr_w == 4'd9) seq[wr_b] <= up_wdata;
        end else case (up_waddr)
          WORD_SCRATCH:     up_scratch     <= up_wdata;
          WORD_ACTIVE_SEL:  up_active_sel  <= up_wdata[0];
          WORD_OUT_EN:      up_out_en      <= up_wdata[0];
          WORD_CAPTURE_SEL: up_capture_sel <= up_wdata[2:0];
          default: ;
        endcase
      end
    end
  end

  // sticky flags, FAULTED, counters (an event wins over a same-cycle clear)
  always @(posedge up_clk) begin
    if (!up_rstn) begin
      st_unf <= 1'b0; st_ovf <= 1'b0; st_sat <= 1'b0; st_slip <= 1'b0;
      st_faulted <= 1'b0;
      sat_cnt <= 32'd0; unf_cnt <= 32'd0; ovf_cnt <= 32'd0; slip_cnt <= 32'd0;
    end else begin
      st_unf  <= (live & ev_unf)  | (st_unf  & ~(w1c & up_wdata[8]));
      st_ovf  <= (live & ev_ovf)  | (st_ovf  & ~(w1c & up_wdata[9]));
      st_sat  <= (live & ev_sat)  | (st_sat  & ~(w1c & up_wdata[10]));
      st_slip <= (live & ev_slip) | (st_slip & ~(w1c & up_wdata[11]));
      st_faulted <= (live & fault_ev) | (st_faulted & ~clr_faulted);
      if (live & ev_sat)  sat_cnt  <= sat_cnt  + 1'b1;
      if (live & ev_unf)  unf_cnt  <= unf_cnt  + 1'b1;
      if (live & ev_ovf)  ovf_cnt  <= ovf_cnt  + 1'b1;
      if (live & ev_slip) slip_cnt <= slip_cnt + 1'b1;
    end
  end

  // ------------------------------------------------------------------- reads
  function [31:0] sext; input [W_W-1:0] v; sext = {{(32-W_W){v[W_W-1]}}, v}; endfunction

  always @(posedge up_clk) begin
    if (!up_rstn) begin
      up_rack  <= 1'b0;
      up_rdata <= 32'd0;
    end else begin
      up_rack <= up_rreq;
      if (up_rreq) begin
        if (rd_bank) begin
          if (rd_w < 4'd8)       up_rdata <= sext(wt[rd_b][rd_w[2:0]]);
          else if (rd_w == 4'd8) up_rdata <= {{(32-SHIFT_W){1'b0}}, sh[rd_b]};
          else if (rd_w == 4'd9) up_rdata <= seq[rd_b];
          else                   up_rdata <= 32'd0;
        end else case (up_raddr)
          WORD_ID:          up_rdata <= ID_VALUE;
          WORD_SCRATCH:     up_rdata <= up_scratch;
          WORD_ACTIVE_SEL:  up_rdata <= {31'd0, up_active_sel};
          WORD_OUT_EN:      up_rdata <= {31'd0, up_out_en};
          WORD_CAPTURE_SEL: up_rdata <= {29'd0, up_capture_sel};
          WORD_STATUS:      up_rdata <= {20'd0, st_slip, st_sat, st_ovf, st_unf,
                                         3'd0, st_faulted, s_armed, s_dma_q, s_dma_i,
                                         s_running};
          WORD_SAT_COUNT:   up_rdata <= sat_cnt;
          WORD_UNF_COUNT:   up_rdata <= unf_cnt;
          WORD_OVF_COUNT:   up_rdata <= ovf_cnt;
          WORD_INFO:        up_rdata <= {INFO_WIN, INFO_DEPTH, INFO_PRIME, INFO_LAT};
          WORD_SLIP_COUNT:  up_rdata <= slip_cnt;
          WORD_LEVEL:       up_rdata <= {19'd0, s_lmax, 3'd0, s_lmin};
          default:          up_rdata <= 32'd0;
        endcase
      end
    end
  end

  // ----------------------------------------------------------------- outputs
  genvar g;
  generate for (g = 0; g < 8; g = g + 1) begin : g_pack
    assign w_bank0[g*W_W +: W_W] = wt[0][g];
    assign w_bank1[g*W_W +: W_W] = wt[1][g];
  end endgenerate
  assign shift_bank0 = sh[0];
  assign shift_bank1 = sh[1];
  assign active_sel  = up_active_sel;
  assign capture_sel = up_capture_sel;

  up_axi #(
    .AXI_ADDRESS_WIDTH (16)
  ) i_up_axi (
    .up_rstn         (up_rstn),
    .up_clk          (up_clk),
    .up_axi_awvalid  (s_axi_awvalid),
    .up_axi_awaddr   (s_axi_awaddr),
    .up_axi_awready  (s_axi_awready),
    .up_axi_wvalid   (s_axi_wvalid),
    .up_axi_wdata    (s_axi_wdata),
    .up_axi_wstrb    (s_axi_wstrb),
    .up_axi_wready   (s_axi_wready),
    .up_axi_bvalid   (s_axi_bvalid),
    .up_axi_bresp    (s_axi_bresp),
    .up_axi_bready   (s_axi_bready),
    .up_axi_arvalid  (s_axi_arvalid),
    .up_axi_araddr   (s_axi_araddr),
    .up_axi_arready  (s_axi_arready),
    .up_axi_rvalid   (s_axi_rvalid),
    .up_axi_rresp    (s_axi_rresp),
    .up_axi_rdata    (s_axi_rdata),
    .up_axi_rready   (s_axi_rready),
    .up_wreq         (up_wreq),
    .up_waddr        (up_waddr),
    .up_wdata        (up_wdata),
    .up_wack         (up_wack),
    .up_rreq         (up_rreq),
    .up_raddr        (up_raddr),
    .up_rdata        (up_rdata),
    .up_rack         (up_rack));

endmodule
