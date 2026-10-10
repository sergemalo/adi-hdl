// ============================================================================
// axi_bf_ctrl.v — AXI4-Lite register file for the BFW4 beamformer
//
// Pure register/status block, wired to bf_bank / bf_dac_if / bf_capture in the
// BD (the axi_covar_ctrl <-> covar_bank split). Uses ADI's up_axi shim, like
// axi_covar_ctrl.v.
//
// =====================================================================
// Register map, base 0x790A0000 (byte offsets). bf_regmap.py (B6) must match.
// =====================================================================
//   0x000  ID           RO   0x42465734 ("BFW4")
//   0x004  SCRATCH      RW
//   0x008  ACTIVE_SEL   RW   [0] bank applied to the datapath (2-FF synchronised
//                            in bf_bank; the select travels with each sample)
//   0x00C  TX_SRC       RW   [0] 1 = beamformer output to TX2, 0 = zero
//   0x010  CAPTURE_SEL  RW   [2:0] 0 = off, c+1 = BF output replaces channel c
//                            (c = 0..3) in the RX DMA capture; 5..7 = off
//   0x014  STATUS       RO   [0] FIFO_PRIMED      (l_clk, synchronised)
//                            [1] CORE_DMA_SEL_I   (axi_ad9361_0 TX2 I DATA_SEL == DMA)
//                            [2] CORE_DMA_SEL_Q   (axi_ad9361_0 TX2 Q DATA_SEL == DMA)
//                       W1C  [8] UNF  sticky: FIFO underflow seen
//                            [9] OVF  sticky: FIFO overflow seen
//                            [10] SAT sticky: an output sample was clipped
//   0x018  SAT_COUNT    RO   clipped output samples, wraps (not cleared by W1C)
//   0x01C  UNF_COUNT    RO   underflow events, wraps
//   0x020  OVF_COUNT    RO   overflow events (dropped samples), wraps
//   0x024  INFO         RO   [7:0] bf_bank LATENCY, [15:8] FIFO PRIME,
//                            [23:16] FIFO DEPTH
//
//   Banks: byte 0x100 + 0x40*b + 4*w   (b = 0..1, w = 0..9), bf_golden
//   BANK_FIELD_ORDER:
//     w = 0..7  W0_RE, W0_IM, W1_RE, W1_IM, W2_RE, W2_IM, W3_RE, W3_IM
//               s18, stored as 18 bits, read back sign-extended to 32
//     w = 8     SHIFT       [4:0]
//     w = 9     WEIGHT_SEQ  32-bit, the covar BLOCK_SEQ the weights came from
//     w = 10..15 unmapped, read 0
//   The bank region is decoded by RANGE compare (axi_covar_ctrl lesson).
//
// Reset values = the IDLE declaration: both banks zero, SHIFT 0, ACTIVE_SEL 0,
// TX_SRC 0, CAPTURE_SEL 0.
//
// HOST DISCIPLINE: write the INACTIVE bank, then flip ACTIVE_SEL. Rewrite a
// bank only >= 15 sample clocks (< 0.5 us) after flipping away from it.
// The bank buses are quasi-static into the sample domain (as for fir/phase).
//
// CDC into this domain: six single bits through 2-FF synchronisers named
// bfx_m1_* / bfx_m2_* (XDC false-paths the m1 stage). Event inputs are
// toggles; each edge is one event. Requires s_axi_aclk >= ~2.5x the sample
// rate (100 MHz vs 30.72 MSPS here).
// ============================================================================

`timescale 1ns/100ps

module axi_bf_ctrl #(
  parameter integer W_W      = 18,
  parameter integer SHIFT_W  = 5,
  parameter integer LATENCY  = 13,
  parameter integer PRIME    = 8,
  parameter integer DEPTH    = 16
) (
  // to bf_bank
  output wire [8*W_W-1:0]   w_bank0,
  output wire [8*W_W-1:0]   w_bank1,
  output wire [SHIFT_W-1:0] shift_bank0,
  output wire [SHIFT_W-1:0] shift_bank1,
  output wire               active_sel,
  // to bf_dac_if / bf_capture
  output wire               tx_src,
  output wire [2:0]         capture_sel,
  // status in (async)
  input  wire               sat_toggle,      // divclk
  input  wire               ovf_toggle,      // divclk
  input  wire               unf_toggle,      // l_clk
  input  wire               fifo_primed,     // l_clk
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

  localparam [31:0] ID_VALUE = 32'h42465734;   // "BFW4"
  localparam [7:0]  INFO_LAT   = LATENCY;
  localparam [7:0]  INFO_PRIME = PRIME;
  localparam [7:0]  INFO_DEPTH = DEPTH;

  localparam [13:0] WORD_ID          = 14'h000;
  localparam [13:0] WORD_SCRATCH     = 14'h001;
  localparam [13:0] WORD_ACTIVE_SEL  = 14'h002;
  localparam [13:0] WORD_TX_SRC      = 14'h003;
  localparam [13:0] WORD_CAPTURE_SEL = 14'h004;
  localparam [13:0] WORD_STATUS      = 14'h005;
  localparam [13:0] WORD_SAT_COUNT   = 14'h006;
  localparam [13:0] WORD_UNF_COUNT   = 14'h007;
  localparam [13:0] WORD_OVF_COUNT   = 14'h008;
  localparam [13:0] WORD_INFO        = 14'h009;

  localparam [13:0] BANK_BASE_WORD   = 14'h040;   // byte 0x100
  localparam [13:0] BANK_STRIDE_WORD = 14'h010;   // byte 0x40
  localparam integer NUM_BANKS       = 2;
  localparam integer WORDS_USED      = 10;

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
  // fault: bit-slice decode (aliases e.g. word 0x1040 onto the bank region)
  wire        wr_bank = up_waddr[6];
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
  reg               up_tx_src      = 1'b0;
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
  (* ASYNC_REG = "TRUE" *) reg [5:0] bfx_m1_st = 6'd0;
  (* ASYNC_REG = "TRUE" *) reg [5:0] bfx_m2_st = 6'd0;
  reg [2:0] tog_d = 3'd0;               // previous synchronised toggles
  always @(posedge up_clk) begin
    bfx_m1_st <= {dac_enable_q, dac_enable_i, fifo_primed, unf_toggle, ovf_toggle, sat_toggle};
    bfx_m2_st <= bfx_m1_st;
    tog_d     <= bfx_m2_st[2:0];
  end
  wire ev_sat = bfx_m2_st[0] ^ tog_d[0];
  wire ev_ovf = bfx_m2_st[1] ^ tog_d[1];
  wire ev_unf = bfx_m2_st[2] ^ tog_d[2];
  wire [2:0] live = bfx_m2_st[5:3];      // {dma_q, dma_i, primed}

  reg [31:0] sat_cnt = 32'd0, unf_cnt = 32'd0, ovf_cnt = 32'd0;
  reg        st_unf = 1'b0, st_ovf = 1'b0, st_sat = 1'b0;

  wire w1c = up_wreq && (up_waddr == WORD_STATUS);

  // ------------------------------------------------------------------ writes
  always @(posedge up_clk) begin
    if (!up_rstn) begin
      up_wack        <= 1'b0;
      up_scratch     <= 32'd0;
      up_active_sel  <= 1'b0;
      up_tx_src      <= 1'b0;
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
          WORD_TX_SRC:      up_tx_src      <= up_wdata[0];
          WORD_CAPTURE_SEL: up_capture_sel <= up_wdata[2:0];
          default: ;
        endcase
      end
    end
  end

  // sticky flags (set wins over a same-cycle clear) and counters
  always @(posedge up_clk) begin
    if (!up_rstn) begin
      st_unf <= 1'b0; st_ovf <= 1'b0; st_sat <= 1'b0;
      sat_cnt <= 32'd0; unf_cnt <= 32'd0; ovf_cnt <= 32'd0;
    end else begin
      st_unf <= ev_unf | (st_unf & ~(w1c & up_wdata[8]));
      st_ovf <= ev_ovf | (st_ovf & ~(w1c & up_wdata[9]));
      st_sat <= ev_sat | (st_sat & ~(w1c & up_wdata[10]));
      if (ev_sat) sat_cnt <= sat_cnt + 1'b1;
      if (ev_unf) unf_cnt <= unf_cnt + 1'b1;
      if (ev_ovf) ovf_cnt <= ovf_cnt + 1'b1;
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
          WORD_TX_SRC:      up_rdata <= {31'd0, up_tx_src};
          WORD_CAPTURE_SEL: up_rdata <= {29'd0, up_capture_sel};
          WORD_STATUS:      up_rdata <= {21'd0, st_sat, st_ovf, st_unf, 5'd0, live};
          WORD_SAT_COUNT:   up_rdata <= sat_cnt;
          WORD_UNF_COUNT:   up_rdata <= unf_cnt;
          WORD_OVF_COUNT:   up_rdata <= ovf_cnt;
          WORD_INFO:        up_rdata <= {8'd0, INFO_DEPTH, INFO_PRIME, INFO_LAT};
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
  assign tx_src      = up_tx_src;
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
