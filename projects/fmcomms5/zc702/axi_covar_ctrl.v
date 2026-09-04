// axi_covar_ctrl.v
//
// AXI4-Lite register slave for the covariance-matrix accumulator (covar_bank).
// Pure register/status block -- does NOT instantiate covar_bank.v itself,
// exactly matching axi_fir_ctrl.v's split (that file does not instantiate
// fir_bank.v either): the datapath module is wired to this one externally,
// at a higher level of hierarchy.
//
// covar_bank.v lives entirely in the SAMPLE clock domain and has no reset
// port (confirmed against chan_delay.v: "every register here is valid-
// enabled"). This wrapper lives in the s_axi_aclk / up_clk domain and DOES
// use the standard AXI4-Lite active-low s_axi_aresetn, exactly like
// axi_fir_ctrl.v -- the two reset conventions are correct for their
// respective domains and are not in conflict.
//
// =====================================================================
// Register map (byte offsets from the peripheral base) -- MUST match
// covar_regmap.py exactly; that Python module is the single source of truth
// for these offsets and imports nothing from here, so any change on either
// side needs a matching edit on the other.
// =====================================================================
//
//   GLOBAL control:
//     0x000  ID                    RO   0x434F5634 ("COV4")
//     0x004  SCRATCH               RW   free 32-bit register (AXI liveness)
//     0x008  ENABLE                RW   [0] accumulate enable (unsynchronized
//                                        here -- covar_bank.v does its own
//                                        2-FF sync internally, exactly like
//                                        fir_bank.v does for active_sel)
//     0x00C  LATEST_COMPLETE_BANK  RO   [1:0] synchronized bank index (0..2)
//     0x010  BLOCK_SEQ             RO   monotonic block counter, synchronized
//
//   Bank region (16 values x 2 words = 32 words per bank, 3 banks):
//     BANK_REGION_BASE = 0x1000
//     BANK_STRIDE       = 0x100 (64 words of address space per bank; only
//                         the first 32 are populated -- the rest reads 0,
//                         same "reserve more than used" pattern as FIR's
//                         per-channel 0x100 stride for a 21-tap/84-byte lane)
//     word(bank, w) = BANK_REGION_BASE + bank*BANK_STRIDE + 4*w    (w = 0..31)
//
// Address bit fields (up_addr is a WORD address = byte offset >> 2):
//   Unlike axi_fir_ctrl.v's coeff region (which sits exactly at the top bit
//   of its word-address field, so a single-bit slice partitions the whole
//   space with no aliasing), covar's bank region sits at word address 0x400
//   -- NOT the field's MSB. A bit-slice select here would alias every higher
//   address that happens to share that one bit (e.g. word 0x2400) into the
//   bank region. So bank-region select below uses an explicit RANGE compare
//   instead of a bit slice -- functionally equivalent to FIR's approach when
//   the region IS at the MSB, but correct here where it isn't.
//
// Any unmapped / out-of-range offset reads back 0x00000000, matching
// axi_fir_ctrl.v's convention.
//
// CDC: block_done_toggle (from covar_bank.v, sample-clock domain, async wrt
// up_clk) is 2-FF synchronized HERE. latest_complete_bank_async/
// block_seq_async are captured into up_clk registers only on a detected
// toggle edge -- by which point they have been stable in the source domain
// for many up_clk cycles (see covar_bank.v's header for why no faster
// capture is needed). rd_data is a wide combinational value from covar_bank
// (bank_mem contents, addressed by rd_bank_sel/rd_word_sel which are
// THEMSELVES driven from this up_clk domain via up_raddr) -- protected
// structurally by the triple-buffer's 2-block-period hold, not by a
// handshake; latched through a single up_clk register stage below, same
// timing pattern axi_fir_ctrl.v uses for its own coefficient readback.

`timescale 1ns/100ps

module axi_covar_ctrl #(
  parameter NUM_BANKS      = 3,
  parameter WORDS_PER_BANK = 32     // matches covar_bank's 2*NUM_VALUES (16 values x lo/hi)
) (
  // -----------------------------------------------------------------
  // To/from covar_bank.v -- wired externally, at a higher hierarchy
  // level, exactly like axi_fir_ctrl.v <-> fir_bank.v.
  // -----------------------------------------------------------------
  output                     enable_raw,               // -> covar_bank.enable_raw
  output [1:0]                rd_bank_sel,              // -> covar_bank.rd_bank_sel
  output [4:0]                rd_word_sel,              // -> covar_bank.rd_word_sel
  input  [31:0]                rd_data,                 // <- covar_bank.rd_data
  input                        block_done_toggle,        // <- covar_bank.block_done_toggle (async)
  input  [1:0]                 latest_complete_bank_async,// <- covar_bank.latest_complete_bank (async)
  input  [31:0]                block_seq_async,          // <- covar_bank.block_seq (async)

  // axi4-lite slave interface (identical port list/style to axi_fir_ctrl.v)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input                                 s_axi_aclk,
  input                                 s_axi_aresetn,

  input                                 s_axi_awvalid,
  input       [15:0]                    s_axi_awaddr,
  input       [ 2:0]                    s_axi_awprot,
  output                                s_axi_awready,

  input                                 s_axi_wvalid,
  input       [31:0]                    s_axi_wdata,
  input       [ 3:0]                    s_axi_wstrb,
  output                                s_axi_wready,

  output                                s_axi_bvalid,
  output      [ 1:0]                    s_axi_bresp,
  input                                 s_axi_bready,

  input                                 s_axi_arvalid,
  input       [15:0]                    s_axi_araddr,
  input       [ 2:0]                    s_axi_arprot,
  output                                s_axi_arready,

  output                                s_axi_rvalid,
  output      [ 1:0]                    s_axi_rresp,
  output      [31:0]                    s_axi_rdata,
  input                                 s_axi_rready
);

  localparam [31:0] ID_VALUE = 32'h434F5634;   // "COV4"

  // Word addresses -- MUST match covar_regmap.py's byte offsets / 4 exactly.
  localparam [13:0] WORD_ID                   = 14'h000;
  localparam [13:0] WORD_SCRATCH              = 14'h001;
  localparam [13:0] WORD_ENABLE               = 14'h002;
  localparam [13:0] WORD_LATEST_COMPLETE_BANK = 14'h003;
  localparam [13:0] WORD_BLOCK_SEQ            = 14'h004;

  localparam [13:0] BANK_REGION_BASE_WORD = 14'h400;  // byte 0x1000 / 4
  localparam [13:0] BANK_STRIDE_WORD      = 14'h040;  // byte 0x100  / 4 (64 words)

  // ---------------------------------------------------------------------
  // Compile-time guards, matching axi_fir_ctrl.v's style: halt elaboration
  // loudly rather than silently building an address map that can't hold
  // what the parameters ask for.
  // ---------------------------------------------------------------------
  generate
    if (NUM_BANKS > 4) begin: g_num_banks_guard
      // bank index field below is 2 bits (up to 4); NUM_BANKS must fit.
      NUM_BANKS_exceeds_2_bit_bank_index_field _bad_num_banks ();
    end
    if (WORDS_PER_BANK > 64) begin: g_words_per_bank_guard
      // BANK_STRIDE_WORD (64 words) must be able to hold one bank.
      WORDS_PER_BANK_exceeds_bank_stride _bad_words_per_bank ();
    end
  endgenerate

  // ---------------------------------------------------------------------
  // up_axi register bus (AXI_ADDRESS_WIDTH = 16 => 14-bit word address)
  // ---------------------------------------------------------------------
  wire                    up_clk;
  wire                    up_rstn;

  wire                    up_wreq;
  wire  [13:0]            up_waddr;
  wire  [31:0]            up_wdata;
  reg                     up_wack  = 1'b0;

  wire                    up_rreq;
  wire  [13:0]            up_raddr;
  reg   [31:0]            up_rdata = 32'd0;
  reg                     up_rack  = 1'b0;

  assign up_clk  = s_axi_aclk;
  assign up_rstn = s_axi_aresetn;

  // ---------------------------------------------------------------------
  // Bank-region address decode: RANGE compare (see header note on why this
  // differs from axi_fir_ctrl.v's bit-slice, which only works because that
  // region sits exactly at the address field's MSB -- covar's doesn't).
  // ---------------------------------------------------------------------
  wire wr_bank_region = (up_waddr >= BANK_REGION_BASE_WORD) &&
                         (up_waddr <  BANK_REGION_BASE_WORD + NUM_BANKS * BANK_STRIDE_WORD);
  wire [13:0] wr_sub_addr   = up_waddr - BANK_REGION_BASE_WORD;
  wire [1:0]  wr_bank_idx   = wr_sub_addr[7:6];
  wire [5:0]  wr_word_idx   = wr_sub_addr[5:0];
  wire        wr_bank_valid = wr_bank_region && (wr_word_idx < WORDS_PER_BANK);

  wire rd_bank_region = (up_raddr >= BANK_REGION_BASE_WORD) &&
                         (up_raddr <  BANK_REGION_BASE_WORD + NUM_BANKS * BANK_STRIDE_WORD);
  wire [13:0] rd_sub_addr   = up_raddr - BANK_REGION_BASE_WORD;
  wire [1:0]  rd_bank_idx   = rd_sub_addr[7:6];
  wire [5:0]  rd_word_idx   = rd_sub_addr[5:0];
  wire        rd_bank_valid = rd_bank_region && (rd_word_idx < WORDS_PER_BANK);

  // Feeds covar_bank.v's readback port continuously; covar_bank's mux is
  // purely combinational, so this is safe to drive every cycle regardless
  // of up_rreq (only actually latched into up_rdata when a read is live).
  assign rd_bank_sel = rd_bank_valid ? rd_bank_idx  : 2'd0;
  assign rd_word_sel = rd_bank_valid ? rd_word_idx[4:0] : 5'd0;

  // ---------------------------------------------------------------------
  // block_done_toggle: 2-FF synchronizer (single bit -- always safe).
  // latest_complete_bank_async / block_seq_async are captured ONLY on a
  // detected toggle edge, per covar_bank.v's documented CDC contract: by
  // the time the synchronized toggle shows a change, the source values
  // have been stable for many up_clk cycles already.
  // ---------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) reg toggle_meta = 1'b0, toggle_sync = 1'b0, toggle_sync_d = 1'b0;
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      toggle_meta   <= 1'b0;
      toggle_sync   <= 1'b0;
      toggle_sync_d <= 1'b0;
    end else begin
      toggle_meta   <= block_done_toggle;
      toggle_sync   <= toggle_meta;
      toggle_sync_d <= toggle_sync;
    end
  end
  wire toggle_edge = toggle_sync ^ toggle_sync_d;

  reg [1:0]  up_latest_complete_bank = 2'd0;
  reg [31:0] up_block_seq            = 32'd0;
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_latest_complete_bank <= 2'd0;
      up_block_seq            <= 32'd0;
    end else if (toggle_edge) begin
      up_latest_complete_bank <= latest_complete_bank_async;
      up_block_seq            <= block_seq_async;
    end
  end

  // ---------------------------------------------------------------------
  // ENABLE / SCRATCH: plain up_clk registers, no synchronization needed on
  // this side -- covar_bank.v synchronizes enable_raw itself internally,
  // exactly like fir_bank.v does for active_sel ("Each FIR lane
  // synchronizes the shared active_sel itself").
  // ---------------------------------------------------------------------
  reg [31:0] up_scratch = 32'd0;
  reg        up_enable  = 1'b0;

  assign enable_raw = up_enable;

  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_wack    <= 1'b0;
      up_scratch <= 32'd0;
      up_enable  <= 1'b0;
    end else begin
      up_wack <= up_wreq;

      if ((up_wreq == 1'b1) && (up_waddr == WORD_SCRATCH)) begin
        up_scratch <= up_wdata;
      end
      if ((up_wreq == 1'b1) && (up_waddr == WORD_ENABLE)) begin
        up_enable <= up_wdata[0];
      end
      // LATEST_COMPLETE_BANK / BLOCK_SEQ / ID / bank region are read-only;
      // writes to them are accepted at the AXI level (up_wack still fires)
      // but have no effect, matching axi_fir_ctrl.v's convention for its
      // own read-only ID register.
    end
  end

  // read path -- decode combinationally (above), latch on up_rreq, same
  // one-cycle timing pattern as axi_fir_ctrl.v's up_coeff0/up_coeff1 reads.
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_rack  <= 1'b0;
      up_rdata <= 32'd0;
    end else begin
      up_rack <= up_rreq;

      if (up_rreq == 1'b1) begin
        if (up_raddr == WORD_ID) begin
          up_rdata <= ID_VALUE;
        end else if (up_raddr == WORD_SCRATCH) begin
          up_rdata <= up_scratch;
        end else if (up_raddr == WORD_ENABLE) begin
          up_rdata <= {31'd0, up_enable};
        end else if (up_raddr == WORD_LATEST_COMPLETE_BANK) begin
          up_rdata <= {30'd0, up_latest_complete_bank};
        end else if (up_raddr == WORD_BLOCK_SEQ) begin
          up_rdata <= up_block_seq;
        end else if (rd_bank_valid) begin
          up_rdata <= rd_data;      // covar_bank's combinational mux, addressed above
        end else begin
          up_rdata <= 32'd0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // ADI AXI4-Lite shim (identical instantiation to axi_fir_ctrl.v)
  // ---------------------------------------------------------------------
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
