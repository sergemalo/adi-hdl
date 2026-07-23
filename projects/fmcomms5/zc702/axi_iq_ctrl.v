// axi_iq_ctrl.v
//
// AXI4-Lite register slave for the RX-path sample injector (iq_override).
// Replaces the 32-bit axi_gpio that previously carried the override controls:
// that GPIO could hold ONE common I/Q pair (32 bits), but per-channel injection
// needs 8 x 16 = 128 bits of distinct lane values, plus 128 bits of pattern
// seeds. Same base address (0x79060000), same up_axi shim as axi_fir_ctrl.
//
// Two injection modes, selected by CTRL[1]:
//   DC      (0) - each lane emits its own constant. Distinct values per lane
//                 make INPUT-side routing observable (which RX lane feeds which
//                 FIR), which a common DC structurally cannot show.
//   PATTERN (1) - each lane emits its own 16-bit LFSR sequence (distinct seed).
//                 Deterministic and reproducible in Python, so the full
//                 convolution can be checked bit-exactly on all 8 lanes at once
//                 -- no reference lane needed, which is what the 8-lane capture
//                 topology made impossible for the old golden-model tests.
//
// =====================================================================
// Register map (byte offsets from base)
// =====================================================================
//   0x000  ID       RO   0x49514338 ("IQC8")
//   0x004  SCRATCH  RW   free 32-bit register (AXI liveness test)
//   0x008  CTRL     RW   [0] override_en   1 = inject, 0 = passthrough (live RX)
//                        [1] pattern_mode  0 = DC, 1 = LFSR
//                        [2] pattern_hold  1 = hold every LFSR at its seed
//   0x040 + 4*k     RW   lane k DC value      [15:0], k = 0..7
//   0x080 + 4*k     RW   lane k LFSR seed     [15:0], k = 0..7
//
// pattern_hold is a LEVEL, not a pulse: software writes hold=1 (all LFSRs load
// their seed), then hold=0 (all start running from a known state). A level
// crosses clock domains safely with a plain synchronizer; a pulse would not.
//
// CDC: all outputs leave in the s_axi_aclk domain. iq_override synchronizes the
// three control bits and registers the quasi-static value/seed buses, exactly
// as it did with the GPIO signals. Covered by the existing s_axi <-> FIR-clock
// set_clock_groups -asynchronous.

`timescale 1ns/100ps

module axi_iq_ctrl #(
  parameter NUM_LANES  = 8,
  parameter DATA_WIDTH = 16
) (
  // to iq_override (s_axi_aclk domain)
  output                                  override_en,
  output                                  pattern_mode,
  output                                  pattern_hold,
  output [(NUM_LANES*DATA_WIDTH)-1:0]     dc_flat,     // lane k = k-th 16-bit slice
  output [(NUM_LANES*DATA_WIDTH)-1:0]     seed_flat,   // lane k = k-th 16-bit slice

  // axi4-lite slave interface
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

  localparam [31:0] ID_VALUE = 32'h49514338;   // "IQC8"

  // word addresses (byte offset >> 2)
  localparam [13:0] ADDR_ID      = 14'h000;
  localparam [13:0] ADDR_SCRATCH = 14'h001;
  localparam [13:0] ADDR_CTRL    = 14'h002;
  localparam [13:0] ADDR_DC_BASE = 14'h010;    // 0x040 >> 2
  localparam [13:0] ADDR_SD_BASE = 14'h020;    // 0x080 >> 2

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

  reg [31:0]            up_scratch = 32'd0;
  reg                   up_en      = 1'b0;
  reg                   up_mode    = 1'b0;
  reg                   up_hold    = 1'b0;

  reg [DATA_WIDTH-1:0]  up_dc   [0:NUM_LANES-1];
  reg [DATA_WIDTH-1:0]  up_seed [0:NUM_LANES-1];

  integer li, fi;

  // ---------------------------------------------------------------------
  // Write path. Per-word comparator decode (same proven style as
  // axi_fir_ctrl): a word only stores when its exact address matches, so an
  // unmapped offset writes nothing and cannot alias into a neighbouring lane.
  // ---------------------------------------------------------------------
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_wack    <= 1'b0;
      up_scratch <= 32'd0;
      up_en      <= 1'b0;
      up_mode    <= 1'b0;
      up_hold    <= 1'b0;
      for (fi = 0; fi < NUM_LANES; fi = fi + 1) begin
        up_dc[fi]   <= {DATA_WIDTH{1'b0}};
        up_seed[fi] <= {DATA_WIDTH{1'b0}};
      end
    end else begin
      up_wack <= up_wreq;

      if ((up_wreq == 1'b1) && (up_waddr == ADDR_SCRATCH)) begin
        up_scratch <= up_wdata;
      end
      if ((up_wreq == 1'b1) && (up_waddr == ADDR_CTRL)) begin
        up_en   <= up_wdata[0];
        up_mode <= up_wdata[1];
        up_hold <= up_wdata[2];
      end

      for (li = 0; li < NUM_LANES; li = li + 1) begin
        if ((up_wreq == 1'b1) && (up_waddr == (ADDR_DC_BASE + li[13:0]))) begin
          up_dc[li] <= up_wdata[DATA_WIDTH-1:0];
        end
        if ((up_wreq == 1'b1) && (up_waddr == (ADDR_SD_BASE + li[13:0]))) begin
          up_seed[li] <= up_wdata[DATA_WIDTH-1:0];
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Read path. Runtime-indexed read of the small flop arrays; unmapped
  // offsets read back 0.
  // ---------------------------------------------------------------------
  wire        rd_is_dc   = (up_raddr >= ADDR_DC_BASE) &&
                           (up_raddr <  (ADDR_DC_BASE + NUM_LANES));
  wire        rd_is_seed = (up_raddr >= ADDR_SD_BASE) &&
                           (up_raddr <  (ADDR_SD_BASE + NUM_LANES));
  wire [13:0] rd_dc_idx   = up_raddr - ADDR_DC_BASE;
  wire [13:0] rd_seed_idx = up_raddr - ADDR_SD_BASE;

  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_rack  <= 1'b0;
      up_rdata <= 32'd0;
    end else begin
      up_rack <= up_rreq;

      if (up_rreq == 1'b1) begin
        if (up_raddr == ADDR_ID) begin
          up_rdata <= ID_VALUE;
        end else if (up_raddr == ADDR_SCRATCH) begin
          up_rdata <= up_scratch;
        end else if (up_raddr == ADDR_CTRL) begin
          up_rdata <= {29'd0, up_hold, up_mode, up_en};
        end else if (rd_is_dc) begin
          up_rdata <= {{(32-DATA_WIDTH){1'b0}}, up_dc[rd_dc_idx[2:0]]};
        end else if (rd_is_seed) begin
          up_rdata <= {{(32-DATA_WIDTH){1'b0}}, up_seed[rd_seed_idx[2:0]]};
        end else begin
          up_rdata <= 32'd0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Flatten to the datapath. Lane k occupies bits [(k+1)*16-1 : k*16].
  // ---------------------------------------------------------------------
  assign override_en  = up_en;
  assign pattern_mode = up_mode;
  assign pattern_hold = up_hold;

  genvar gk;
  generate
    for (gk = 0; gk < NUM_LANES; gk = gk + 1) begin: g_flat
      assign dc_flat  [gk*DATA_WIDTH +: DATA_WIDTH] = up_dc[gk];
      assign seed_flat[gk*DATA_WIDTH +: DATA_WIDTH] = up_seed[gk];
    end
  endgenerate

  // ---------------------------------------------------------------------
  // ADI AXI4-Lite shim
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
