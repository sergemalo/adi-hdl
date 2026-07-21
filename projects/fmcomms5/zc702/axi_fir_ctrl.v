// axi_fir_ctrl.v  (WIDENED: NUM_CHANNELS lanes, one AXI4-Lite slave)
//
// AXI4-Lite register slave for an N-channel bank of double-buffered FIR
// coefficients. One peripheral, one base address, one up_axi shim -- feeding
// NUM_CHANNELS FIR lanes with a SHARED (global) coeff_frac and a SHARED
// (global) active_sel so that a single CTRL write commits all lanes' new
// coefficient sets in the same sample beat (the coherent swap the array needs
// for adaptive nulling -- 8 separate peripherals could not do this atomically).
//
// This is the Rung-1 register block: it is brought up on its own, with NO
// datapath attached, and validated by devmem readback (fir_ch_regmap_test.py).
// The wide coeff_flat0/1 buses dangle in the Rung-1 build; the FIR lanes are
// wired to their per-channel slices in Rung 3.
//
// =====================================================================
// Register map (byte offsets from the peripheral base)
// =====================================================================
//
//   GLOBAL control (unchanged offsets from the single-channel block):
//     0x000  ID        RO   reads 0x46495238 ("FIR8" -- marks the widened map)
//     0x004  SCRATCH   RW   free 32-bit register (AXI liveness test)
//     0x008  CFG       RW   [4:0] coeff_frac (Q-format shift, ALL channels)
//     0x00C  CTRL      RW   [0]   active_sel (0=bank0, 1=bank1, ALL channels)
//
//   PER-CHANNEL coefficients (channel c in 0..NUM_CHANNELS-1, tap k in
//   0..NUM_COEFF-1), living in the top half of the 64 KB window:
//     COEFF_BASE = 0x8000
//     bank0[c][k] = 0x8000 + c*0x100 + 0x000 + 4*k
//     bank1[c][k] = 0x8000 + c*0x100 + 0x080 + 4*k
//
//   Each channel occupies a 0x100 (256-byte) stride = two 128-byte banks.
//   c=0 -> 0x8000, c=1 -> 0x8100, ... c=7 -> 0x8700.
//
// Address bit fields (up_addr is a WORD address = byte offset >> 2):
//     up_addr[13]   = 1 selects the coefficient region (0 = global control)
//     up_addr[8:6]  = channel  (0..7)
//     up_addr[5]    = bank     (0 = bank0, 1 = bank1)
//     up_addr[4:0]  = tap      (0..31; only < NUM_COEFF is live)
//     up_addr[12:9] = reserved (must be 0)
//
// Any unmapped / out-of-range offset reads back 0x00000000.
//
// Commit protocol (per the double-buffer contract, now array-wide):
//   1. for every channel, write the full new tap set into the INACTIVE bank
//      (bank = ~active_sel)
//   2. write CTRL with active_sel flipped  <-- ONE write, atomic for all lanes
// Do not write the active bank. coeff_frac is NOT double buffered (it is the
// coefficient format, fixed at init, shared by all channels).
//
// CDC: coeff_flat0/1 and active_sel leave in the s_axi_aclk domain. Each FIR
// lane synchronizes the shared active_sel itself; the wide banks are
// quasi-static and cross via a false_path (same treatment as before, just
// more bits -- the set_clock_groups scope must cover the wider buses).

`timescale 1ns/100ps

module axi_fir_ctrl #(
  parameter NUM_CHANNELS = 8,     // FIR lanes fed by this block (<= 8)
  parameter NUM_COEFF    = 3,     // taps per lane (<= 32)
  parameter COEFF_WIDTH  = 18
) (
  // per-channel coefficient buses to the FIR datapath (s_axi_aclk domain).
  // channel c occupies bits [(c+1)*NUM_COEFF*COEFF_WIDTH-1 : c*NUM_COEFF*COEFF_WIDTH];
  // within that, tap k occupies the k-th COEFF_WIDTH slice (same packing the
  // single-channel block used, just repeated per channel).
  output [(NUM_CHANNELS*NUM_COEFF*COEFF_WIDTH)-1:0] coeff_flat0,
  output [(NUM_CHANNELS*NUM_COEFF*COEFF_WIDTH)-1:0] coeff_flat1,
  output                                            active_sel,   // global, 1 bit
  output [4:0]                                       coeff_frac,   // global

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

  localparam [31:0] ID_VALUE = 32'h46495238;   // "FIR8"

  localparam TOTAL_COEFF = NUM_CHANNELS * NUM_COEFF;

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
  // Address decode: split the word address into region / channel / bank /
  // tap fields (see the map above). These are pure combinational slices.
  // ---------------------------------------------------------------------
  wire        wr_coeff_region = up_waddr[13];
  wire [ 2:0] wr_chan         = up_waddr[8:6];
  wire        wr_bank         = up_waddr[5];
  wire [ 4:0] wr_tap          = up_waddr[4:0];

  wire        rd_coeff_region = up_raddr[13];
  wire [ 2:0] rd_chan         = up_raddr[8:6];
  wire        rd_bank         = up_raddr[5];
  wire [ 4:0] rd_tap          = up_raddr[4:0];

  // ---------------------------------------------------------------------
  // Compile-time guards. The address fields cap the map at 8 channels and
  // 32 taps; exceeding either would silently alias writes, so halt
  // elaboration loudly instead (the not-taken branch references an
  // undefined module -> a self-describing hard error only when the bad
  // condition holds).
  // ---------------------------------------------------------------------
  generate
    if (NUM_COEFF > 32) begin: g_num_coeff_guard
      NUM_COEFF_exceeds_32_tap_address_map _bad_num_coeff ();
    end
    if (NUM_CHANNELS > 8) begin: g_num_chan_guard
      NUM_CHANNELS_exceeds_8_channel_address_map _bad_num_chan ();
    end
  endgenerate

  // ---------------------------------------------------------------------
  // Registers.  Coefficient storage is a flat flop array indexed by
  // (channel*NUM_COEFF + tap); every word is read in parallel by the
  // flatten block below, which forces a register-file (not BRAM)
  // implementation -- exactly what a fully-parallel FIR needs.
  // ---------------------------------------------------------------------
  reg [31:0]              up_scratch     = 32'd0;
  reg [ 4:0]              up_coeff_frac  = 5'd0;
  reg                     up_active_sel  = 1'b0;

  (* ram_style = "registers" *)
  reg [COEFF_WIDTH-1:0]   up_coeff0 [0:TOTAL_COEFF-1];
  (* ram_style = "registers" *)
  reg [COEFF_WIDTH-1:0]   up_coeff1 [0:TOTAL_COEFF-1];

  integer ci, ti, fi;

  // write path. The per-word comparator decode (matching the proven
  // single-channel block, extended with the channel index ci) only writes a
  // storage word when BOTH channel and tap match -- so a tap address >=
  // NUM_COEFF, or an unpopulated channel, writes nothing and cannot alias
  // into a neighbour's region.
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_wack       <= 1'b0;
      up_scratch    <= 32'd0;
      up_coeff_frac <= 5'd0;
      up_active_sel <= 1'b0;
      for (fi = 0; fi < TOTAL_COEFF; fi = fi + 1) begin
        up_coeff0[fi] <= {COEFF_WIDTH{1'b0}};
        up_coeff1[fi] <= {COEFF_WIDTH{1'b0}};
      end
    end else begin
      up_wack <= up_wreq;

      if ((up_wreq == 1'b1) && (up_waddr == 14'h001)) begin
        up_scratch <= up_wdata;
      end
      if ((up_wreq == 1'b1) && (up_waddr == 14'h002)) begin
        up_coeff_frac <= up_wdata[4:0];
      end
      if ((up_wreq == 1'b1) && (up_waddr == 14'h003)) begin
        up_active_sel <= up_wdata[0];      // <-- atomic commit, all channels
      end

      for (ci = 0; ci < NUM_CHANNELS; ci = ci + 1) begin
        for (ti = 0; ti < NUM_COEFF; ti = ti + 1) begin
          if ((up_wreq == 1'b1) && wr_coeff_region && (wr_bank == 1'b0) &&
              (wr_chan == ci[2:0]) && (wr_tap == ti[4:0])) begin
            up_coeff0[ci*NUM_COEFF + ti] <= up_wdata[COEFF_WIDTH-1:0];
          end
          if ((up_wreq == 1'b1) && wr_coeff_region && (wr_bank == 1'b1) &&
              (wr_chan == ci[2:0]) && (wr_tap == ti[4:0])) begin
            up_coeff1[ci*NUM_COEFF + ti] <= up_wdata[COEFF_WIDTH-1:0];
          end
        end
      end
    end
  end

  // read path. Runtime-indexed read of the flop array (a big mux); the
  // tap/channel range checks make an out-of-range address read back 0.
  wire rd_coeff_valid = rd_coeff_region &&
                        (rd_tap  < NUM_COEFF) &&
                        (rd_chan < NUM_CHANNELS);
  wire [$clog2(TOTAL_COEFF>1?TOTAL_COEFF:2)-1:0] rd_index = rd_chan*NUM_COEFF + rd_tap;

  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_rack  <= 1'b0;
      up_rdata <= 32'd0;
    end else begin
      up_rack <= up_rreq;

      if (up_rreq == 1'b1) begin
        if (up_raddr == 14'h000) begin
          up_rdata <= ID_VALUE;
        end else if (up_raddr == 14'h001) begin
          up_rdata <= up_scratch;
        end else if (up_raddr == 14'h002) begin
          up_rdata <= {27'd0, up_coeff_frac};
        end else if (up_raddr == 14'h003) begin
          up_rdata <= {31'd0, up_active_sel};
        end else if (rd_coeff_valid && (rd_bank == 1'b0)) begin
          up_rdata <= {{(32-COEFF_WIDTH){1'b0}}, up_coeff0[rd_index]};
        end else if (rd_coeff_valid && (rd_bank == 1'b1)) begin
          up_rdata <= {{(32-COEFF_WIDTH){1'b0}}, up_coeff1[rd_index]};
        end else begin
          up_rdata <= 32'd0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Flatten both banks out to the datapath. channel c, tap n -> flat bit
  // ((c*NUM_COEFF + n)*COEFF_WIDTH). The parallel read of every word here is
  // what pins the storage to flops.
  // ---------------------------------------------------------------------
  assign coeff_frac = up_coeff_frac;
  assign active_sel = up_active_sel;

  genvar gc, gn;
  generate
    for (gc = 0; gc < NUM_CHANNELS; gc = gc + 1) begin: g_chan
      for (gn = 0; gn < NUM_COEFF; gn = gn + 1) begin: g_coeff
        localparam integer BIT = (gc*NUM_COEFF + gn) * COEFF_WIDTH;
        assign coeff_flat0[BIT +: COEFF_WIDTH] = up_coeff0[gc*NUM_COEFF + gn];
        assign coeff_flat1[BIT +: COEFF_WIDTH] = up_coeff1[gc*NUM_COEFF + gn];
      end
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
