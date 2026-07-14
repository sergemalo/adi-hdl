// axi_fir_ctrl.v
//
// Minimal AXI4-Lite register slave for FIR coefficients, DOUBLE BUFFERED.
// Built on ADI's up_axi shim (library/common/up_axi.v).
//
// Two coefficient banks are stored.  Software writes the *inactive* bank,
// then flips active_sel (CTRL bit 0) to commit.  The FIR selects between the
// two banks with a synchronized copy of active_sel, so no output sample ever
// sees a mixed (torn) tap set.
//
// Register map (byte offsets from the peripheral base address):
//
//   0x000  ID        RO   reads 0x46495230 ("FIR0")
//   0x004  SCRATCH   RW   free 32-bit register, write/read test
//   0x008  CFG       RW   [4:0] coeff_frac (Q-format shift), rest reads 0
//   0x00C  CTRL      RW   [0] active_sel (0 = bank0 feeds FIR, 1 = bank1)
//   0x040  BANK0[0]  RW   COEFF_WIDTH bits, zero-extended on read
//   0x044  BANK0[1]  RW
//   0x048  BANK0[2]  RW
//   ...    BANK0[k]  RW   at 0x040 + 4*k, for k < NUM_COEFF
//   0x080  BANK1[0]  RW
//   0x084  BANK1[1]  RW
//   0x088  BANK1[2]  RW
//   ...    BANK1[k]  RW   at 0x080 + 4*k, for k < NUM_COEFF
//
// Any unmapped offset reads back 0x00000000.
// A readback of 0xDEADDEAD means up_rack never asserted (up_axi timeout).
//
// Software commit protocol:
//   1. read CTRL -> current active_sel (or track it in software)
//   2. write the full new tap set into the INACTIVE bank (bank = ~active_sel)
//   3. write CTRL with active_sel flipped   <-- atomic commit
// Do not write the active bank; that is the torn case double buffering exists
// to avoid.  After the flip, the old bank is free to rewrite on the next
// update -- see the CDC note in fir_i0.v (the swap propagates in a few FIR
// clocks, far faster than the next /dev/mem write).
//
// coeff_frac is NOT double buffered: it is the coefficient format, fixed at
// init, not part of the per-update swap.
//
// NOTE: coeff_flat0/1 and active_sel leave this module in the s_axi_aclk
// domain. The FIR runs in the ADC clock domain and synchronizes active_sel
// itself; the wide banks are quasi-static and cross via a false_path.

`timescale 1ns/100ps

module axi_fir_ctrl #(
  parameter NUM_COEFF   = 3,
  parameter COEFF_WIDTH = 18
) (
  // coefficient outputs to the FIR datapath (s_axi_aclk domain)
  output [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat0,
  output [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat1,
  output                                active_sel,   // 1 bit, synchronized in the FIR
  output [4:0]                          coeff_frac,

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

  localparam [31:0] ID_VALUE = 32'h46495230;   // "FIR0"

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
  // Address decode.  up_waddr is a WORD address: byte offset >> 2.
  //   bank0 : word 0x010..0x01F  (byte 0x040..0x07C)
  //   bank1 : word 0x020..0x02F  (byte 0x080..0x0BC)
  // ---------------------------------------------------------------------

  wire        wr_bank0_sel = (up_waddr[13:4] == 10'h001);
  wire        wr_bank1_sel = (up_waddr[13:4] == 10'h002);
  wire [ 3:0] wr_coeff_idx =  up_waddr[3:0];

  wire        rd_bank0_sel = (up_raddr[13:4] == 10'h001);
  wire        rd_bank1_sel = (up_raddr[13:4] == 10'h002);
  wire [ 3:0] rd_coeff_idx =  up_raddr[3:0];

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------

  reg [31:0]              up_scratch     = 32'd0;
  reg [ 4:0]              up_coeff_frac  = 5'd0;
  reg                     up_active_sel  = 1'b0;
  reg [COEFF_WIDTH-1:0]   up_coeff0 [0:NUM_COEFF-1];
  reg [COEFF_WIDTH-1:0]   up_coeff1 [0:NUM_COEFF-1];

  integer i;

  // write path
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_wack       <= 1'b0;
      up_scratch    <= 32'd0;
      up_coeff_frac <= 5'd0;
      up_active_sel <= 1'b0;
      for (i = 0; i < NUM_COEFF; i = i + 1) begin
        up_coeff0[i] <= {COEFF_WIDTH{1'b0}};
        up_coeff1[i] <= {COEFF_WIDTH{1'b0}};
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
        up_active_sel <= up_wdata[0];      // <-- atomic commit
      end

      for (i = 0; i < NUM_COEFF; i = i + 1) begin
        if ((up_wreq == 1'b1) && (wr_bank0_sel == 1'b1) &&
            (wr_coeff_idx == i[3:0])) begin
          up_coeff0[i] <= up_wdata[COEFF_WIDTH-1:0];
        end
        if ((up_wreq == 1'b1) && (wr_bank1_sel == 1'b1) &&
            (wr_coeff_idx == i[3:0])) begin
          up_coeff1[i] <= up_wdata[COEFF_WIDTH-1:0];
        end
      end
    end
  end

  // read path
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
        end else if ((rd_bank0_sel == 1'b1) && (rd_coeff_idx < NUM_COEFF)) begin
          up_rdata <= {{(32-COEFF_WIDTH){1'b0}}, up_coeff0[rd_coeff_idx]};
        end else if ((rd_bank1_sel == 1'b1) && (rd_coeff_idx < NUM_COEFF)) begin
          up_rdata <= {{(32-COEFF_WIDTH){1'b0}}, up_coeff1[rd_coeff_idx]};
        end else begin
          up_rdata <= 32'd0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Flatten both banks out to the datapath
  // ---------------------------------------------------------------------

  assign coeff_frac = up_coeff_frac;
  assign active_sel = up_active_sel;

  genvar n;
  generate
    for (n = 0; n < NUM_COEFF; n = n + 1) begin: g_coeff
      assign coeff_flat0[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH] = up_coeff0[n];
      assign coeff_flat1[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH] = up_coeff1[n];
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
