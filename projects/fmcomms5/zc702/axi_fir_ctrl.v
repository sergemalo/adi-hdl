// axi_fir_ctrl.v
//
// Minimal AXI4-Lite register slave for FIR coefficients.
// Built on ADI's up_axi shim (library/common/up_axi.v).
//
// Register map (byte offsets from the peripheral base address):
//
//   0x000  ID        RO   reads 0x46495230 ("FIR0")
//   0x004  SCRATCH   RW   free 32-bit register, write/read test
//   0x040  COEFF[0]  RW   COEFF_WIDTH bits, zero-extended on read
//   0x044  COEFF[1]  RW
//   0x048  COEFF[2]  RW
//   ...    COEFF[k]  RW   at 0x040 + 4*k, for k < NUM_COEFF
//
// Any unmapped offset reads back 0x00000000.
// A readback of 0xDEADDEAD means up_rack never asserted (up_axi timeout).
//
// Coefficients are stored in flip-flops, not BRAM: a fully parallel FIR
// needs all NUM_COEFF taps readable in the same clock cycle.
//
// NOTE: coeff_flat leaves this module in the s_axi_aclk domain. The FIR
// datapath runs in the ADC clock domain. Crossing that boundary safely is
// a separate step (double-buffered banks + synchronized bank_sel).

`timescale 1ns/100ps

module axi_fir_ctrl #(
  parameter NUM_COEFF   = 3,
  parameter COEFF_WIDTH = 18
) (
  // coefficient outputs to the FIR datapath (s_axi_aclk domain)
  // leave unconnected until the FIR exists
  output [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat,

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
  // Coefficient block occupies word addresses 0x010..0x01F  (byte 0x40..0x7C)
  // ---------------------------------------------------------------------

  wire        wr_coeff_sel = (up_waddr[13:4] == 10'h001);
  wire [ 3:0] wr_coeff_idx =  up_waddr[3:0];

  wire        rd_coeff_sel = (up_raddr[13:4] == 10'h001);
  wire [ 3:0] rd_coeff_idx =  up_raddr[3:0];

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------

  reg [31:0]              up_scratch = 32'd0;
  reg [COEFF_WIDTH-1:0]   up_coeff [0:NUM_COEFF-1];

  integer i;

  // write path
  always @(posedge up_clk) begin
    if (up_rstn == 1'b0) begin
      up_wack    <= 1'b0;
      up_scratch <= 32'd0;
      for (i = 0; i < NUM_COEFF; i = i + 1) begin
        up_coeff[i] <= {COEFF_WIDTH{1'b0}};
      end
    end else begin
      up_wack <= up_wreq;

      if ((up_wreq == 1'b1) && (up_waddr == 14'h001)) begin
        up_scratch <= up_wdata;
      end

      for (i = 0; i < NUM_COEFF; i = i + 1) begin
        if ((up_wreq == 1'b1) && (wr_coeff_sel == 1'b1) &&
            (wr_coeff_idx == i[3:0])) begin
          up_coeff[i] <= up_wdata[COEFF_WIDTH-1:0];
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
        end else if ((rd_coeff_sel == 1'b1) && (rd_coeff_idx < NUM_COEFF)) begin
          up_rdata <= {{(32-COEFF_WIDTH){1'b0}}, up_coeff[rd_coeff_idx]};
        end else begin
          up_rdata <= 32'd0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Flatten coefficients out to the datapath
  // ---------------------------------------------------------------------

  genvar n;
  generate
    for (n = 0; n < NUM_COEFF; n = n + 1) begin: g_coeff
      assign coeff_flat[((n+1)*COEFF_WIDTH)-1 : n*COEFF_WIDTH] = up_coeff[n];
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