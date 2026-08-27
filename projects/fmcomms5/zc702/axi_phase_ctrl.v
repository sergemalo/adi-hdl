`timescale 1ns/1ps
// ----------------------------------------------------------------------------
// axi_phase_ctrl.v -- AXI4-Lite register file + dual coefficient banks for the
// phase/gain calibration stage. Mirrors axi_fir_ctrl.
//
// Register map (byte offsets), base 0x79080000:
//   0x000  ID       (RO) = "PHG4" = 0x50484734
//   0x004  SCRATCH  (RW)
//   0x008  CFG      (RW) [4:0] = coeff_frac (reserved: datapath shift is fixed at
//                        the build-time COEFF_FRAC; CFG must read the same value)
//   0x00C  CTRL     (RW) [0]   = active_sel  (one write commits all channels)
//   coeff word addr = 0x8000 + c*0x100 + k*0x80 + {a:0x00, b:0x04}
//     addr[15]   = 1 -> coefficient region
//     addr[9:8]  = channel c (0..3)
//     addr[7]    = bank k (0..1)
//     addr[2]    = 0 -> a (in-phase/gain term), 1 -> b (quadrature term)
//
// Both banks reset to IDENTITY (a = 1<<COEFF_FRAC = 65536, b = 0) so a fresh
// bitstream is transparent before software writes anything. Coefficients are
// signed Q2.16; software writes the low COEFF_WIDTH bits, readback sign-extends.
//
// DISCIPLINE: write the INACTIVE bank, then flip active_sel. Never write the
// active bank. The datapath reads coefficients combinationally in its own clock
// domain; only active_sel is synchronized (in phase_bank). This is safe because
// the inactive bank is stable before the synchronized swap makes it live.
// ----------------------------------------------------------------------------
module axi_phase_ctrl #(
  parameter integer COEFF_WIDTH        = 18,
  parameter integer COEFF_FRAC         = 16,
  parameter integer C_S_AXI_DATA_WIDTH = 32,
  parameter integer C_S_AXI_ADDR_WIDTH = 16
)(
  // datapath-facing outputs (to phase_bank)
  output wire                     active_sel,
  output wire [4:0]               coeff_frac_o,
  output wire [4*COEFF_WIDTH-1:0] a_bank0,
  output wire [4*COEFF_WIDTH-1:0] b_bank0,
  output wire [4*COEFF_WIDTH-1:0] a_bank1,
  output wire [4*COEFF_WIDTH-1:0] b_bank1,

  // AXI4-Lite slave
  input  wire                              s_axi_aclk,
  input  wire                              s_axi_aresetn,
  input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
  input  wire [2:0]                        s_axi_awprot,
  input  wire                              s_axi_awvalid,
  output wire                              s_axi_awready,
  input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
  input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
  input  wire                              s_axi_wvalid,
  output wire                              s_axi_wready,
  output wire [1:0]                        s_axi_bresp,
  output wire                              s_axi_bvalid,
  input  wire                              s_axi_bready,
  input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
  input  wire [2:0]                        s_axi_arprot,
  input  wire                              s_axi_arvalid,
  output wire                              s_axi_arready,
  output wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
  output wire [1:0]                        s_axi_rresp,
  output wire                              s_axi_rvalid,
  input  wire                              s_axi_rready
);
  localparam [31:0] ID_VALUE = 32'h5048_4734;                 // "PHG4"
  localparam [COEFF_WIDTH-1:0] IDENT_A = (1 <<< COEFF_FRAC);   // 65536
  localparam [COEFF_WIDTH-1:0] IDENT_B = {COEFF_WIDTH{1'b0}};

  wire aclk  = s_axi_aclk;
  wire arstn = s_axi_aresetn;

  // ---- storage ----
  reg [31:0]            scratch;
  reg [4:0]             cfg_frac;
  reg                   ctrl_sel;
  reg [COEFF_WIDTH-1:0] ca [0:3][0:1];   // [channel][bank] : a term
  reg [COEFF_WIDTH-1:0] cb [0:3][0:1];   // [channel][bank] : b term

  integer c, k;

  // ---- write channel (AMD single-transaction template) ----
  reg awready_r, wready_r, bvalid_r, aw_en;
  reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_q;

  always @(posedge aclk) begin
    if (!arstn) begin awready_r <= 1'b0; aw_en <= 1'b1; end
    else if (!awready_r && s_axi_awvalid && s_axi_wvalid && aw_en) begin
      awready_r <= 1'b1; aw_en <= 1'b0;
    end else if (s_axi_bready && bvalid_r) begin
      aw_en <= 1'b1; awready_r <= 1'b0;
    end else awready_r <= 1'b0;
  end
  always @(posedge aclk)
    if (!awready_r && s_axi_awvalid && s_axi_wvalid && aw_en) awaddr_q <= s_axi_awaddr;
  always @(posedge aclk) begin
    if (!arstn) wready_r <= 1'b0;
    else if (!wready_r && s_axi_wvalid && s_axi_awvalid && aw_en) wready_r <= 1'b1;
    else wready_r <= 1'b0;
  end

  wire wr = awready_r && s_axi_awvalid && wready_r && s_axi_wvalid;
  wire        wr_coeff = awaddr_q[15];
  wire [1:0]  wr_chan  = awaddr_q[9:8];
  wire        wr_bank  = awaddr_q[7];
  wire        wr_isb   = awaddr_q[2];

  always @(posedge aclk) begin
    if (!arstn) begin
      scratch  <= 32'd0;
      cfg_frac <= COEFF_FRAC[4:0];
      ctrl_sel <= 1'b0;
      for (c=0;c<4;c=c+1) for (k=0;k<2;k=k+1) begin
        ca[c][k] <= IDENT_A; cb[c][k] <= IDENT_B;
      end
    end else if (wr) begin
      if (wr_coeff) begin
        if (wr_isb) cb[wr_chan][wr_bank] <= s_axi_wdata[COEFF_WIDTH-1:0];
        else        ca[wr_chan][wr_bank] <= s_axi_wdata[COEFF_WIDTH-1:0];
      end else begin
        case (awaddr_q[3:2])
          2'd1: scratch  <= s_axi_wdata;
          2'd2: cfg_frac <= s_axi_wdata[4:0];
          2'd3: ctrl_sel <= s_axi_wdata[0];
          default: ;                          // 2'd0 = ID is read-only
        endcase
      end
    end
  end

  always @(posedge aclk) begin
    if (!arstn) bvalid_r <= 1'b0;
    else if (wr) bvalid_r <= 1'b1;
    else if (s_axi_bready) bvalid_r <= 1'b0;
  end

  // ---- read channel ----
  reg arready_r, rvalid_r;
  reg [31:0] rdata_r;
  reg [C_S_AXI_ADDR_WIDTH-1:0] araddr_q;

  wire        rd_coeff = araddr_q[15];
  wire [1:0]  rd_chan  = araddr_q[9:8];
  wire        rd_bank  = araddr_q[7];
  wire        rd_isb   = araddr_q[2];

  function [31:0] sext;
    input [COEFF_WIDTH-1:0] v;
    sext = {{(32-COEFF_WIDTH){v[COEFF_WIDTH-1]}}, v};
  endfunction

  always @(posedge aclk) begin
    if (!arstn) begin arready_r <= 1'b0; araddr_q <= 0; end
    else if (!arready_r && s_axi_arvalid) begin arready_r <= 1'b1; araddr_q <= s_axi_araddr; end
    else arready_r <= 1'b0;
  end

  always @(posedge aclk) begin
    if (!arstn) rvalid_r <= 1'b0;
    else if (arready_r && s_axi_arvalid && !rvalid_r) begin
      rvalid_r <= 1'b1;
      if (rd_coeff)
        rdata_r <= rd_isb ? sext(cb[rd_chan][rd_bank]) : sext(ca[rd_chan][rd_bank]);
      else case (araddr_q[3:2])
        2'd0: rdata_r <= ID_VALUE;
        2'd1: rdata_r <= scratch;
        2'd2: rdata_r <= {27'd0, cfg_frac};
        2'd3: rdata_r <= {31'd0, ctrl_sel};
        default: rdata_r <= 32'd0;
      endcase
    end else if (rvalid_r && s_axi_rready) rvalid_r <= 1'b0;
  end

  // ---- packed outputs to phase_bank ----
  genvar g;
  generate for (g=0; g<4; g=g+1) begin: pk
    assign a_bank0[g*COEFF_WIDTH +: COEFF_WIDTH] = ca[g][0];
    assign b_bank0[g*COEFF_WIDTH +: COEFF_WIDTH] = cb[g][0];
    assign a_bank1[g*COEFF_WIDTH +: COEFF_WIDTH] = ca[g][1];
    assign b_bank1[g*COEFF_WIDTH +: COEFF_WIDTH] = cb[g][1];
  end endgenerate

  assign active_sel   = ctrl_sel;
  assign coeff_frac_o = cfg_frac;

  assign s_axi_awready = awready_r;
  assign s_axi_wready  = wready_r;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_bvalid  = bvalid_r;
  assign s_axi_arready = arready_r;
  assign s_axi_rdata   = rdata_r;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rvalid  = rvalid_r;
endmodule
