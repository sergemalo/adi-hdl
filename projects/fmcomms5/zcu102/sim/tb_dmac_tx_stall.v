// ARGUS-97: how long can the TX DMA's memory side stall before the DAC starves?
//
// The mirror of tb_dmac_stall.v. The TX DMA reads ahead into the same
// FIFO_SIZE x MAX_BYTES_PER_BURST buffer (1 KiB in fmcomms5_bd.tcl). If memory
// reads stall longer than the buffer lasts, util_upack2 asks for a frame the
// DMA does not have: the DAC plays stale data (an underrun inside a block).
//
// The real axi_dmac top with the TX DMA's parameters (fmcomms5_bd.tcl): AXI4
// source, 64 bits at 250 MHz (sys_dma_clk); AXI-Stream destination, 128 bits
// at divclk = 61.44 MHz; CYCLIC 1 (the parameter; transfers here are not
// cyclic, as for streaming playback). Driver as tb_dmac_stall.v: enable, then
// 64 KiB transfers from consecutive addresses, the next one queued.
//
// Memory model: word w at byte address 8w reads w. ARREADY and RVALID random
// (7 of 8 cycles), read latency 200 ns, both held low for STALL_NS from
// 150 us on. Consumer: m_axis_ready every divclk once the first frame came
// (util_upack2 takes one 128-bit frame per divclk at 61.44 MSPS 2R2T).
//
// PASS: no starved cycle (ready without valid), every frame is {w+1, w}
// following the previous one, enough frames, register 0x010 bits 19:16 read
// log2 of the burst size.
//
// Defines (xvlog -d): FIFO_SIZE, MAX_BYTES (MAX_BYTES_PER_BURST), STALL_NS.

`timescale 1ns/1ps

`ifndef FIFO_SIZE
  `define FIFO_SIZE 8
`endif
`ifndef MAX_BYTES
  `define MAX_BYTES 128
`endif
`ifndef STALL_NS
  `define STALL_NS 0
`endif

module tb_dmac_tx_stall;

  localparam FIFO_SIZE = `FIFO_SIZE;
  localparam MAX_BYTES = `MAX_BYTES;
  localparam real STALL_NS = `STALL_NS;
  localparam real T_STALL = 150000.0;
  localparam real T_END = T_STALL + STALL_NS + 150000.0;
  localparam LATENCY = 50;                  // sys_dma_clk cycles: 200 ns
  localparam XFER_BYTES = 65536;
  localparam BASE = 32'h1000_0000;

  reg s_clk = 1'b0;
  reg d_clk = 1'b0;
  reg f_clk = 1'b0;
  always #5.0 s_clk = ~s_clk;
  always #2.0 d_clk = ~d_clk;
  always #8.138 f_clk = ~f_clk;

  reg resetn = 1'b0;
  initial begin
    #200 resetn = 1'b1;
  end

  // --- AXI-Lite master (as tb_dmac_stall.v) ----------------------------------
  reg  [10:0] s_awaddr = 0;
  reg         s_awvalid = 0;
  wire        s_awready;
  reg  [31:0] s_wdata = 0;
  reg         s_wvalid = 0;
  wire        s_wready;
  wire        s_bvalid;
  reg  [10:0] s_araddr = 0;
  reg         s_arvalid = 0;
  wire        s_arready;
  wire        s_rvalid;
  wire [31:0] s_rdata;

  task wr(input [10:0] a, input [31:0] d);
    begin
      @(posedge s_clk);
      s_awaddr <= a; s_awvalid <= 1'b1; s_wdata <= d; s_wvalid <= 1'b1;
      fork
        begin @(posedge s_clk); while (!s_awready) @(posedge s_clk); s_awvalid <= 1'b0; end
        begin @(posedge s_clk); while (!s_wready) @(posedge s_clk); s_wvalid <= 1'b0; end
      join
      while (!s_bvalid) @(posedge s_clk);
    end
  endtask

  task rd(input [10:0] a, output [31:0] d);
    begin
      @(posedge s_clk);
      s_araddr <= a; s_arvalid <= 1'b1;
      @(posedge s_clk); while (!s_arready) @(posedge s_clk);
      s_arvalid <= 1'b0;
      while (!s_rvalid) @(posedge s_clk);
      d = s_rdata;
    end
  endtask

  // --- AXI read slave (the memory) -------------------------------------------
  wire [31:0] araddr;
  wire [7:0]  arlen;
  wire        arvalid;
  wire        rready;
  reg         rnd_ar = 1'b1;
  reg         rnd_r = 1'b1;
  reg         stall = 1'b0;
  wire        arready = rnd_ar && !stall;

  reg [31:0]  q_addr [0:255];
  reg [8:0]   q_len [0:255];
  integer     q_due [0:255];
  integer     cyc = 0;                     // a counter, not $realtime: a
                                           // continuous assignment is only
                                           // re-evaluated when an operand changes
  reg [7:0]   q_wr = 0;
  reg [7:0]   q_rd = 0;
  reg [8:0]   beat = 0;
  wire        q_ready = q_wr != q_rd && cyc >= q_due[q_rd];
  wire        rvalid = q_ready && rnd_r && !stall;
  wire [63:0] rdata = (q_addr[q_rd] - BASE) / 8 + beat;
  wire        rlast = beat == q_len[q_rd];

  always @(posedge d_clk) begin
    cyc <= cyc + 1;
    rnd_ar <= ($random & 7) != 0;
    rnd_r <= ($random & 7) != 0;
    stall <= ($realtime >= T_STALL) && ($realtime < T_STALL + STALL_NS);
    if (arvalid && arready) begin
      q_addr[q_wr] <= araddr;
      q_len[q_wr] <= arlen;
      q_due[q_wr] <= cyc + LATENCY;
      q_wr <= q_wr + 1;
    end
    if (rvalid && rready) begin
      if (rlast) begin
        beat <= 0;
        q_rd <= q_rd + 1;
      end else begin
        beat <= beat + 1;
      end
    end
  end

  // --- consumer: one frame per divclk once the first frame came --------------
  wire [127:0] m_axis_data;
  wire         m_axis_valid;
  reg          sink_on = 1'b0;
  wire         m_axis_ready = 1'b1;
  reg  [63:0]  expect_lo = 0;
  integer      frames = 0;
  integer      jumps = 0;
  integer      starved = 0;
  real         first_starved = -1.0;

  always @(posedge f_clk) begin
    if (m_axis_valid) begin
      if (sink_on && m_axis_data[63:0] !== expect_lo) begin
        jumps = jumps + 1;
        if (jumps <= 10)
          $display("  JUMP at %.3f us: word %0d, expected %0d",
                   $realtime / 1000.0, m_axis_data[63:0], expect_lo);
      end
      if (m_axis_data[127:64] !== m_axis_data[63:0] + 1) begin
        jumps = jumps + 1;
        if (jumps <= 10)
          $display("  BAD FRAME at %.3f us: %h", $realtime / 1000.0, m_axis_data);
      end
      expect_lo = m_axis_data[63:0] + 2;
      frames = frames + 1;
      sink_on = 1'b1;
    end else if (sink_on && $realtime < T_END - 20000.0) begin
      starved = starved + 1;                  // the DAC wanted a frame
      if (first_starved < 0.0) first_starved = $realtime;
    end
  end

  // --- the DMA ---------------------------------------------------------------
  axi_dmac #(
    .ID(0),
    .DMA_TYPE_SRC(0),
    .DMA_TYPE_DEST(1),
    .CYCLIC(1),
    .AXI_SLICE_SRC(0),
    .AXI_SLICE_DEST(0),
    .DMA_2D_TRANSFER(0),
    .DMA_DATA_WIDTH_SRC(64),
    .DMA_DATA_WIDTH_DEST(128),
    .CACHE_COHERENT(1),
    .DMA_AXI_ADDR_WIDTH(32),
    .FIFO_SIZE(FIFO_SIZE),
    .MAX_BYTES_PER_BURST(MAX_BYTES),
    .ALLOW_ASYM_MEM(1)                 // set by library/axi_dmac/bd/bd.tcl in a block design
  ) dut (
    .s_axi_aclk(s_clk),
    .s_axi_aresetn(resetn),
    .s_axi_awvalid(s_awvalid),
    .s_axi_awaddr(s_awaddr),
    .s_axi_awready(s_awready),
    .s_axi_awprot(3'b0),
    .s_axi_wvalid(s_wvalid),
    .s_axi_wdata(s_wdata),
    .s_axi_wstrb(4'hf),
    .s_axi_wready(s_wready),
    .s_axi_bvalid(s_bvalid),
    .s_axi_bresp(),
    .s_axi_bready(1'b1),
    .s_axi_arvalid(s_arvalid),
    .s_axi_araddr(s_araddr),
    .s_axi_arready(s_arready),
    .s_axi_arprot(3'b0),
    .s_axi_rvalid(s_rvalid),
    .s_axi_rready(1'b1),
    .s_axi_rresp(),
    .s_axi_rdata(s_rdata),
    .irq(),
    .sync(1'b1),

    .m_dest_axi_aclk(1'b0),
    .m_dest_axi_aresetn(1'b0),

    .m_src_axi_aclk(d_clk),
    .m_src_axi_aresetn(resetn),
    .m_src_axi_arready(arready),
    .m_src_axi_arvalid(arvalid),
    .m_src_axi_araddr(araddr),
    .m_src_axi_arlen(arlen),
    .m_src_axi_arsize(),
    .m_src_axi_arburst(),
    .m_src_axi_arprot(),
    .m_src_axi_arcache(),
    .m_src_axi_rdata(rdata),
    .m_src_axi_rready(rready),
    .m_src_axi_rvalid(rvalid),
    .m_src_axi_rresp(2'b00),
    .m_src_axi_rid(1'b0),
    .m_src_axi_rlast(rlast),
    .m_src_axi_awready(1'b0),
    .m_src_axi_wready(1'b0),
    .m_src_axi_bvalid(1'b0),
    .m_src_axi_bresp(2'b00),
    .m_src_axi_bid(1'b0),

    .m_sg_axi_aclk(1'b0),
    .m_sg_axi_aresetn(1'b0),

    .m_axis_aclk(f_clk),
    .m_axis_ready(m_axis_ready),
    .m_axis_valid(m_axis_valid),
    .m_axis_data(m_axis_data),
    .m_axis_last(),
    .m_axis_xfer_req());

  // --- the driver ------------------------------------------------------------
  reg [31:0] r;
  integer    burst_width;
  integer    xfers = 0;
  reg [31:0] src = BASE;

  initial begin
    @(posedge resetn);
    repeat (10) @(posedge s_clk);
    rd(11'h010, r);
    burst_width = r[19:16];
    wr(11'h400, 32'h1);                         // CONTROL: enable
    forever begin
      wr(11'h414, src);                         // SRC_ADDRESS
      wr(11'h418, XFER_BYTES - 1);              // X_LENGTH
      wr(11'h40c, 32'h0);                       // FLAGS: not cyclic
      wr(11'h408, 32'h1);                       // TRANSFER_SUBMIT
      xfers = xfers + 1;
      src = src + XFER_BYTES;
      r = 1;
      while (r[0]) rd(11'h408, r);
    end
  end

  initial begin
    if (!(STALL_NS == 0.0))
      $display("stall: %.1f us from %.1f us", STALL_NS / 1000.0, T_STALL / 1000.0);
    #(T_END);
    $display("axi_dmac FIFO_SIZE %0d x MAX_BYTES_PER_BURST %0d = %0d bytes of buffering",
             FIFO_SIZE, MAX_BYTES, FIFO_SIZE * MAX_BYTES);
    $display("register 0x010 bits 19:16 (BYTES_PER_BURST_WIDTH): %0d", burst_width);
    $display("transfers submitted %0d, frames played %0d, jumps %0d, starved cycles %0d",
             xfers, frames, jumps, starved);
    if (first_starved >= 0.0)
      $display("first starved cycle at %.3f us (%.3f us into the stall)",
               first_starved / 1000.0, (first_starved - T_STALL) / 1000.0);
    // one frame per divclk (16.276 ns) after the 2 us start-up: 7/8 at the least
    if (starved == 0 && jumps == 0 && frames > (T_END - 2000.0) / 16.276 * 7 / 8
        && burst_width == $clog2(MAX_BYTES))
      $display("RESULT: PASS");
    else
      $display("RESULT: FAIL");
    $finish;
  end

endmodule
