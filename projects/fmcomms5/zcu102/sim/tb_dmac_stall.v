// ARGUS-97: how long can the RX DMA's memory side stall before samples are lost?
//
// On the board at 61.44 MSPS (r97_rung1e R4) the RX overflow flag set while
// the reader was never behind, and 1 to 10 samples went missing inside DMA
// blocks: the DMA dropped samples on its way to memory. axi_dmac buffers
// FIFO_SIZE bursts of MAX_BYTES_PER_BURST bytes between its FIFO input and
// its AXI write port; fmcomms5_bd.tcl left them at 8 x 128 B = 1 KiB, about
// 1 us at 983 MB/s. A longer stall of the memory side overflows it.
//
// The real axi_dmac top with the RX DMA's parameters (fmcomms5_bd.tcl):
// FIFO source, 128 bits at divclk = 61.44 MHz (one frame of 8 x 16 bits per
// cycle, as util_cpack2 delivers at 61.44 MSPS 2R2T); AXI4 destination,
// 64 bits at 250 MHz (sys_dma_clk); SYNC_TRANSFER_START 1; AXI_SLICE_SRC 1.
// Programmed over AXI-Lite as the driver does: enable, then 64 KiB transfers
// submitted back to back (the next one queued while the current one runs).
//
// Stimulus: fifo_wr_en every divclk from the first transfer on, as the ADC
// path never pauses. Frame k is {2k+1, 2k} (two 64-bit words), so the memory
// sees 0, 1, 2, ... A dropped frame is a jump.
// Memory model: AWREADY and WREADY random (7 of 8 cycles: contention), and
// both low for STALL_NS from 150 us on (the stall under test).
//
// PASS: no overflow cycle, every written word follows the previous one,
// enough words written, and register 0x010 (INTERFACE_DESCRIPTION_1) bits
// 19:16 read log2 of the burst size (the board reads the same register to
// tell the bitstreams apart).
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

module tb_dmac_stall;

  localparam FIFO_SIZE = `FIFO_SIZE;
  localparam MAX_BYTES = `MAX_BYTES;
  localparam real STALL_NS = `STALL_NS;
  localparam real T_STALL = 150000.0;
  localparam real T_END = T_STALL + STALL_NS + 150000.0;
  localparam XFER_BYTES = 65536;

  // clocks: s_axi 100 MHz, sys_dma_clk 250 MHz, divclk 61.44 MHz
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

  // --- AXI-Lite master -----------------------------------------------------
  reg  [10:0] s_awaddr = 0;
  reg         s_awvalid = 0;
  wire        s_awready;
  reg  [31:0] s_wdata = 0;
  reg         s_wvalid = 0;
  wire        s_wready;
  wire        s_bvalid;
  reg         s_bready = 1'b1;
  reg  [10:0] s_araddr = 0;
  reg         s_arvalid = 0;
  wire        s_arready;
  wire        s_rvalid;
  wire [31:0] s_rdata;
  reg         s_rready = 1'b1;

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

  // --- AXI write slave (the memory) ------------------------------------------
  wire [31:0] awaddr;
  wire [7:0]  awlen;
  wire        awvalid;
  wire [63:0] wdata;
  wire [7:0]  wstrb;
  wire        wvalid;
  wire        wlast;
  wire        bready;
  reg         rnd_aw = 1'b1;
  reg         rnd_w = 1'b1;
  reg         stall = 1'b0;
  wire        awready = rnd_aw && !stall;
  wire        wready = rnd_w && !stall;
  reg  [15:0] b_pending = 0;
  wire        bvalid = b_pending != 0;

  always @(posedge d_clk) begin
    rnd_aw <= ($random & 7) != 0;
    rnd_w <= ($random & 7) != 0;
    stall <= ($realtime >= T_STALL) && ($realtime < T_STALL + STALL_NS);
    b_pending <= b_pending + (wvalid && wready && wlast) - (bvalid && bready);
  end

  reg [63:0] expect_w = 0;
  reg        started = 1'b0;
  integer    words = 0;
  integer    jumps = 0;
  always @(posedge d_clk) begin
    if (wvalid && wready) begin
      words = words + 1;
      if (wstrb !== 8'hff) begin
        jumps = jumps + 1;
        $display("  %0t: wstrb %h", $time, wstrb);
      end
      if (started && wdata !== expect_w) begin
        jumps = jumps + 1;
        if (jumps <= 10)
          $display("  JUMP at %.3f us: word %0d, expected %0d (%0d frames lost)",
                   $realtime / 1000.0, wdata, expect_w, (wdata - expect_w) / 2);
      end
      started = 1'b1;
      expect_w = wdata + 1;
    end
  end

  // --- source: one frame per divclk, from the first transfer on --------------
  reg          src_on = 1'b0;
  reg  [62:0]  k = 0;
  wire         fifo_wr_en = src_on;
  wire [127:0] fifo_wr_din = {k, 1'b1, k, 1'b0};
  wire         fifo_wr_overflow;
  wire         fifo_wr_xfer_req;
  integer      overflow_cycles = 0;
  real         first_overflow = -1.0;

  always @(posedge f_clk) begin
    if (fifo_wr_xfer_req) src_on <= 1'b1;
    if (src_on) k <= k + 1;
    if (src_on && fifo_wr_overflow) begin
      overflow_cycles = overflow_cycles + 1;
      if (first_overflow < 0.0) first_overflow = $realtime;
    end
  end

  // --- the DMA ---------------------------------------------------------------
  axi_dmac #(
    .ID(0),
    .DMA_TYPE_SRC(2),
    .DMA_TYPE_DEST(0),
    .CYCLIC(0),
    .SYNC_TRANSFER_START(1),
    .AXI_SLICE_SRC(1),
    .AXI_SLICE_DEST(0),
    .DMA_2D_TRANSFER(0),
    .DMA_DATA_WIDTH_SRC(128),
    .DMA_DATA_WIDTH_DEST(64),
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
    .s_axi_bready(s_bready),
    .s_axi_arvalid(s_arvalid),
    .s_axi_araddr(s_araddr),
    .s_axi_arready(s_arready),
    .s_axi_arprot(3'b0),
    .s_axi_rvalid(s_rvalid),
    .s_axi_rready(s_rready),
    .s_axi_rresp(),
    .s_axi_rdata(s_rdata),
    .irq(),
    .sync(1'b1),

    .m_dest_axi_aclk(d_clk),
    .m_dest_axi_aresetn(resetn),
    .m_dest_axi_awaddr(awaddr),
    .m_dest_axi_awlen(awlen),
    .m_dest_axi_awsize(),
    .m_dest_axi_awburst(),
    .m_dest_axi_awprot(),
    .m_dest_axi_awcache(),
    .m_dest_axi_awvalid(awvalid),
    .m_dest_axi_awready(awready),
    .m_dest_axi_wdata(wdata),
    .m_dest_axi_wstrb(wstrb),
    .m_dest_axi_wready(wready),
    .m_dest_axi_wvalid(wvalid),
    .m_dest_axi_wlast(wlast),
    .m_dest_axi_bvalid(bvalid),
    .m_dest_axi_bresp(2'b00),
    .m_dest_axi_bready(bready),
    .m_dest_axi_bid(1'b0),
    .m_dest_axi_arready(1'b0),
    .m_dest_axi_rvalid(1'b0),
    .m_dest_axi_rresp(2'b00),
    .m_dest_axi_rdata(64'b0),
    .m_dest_axi_rid(1'b0),
    .m_dest_axi_rlast(1'b0),

    .m_src_axi_aclk(1'b0),
    .m_src_axi_aresetn(1'b0),
    .m_sg_axi_aclk(1'b0),
    .m_sg_axi_aresetn(1'b0),

    .fifo_wr_clk(f_clk),
    .fifo_wr_en(fifo_wr_en),
    .fifo_wr_din(fifo_wr_din),
    .fifo_wr_overflow(fifo_wr_overflow),
    .fifo_wr_xfer_req(fifo_wr_xfer_req));

  // --- the driver: enable, then transfers back to back ------------------------
  reg [31:0] r;
  integer    burst_width;
  integer    xfers = 0;
  reg [31:0] dest = 32'h1000_0000;

  initial begin
    @(posedge resetn);
    repeat (10) @(posedge s_clk);
    rd(11'h010, r);
    burst_width = r[19:16];
    wr(11'h400, 32'h1);                         // CONTROL: enable
    forever begin
      wr(11'h410, dest);                        // DEST_ADDRESS
      wr(11'h418, XFER_BYTES - 1);              // X_LENGTH
      wr(11'h40c, 32'h0);                       // FLAGS
      wr(11'h408, 32'h1);                       // TRANSFER_SUBMIT
      xfers = xfers + 1;
      dest = dest + XFER_BYTES;
      r = 1;
      while (r[0]) rd(11'h408, r);              // queued: wait for its start
    end
  end

  initial begin
    if (!(STALL_NS == 0.0))
      $display("stall: %.1f us from %.1f us", STALL_NS / 1000.0, T_STALL / 1000.0);
    #(T_END);
    $display("axi_dmac FIFO_SIZE %0d x MAX_BYTES_PER_BURST %0d = %0d bytes of buffering",
             FIFO_SIZE, MAX_BYTES, FIFO_SIZE * MAX_BYTES);
    $display("register 0x010 bits 19:16 (BYTES_PER_BURST_WIDTH): %0d", burst_width);
    $display("transfers submitted %0d, words written %0d, jumps %0d, overflow cycles %0d",
             xfers, words, jumps, overflow_cycles);
    if (first_overflow >= 0.0)
      $display("first overflow at %.3f us (%.3f us into the stall)",
               first_overflow / 1000.0, (first_overflow - T_STALL) / 1000.0);
    // 983 MB/s for T_END less the 1 us of start-up: 7/8 of it at the least
    if (overflow_cycles == 0 && jumps == 0 && words > (T_END - 1000.0) * 0.983 / 8 * 7 / 8
        && burst_width == $clog2(MAX_BYTES))
      $display("RESULT: PASS");
    else
      $display("RESULT: FAIL");
    $finish;
  end

endmodule
