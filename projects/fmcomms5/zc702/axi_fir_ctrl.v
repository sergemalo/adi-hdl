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
//   0x080  BANK0[0]  RW   COEFF_WIDTH bits, zero-extended on read
//   0x084  BANK0[1]  RW
//   0x088  BANK0[2]  RW
//   ...    BANK0[k]  RW   at 0x080 + 4*k, for k < NUM_COEFF
//   0x100  BANK1[0]  RW
//   0x104  BANK1[1]  RW
//   0x108  BANK1[2]  RW
//   ...    BANK1[k]  RW   at 0x100 + 4*k, for k < NUM_COEFF
//
// Each bank is a 32-word (128-byte) aligned block, so the tap index is a
// 5-bit field (up_waddr[4:0]) -> up to 32 taps/bank; NUM_COEFF picks how many
// are live. Banks were re-spaced from the original 16-word blocks (bank0 at
// 0x040, bank1 at 0x080) to make room for 21-tap operation. The old 16-word
// window at 0x040 is now unmapped.
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
//
// =====================================================================
//  VERILOG BASICS  (quick reference for the constructs used below)
// =====================================================================
//
//  The golden rule: some Verilog constructs describe PHYSICAL hardware
//  (wires and flip-flops that exist on the FPGA), while others are
//  COMPILE-TIME ONLY -- they are consumed during "elaboration" (the step
//  before synthesis that specializes the module and unrolls loops) and
//  leave NO trace in the final netlist.  Keep the two groups separate:
//
//  ---- Compile-time only (vanish during elaboration, never hardware) ----
//   parameter    Constant set at instantiation from outside (#(...)).
//                Like a C++ template argument. Shapes the hardware
//                (sizes, counts) but is not itself hardware.
//   localparam   Same as parameter, but INTERNAL -- cannot be overridden
//                from outside. Used for fixed internal constants.
//   integer      32-bit signed procedural variable. In synthesizable RTL
//                it is almost always a for-loop counter that gets UNROLLED
//                (constant bounds) inside an always block, then discarded.
//   genvar       Loop index used ONLY in a generate block to unroll
//                STRUCTURE (assigns, instances). Purely a stencil; never
//                a signal. (integer unrolls behavior; genvar unrolls
//                structure.)
//
//  ---- Can become physical hardware ----
//   wire         A net = a physical conductor. Stores NOTHING; it only
//                carries whatever continuously drives it. Bare wire = plain
//                routing; wire = expression = routing + combinational logic.
//   reg          MISNOMER: does NOT mean "register". It is just a variable
//                assignable in procedural (always/initial) code. What it
//                BECOMES depends entirely on HOW it is assigned:
//                  - assigned in always @(posedge clk) -> flip-flops (state)
//                  - assigned in always @(*) covering all branches -> pure
//                    combinational logic (no storage)
//                  - always @(*) with a missing branch -> accidental LATCH
//                (SystemVerilog 'logic' replaces reg/wire and drops this
//                 naming trap.)
//
//  ---- Assignment styles ----
//   assign  (outside any block) = CONTINUOUS assignment: a permanent,
//           always-active connection. No clock, no trigger -- the left side
//           always tracks the right-side expression. Can only target a wire.
//   <=  inside always @(posedge clk) = CLOCKED (nonblocking) assignment:
//           samples the right side once per rising edge and HOLDS it -> flop.
//
//  ---- Other ----
//   `timescale 1ns/100ps   Simulation-only: unit / precision for '#' delays
//           in testbenches. Does NOT set clock frequencies and has ZERO
//           effect on synthesis (real clocks come from .xdc + clocking HW).
//   (* attr = "..." *)      A tool ATTRIBUTE (metadata). Hints to Vivado;
//           no effect on simulation or synthesized logic.
//   ANSI port style         Direction + width + name declared inline in the
//           header (used throughout this file). Outputs default to 'wire';
//           write 'output reg' to drive one inside an always block.
//   NOTE: this module has NO inheritance -- synthesizable Verilog has no
//   classes. It CONFORMS to AXI4-Lite by naming convention, and COMPOSES
//   ("has-a") the up_axi sub-module instantiated at the bottom.
// =====================================================================

// `timescale : SIMULATION-ONLY directive. 1ns = time unit for '#' delays,
// 100ps = rounding precision. Does not define or fix any clock; ignored by
// synthesis. (No '#' delays exist in this file, so it has no effect here.)
`timescale 1ns/100ps

// ANSI-style module header. Two parenthesized lists:
//   #( ... )  = parameters (compile-time constants, overridable from outside)
//   ( ... )   = ports (the interface: input / output / inout)
module axi_fir_ctrl #(
  // parameter : compile-time constant, resolved at elaboration. Sets sizes
  // and counts below; changing it produces a DIFFERENT netlist from the same
  // source. Not stored anywhere on the chip.
  parameter NUM_COEFF   = 3,
  parameter COEFF_WIDTH = 18
) (
  // output : a module port. By default an output is a 'wire' (net) -- it can
  // only be driven by 'assign' or by a sub-module instance, NOT inside an
  // always block. To drive an output procedurally you would write
  // 'output reg'. Width [msb:lsb] here is a parameter expression, so the port
  // size is computed at elaboration.
  // coefficient outputs to the FIR datapath (s_axi_aclk domain)
  output [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat0,
  output [(NUM_COEFF*COEFF_WIDTH)-1:0]  coeff_flat1,
  output                                active_sel,   // 1 bit, synchronized in the FIR
  output [4:0]                          coeff_frac,

  // (* ... *) : a Verilog ATTRIBUTE = metadata for the tools. This one is
  // Xilinx-specific and attaches to the s_axi_aclk port that follows: it tells
  // Vivado's block-design inference that this clock drives the 's_axi'
  // interface and pairs it with reset s_axi_aresetn, so the pile of s_axi_*
  // ports gets recognized as ONE connectable AXI interface. Zero effect on
  // simulation or synthesized logic.
  // axi4-lite slave interface
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  // input : a module port driven from outside. (AXI4-Lite is fully
  // unidirectional -- every port here is input or output, never inout.
  // inout maps to real hardware only at a physical chip pin, e.g. I2C SDA.)
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

  // localparam : like parameter but INTERNAL -- cannot be overridden from
  // outside. Compile-time constant; by synthesis the literal 0x46495230 has
  // simply been substituted in wherever ID_VALUE appears. No storage exists.
  localparam [31:0] ID_VALUE = 32'h46495230;   // "FIR0"

  // ---------------------------------------------------------------------
  // up_axi register bus (AXI_ADDRESS_WIDTH = 16 => 14-bit word address)
  // ---------------------------------------------------------------------

  // wire : a net = a physical conductor with NO memory. It just carries
  // whatever is continuously driven onto it. These declare the internal
  // register-bus signals connecting to the up_axi shim below.
  wire                    up_clk;
  wire                    up_rstn;

  wire                    up_wreq;
  wire  [13:0]            up_waddr;
  wire  [31:0]            up_wdata;
  // reg : a procedural variable (NOT necessarily a register). Because up_wack
  // is assigned inside always @(posedge up_clk) further down, it synthesizes
  // to a real flip-flop. The '= 1'b0' sets its power-up/simulation value.
  reg                     up_wack  = 1'b0;

  wire                    up_rreq;
  wire  [13:0]            up_raddr;
  reg   [31:0]            up_rdata = 32'd0;
  reg                     up_rack  = 1'b0;

  // assign (outside a block) : CONTINUOUS assignment. up_clk is always driven
  // to equal s_axi_aclk -- a permanent connection, no clock, no trigger. Here
  // the right side is a bare signal, so this is literally just a wire/alias.
  assign up_clk  = s_axi_aclk;
  assign up_rstn = s_axi_aresetn;

  // ---------------------------------------------------------------------
  // Address decode.  up_waddr is a WORD address: byte offset >> 2.
  //   bank0 : word 0x020..0x03F  (byte 0x080..0x0FC)
  //   bank1 : word 0x040..0x05F  (byte 0x100..0x17C)
  // Tap index is the low 5 bits (up_waddr[4:0]) -> up to 32 taps/bank; the
  // bank select is the next field up (up_waddr[13:5]).
  // ---------------------------------------------------------------------

  // Continuous assign with an EXPRESSION on the right side: still "always
  // driven", but now the wire carries the output of combinational logic --
  // here an equality comparator synthesized into LUTs.
  wire        wr_bank0_sel = (up_waddr[13:5] == 9'h001);
  wire        wr_bank1_sel = (up_waddr[13:5] == 9'h002);
  wire [ 4:0] wr_coeff_idx =  up_waddr[4:0];

  wire        rd_bank0_sel = (up_raddr[13:5] == 9'h001);
  wire        rd_bank1_sel = (up_raddr[13:5] == 9'h002);
  wire [ 4:0] rd_coeff_idx =  up_raddr[4:0];

  // ---------------------------------------------------------------------
  // Compile-time guard.  The 5-bit tap index caps each bank at 32 taps.
  // If NUM_COEFF ever exceeds that, taps 32+ become unaddressable and writes
  // to them silently alias -- so halt elaboration loudly instead of shipping
  // a broken map.  Referencing an undefined module inside a not-taken
  // generate branch is the Verilog-2001 way to force a self-describing hard
  // error only when the bad condition holds.
  // ---------------------------------------------------------------------
  generate
    if (NUM_COEFF > 32) begin: g_num_coeff_guard
      NUM_COEFF_exceeds_32_tap_address_map _bad_num_coeff ();
    end
  endgenerate

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------

  reg [31:0]              up_scratch     = 32'd0;
  reg [ 4:0]              up_coeff_frac  = 5'd0;
  reg                     up_active_sel  = 1'b0;
  // reg ARRAY : an array of regs -> a bank of flip-flops. up_coeff0 is
  // NUM_COEFF entries of COEFF_WIDTH bits each (3 x 18 = 54 flops). Stored in
  // flops (not BRAM) so a fully parallel FIR can read every tap in one cycle.
  reg [COEFF_WIDTH-1:0]   up_coeff0 [0:NUM_COEFF-1];
  reg [COEFF_WIDTH-1:0]   up_coeff1 [0:NUM_COEFF-1];

  // integer : a 32-bit procedural variable used purely as a for-loop counter.
  // The loops below have constant bounds, so the tool UNROLLS them and
  // consumes 'i' at elaboration -- it does NOT become hardware.
  integer i;

  // always @(posedge up_clk) : a clocked procedural block. Every reg assigned
  // here (with <=, nonblocking) becomes a FLIP-FLOP that samples on the rising
  // edge and holds the value. This is what turns 'reg' into real state.
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
            (wr_coeff_idx == i[4:0])) begin
          up_coeff0[i] <= up_wdata[COEFF_WIDTH-1:0];
        end
        if ((up_wreq == 1'b1) && (wr_bank1_sel == 1'b1) &&
            (wr_coeff_idx == i[4:0])) begin
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

  // Continuous assigns: coeff_frac / active_sel are output wires permanently
  // driven by the current value of their flip-flops (up_coeff_frac /
  // up_active_sel). They track those registers instantly, holding no state.
  assign coeff_frac = up_coeff_frac;
  assign active_sel = up_active_sel;

  // genvar + generate : STRUCTURAL loop unrolling done at elaboration. 'n' is
  // a compile-time index (never a signal). With NUM_COEFF=3 the tool stamps
  // out 3 copies of the continuous assigns below, packing each register into
  // its bit-slice of the flat output bus, then discards 'n'.
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

  // Module INSTANTIATION = COMPOSITION ("has-a"), not inheritance. This module
  // contains an instance (named i_up_axi) of ADI's up_axi sub-module, which
  // translates the raw AXI4-Lite handshake into the simpler up_wreq/up_waddr/
  // up_rreq/... register bus this file's logic uses. #(...) overrides its
  // parameter; .port(signal) connects each of its ports by name.
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
