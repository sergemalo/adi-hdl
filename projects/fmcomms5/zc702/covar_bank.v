// =============================================================================
// covar_bank.v -- 4x4 Hermitian covariance-matrix accumulator, triple-buffered.
//
// STATUS: DRAFT, simulated against the covar_golden.py Python model but NOT
// yet synthesized, timed, or run on hardware.
//
// NO RESET PORT -- confirmed against chan_delay.v (ports: clk, valid only;
// "every register here is valid-enabled"). This module follows the same
// convention: no dedicated reset network. All state regs carry an `initial`
// value of 0 (synthesizable -- Vivado maps this to the register's power-on/
// config INIT value, not a reset net, matching how the FPGA bitstream
// actually initializes registers on real hardware). The only run-time
// "clear" mechanism is ENABLE's 0->1 transition (enable_rising), which
// already forces every accumulator/counter to a clean state -- see below.
//
// `valid` strobe: confirmed present at this convention (chan_delay.v gates
// every register update on `valid`); accumulate = valid && enable_sync below
// follows the identical pattern.
//
// Datapath: RX -> iq_override -> fir_bank -> phase_bank -> covar_bank (HERE)
// Tap point: post-phase_bank, plain s16 two's-complement samples (no
// fractional/Q interpretation -- see covar_golden.py header for the full
// derivation).
//
// MACC decomposition: every R entry is built from independent single-product
// accumulators (one multiply, one DSP48E1 in native P<=P+A*B accumulate mode,
// per accumulator), combined into the final 16 Hermitian values only once per
// block (negligible cost, not timing-critical). For NUM_CH=4 this is:
//   2*NUM_CH               = 8  diagonal-support accumulators (I_ch^2, Q_ch^2)
//   4*(NUM_CH*(NUM_CH-1)/2)= 24 off-diagonal-support accumulators
//   -----------------------------------------------------------------
//   32 total single-term accumulators (32 DSP48E1 slices in MACC mode)
//
// Per-accumulator worst case: product of two s16 values bounds at 2^30
// magnitude; accumulating N_SAMPLES=30720 terms adds ceil(log2(30720))=15
// bits -> 2^45 worst-case magnitude, fitting DSP48E1's native 48-bit P
// register with 3 bits of headroom, no external cascade needed.
// Combined R entries (sum/difference of two partials): worst case 2^46
// magnitude -> 47-bit signed, fits the 64-bit bank storage word with margin.
//
// CDC contract for the (not-yet-written) axi_covar_ctrl.v wrapper:
//   - enable_raw:  UNSYNCHRONIZED input from the AXI-lite domain. Synchronized
//                  HERE (2-FF), matching the fir_bank.v active_sel precedent
//                  documented in fir_regmap_ch.py. The wrapper just drives the
//                  raw ENABLE register bit through, no synchronization needed
//                  on that side.
//   - block_done_toggle: single bit, toggles once per completed block. SAFE
//                  to 2-FF synchronize directly in the AXI-lite domain (single
//                  -bit CDC is always safe). latest_complete_bank/block_seq
//                  are plain binary, held stable well before/after the toggle
//                  edge -- the wrapper must only sample them AFTER detecting a
//                  synchronized toggle edge, never poll them directly. (A
//                  mod-3 Gray code was considered and rejected: standard
//                  Gray-code single-bit-transition adjacency does not hold at
//                  the 2->0 wraparound of a non-power-of-2 counter.)
//   - rd_data:     wide combinational read mux, wrapper-addressed via
//                  rd_bank_sel/rd_word_sel. Protected structurally (not by a
//                  handshake): a completed bank is never rewritten for a full
//                  2 block-periods (triple buffering), so it is safe to
//                  register through a single clk_axi flop with no risk of
//                  reading a torn value. No toggle-gating needed for this
//                  port specifically (unlike the status registers above).
// =============================================================================
// TIMING PIPELINE (added after the first real synthesis run reported
// WNS = -2.019 ns on clk_div_sel_1_s, with ~2200 failing endpoints, ALL of
// them bank_mem_reg[*]/D and 15-17 logic levels deep).
//
// Root cause was here, not in phase_bank (whose output flops merely LAUNCH
// the failing paths): the original bank-write path was, in ONE cycle,
//     din -> multiply -> 48b accumulate-add -> 4-way mux -> 64b combine-add
//         -> bank_mem/D
// The second (combine) adder sat in series after the multiply-accumulate
// because the bank write used the pre-register `next_*` value. The MACC
// itself would close; the extra 64-bit adder is what broke it.
//
// Now split into 4 stages. Latency grows by 3 sample-beats (irrelevant --
// only the SET of samples in a block matters, and that is preserved
// exactly), while every stage becomes a short flop-to-flop hop:
//
//   A  din_r        <= din                      (absorbs phase_bank->covar
//                                                routing; ~4.4ns of the old
//                                                path was net delay alone)
//   B  prod_r       <= din_r * din_r            (DSP48E1 M register)
//   C  acc          <= acc + prod_r             (DSP48E1 P accumulate)
//      fin          <= <completed acc>          at block end
//   D  bank_mem     <= fin_x + fin_y            ONE adder, from flops only
//
// valid / enable_sync are delayed alongside the data so the same N samples
// land in the same block: alignment is preserved, not approximated.
// Bit-exactness re-verified against covar_golden.py at N=64 and N=30720
// after this restructuring.
// =============================================================================

`timescale 1ns/100ps

module covar_bank #(
    parameter DATA_WIDTH     = 16,
    parameter NUM_CH         = 4,
    parameter N_SAMPLES      = 30720,
    parameter PART_ACC_WIDTH = 48,          // DSP48E1 native P-register width
    parameter OUT_WIDTH      = 64,          // bank storage word width
    parameter NUM_BANKS      = 3
) (
    input  wire                          clk,
    input  wire                          enable_raw,   // unsynchronized, from AXI-lite domain
    input  wire                          valid,        // per-sample strobe (matches chan_delay.v convention)
    // Real-lane inputs, matching phase_bank_0's own din_0..din_7 port
    // naming (this module receives phase_bank_0's dout_0..dout_7 directly,
    // per fmcomms5_bd.tcl's per-lane ad_connect convention -- NOT a
    // flattened bus like the AXI-control coefficient interfaces use).
    // Lane->channel map (matches phase_regmap.set_channel_dc(), "PROVEN"
    // in the P4 pattern rung): channel c -> lanes 2c (I), 2c+1 (Q).
    input  wire signed [DATA_WIDTH-1:0] din_0, din_1, din_2, din_3,
                                          din_4, din_5, din_6, din_7,

    // Status (sample-clock domain; see CDC contract above)
    output reg                           block_done_toggle = 1'b0,
    output reg  [1:0]                    latest_complete_bank = 2'd0,
    output reg  [31:0]                   block_seq = 32'd0,

    // Narrow readback port (wrapper decodes AXI address into these)
    input  wire [1:0]                    rd_bank_sel,
    input  wire [4:0]                    rd_word_sel,   // 0..2*NUM_VALUES-1
    output wire [31:0]                   rd_data
);

    localparam NUM_PAIRS  = NUM_CH * (NUM_CH - 1) / 2;
    localparam NUM_VALUES = NUM_CH + 2 * NUM_PAIRS;   // 16 for NUM_CH=4

    // Port list above is fixed at 8 real lanes (din_0..din_7) to match the
    // board's actual, current 4-complex-channel FMCOMMS5 datapath and the
    // established per-lane scalar port convention -- not parameterized like
    // the internal MACC generation below. Halt loudly rather than silently
    // building an accumulator for a channel count the ports can't carry.
    generate
      if (NUM_CH != 4) begin: g_num_ch_guard
        NUM_CH_must_be_4_to_match_fixed_din_port_list _bad_num_ch ();
      end
    endgenerate

    // -------------------------------------------------------------------
    // Per-channel unpack: direct assigns from the named ports (not a
    // generate loop -- din_0..din_7 are distinct ports, not an array).
    // -------------------------------------------------------------------
    wire signed [DATA_WIDTH-1:0] i_ch [0:NUM_CH-1];
    wire signed [DATA_WIDTH-1:0] q_ch [0:NUM_CH-1];
    assign i_ch[0] = din_0;  assign q_ch[0] = din_1;
    assign i_ch[1] = din_2;  assign q_ch[1] = din_3;
    assign i_ch[2] = din_4;  assign q_ch[2] = din_5;
    assign i_ch[3] = din_6;  assign q_ch[3] = din_7;
    // -------------------------------------------------------------------
    // enable_raw -> enable_sync (2-FF synchronizer)
    // -------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg enable_meta = 1'b0, enable_sync = 1'b0;
    always @(posedge clk) begin
        enable_meta <= enable_raw;
        enable_sync <= enable_meta;
    end

    // -------------------------------------------------------------------
    // STAGE A: register the incoming lanes, and delay valid/enable with
    // them so a sample and its own enable/valid stay aligned. This flop
    // also terminates the long phase_bank -> covar_bank route (~4.4ns of
    // net delay in the failing report) as its own flop-to-flop hop.
    // -------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] i_a [0:NUM_CH-1];
    reg signed [DATA_WIDTH-1:0] q_a [0:NUM_CH-1];
    reg valid_a = 1'b0, enable_a = 1'b0, enable_a_d = 1'b0;

    integer ai;
    initial for (ai = 0; ai < NUM_CH; ai = ai + 1) begin
        i_a[ai] = {DATA_WIDTH{1'b0}};
        q_a[ai] = {DATA_WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        i_a[0] <= din_0;  q_a[0] <= din_1;
        i_a[1] <= din_2;  q_a[1] <= din_3;
        i_a[2] <= din_4;  q_a[2] <= din_5;
        i_a[3] <= din_6;  q_a[3] <= din_7;
        valid_a    <= valid;
        enable_a   <= enable_sync;
        enable_a_d <= enable_a;
    end

    wire enable_rising_a = enable_a & ~enable_a_d;
    wire accumulate_a    = valid_a & enable_a;

    // -------------------------------------------------------------------
    // STAGE B: register the products (DSP48E1 M register). Carries the
    // control flags forward so they stay aligned with their own product.
    // -------------------------------------------------------------------
    reg valid_b = 1'b0, rising_b = 1'b0;
    always @(posedge clk) begin
        valid_b  <= accumulate_a;
        rising_b <= enable_rising_a;
    end

    // -------------------------------------------------------------------
    // STAGE C: sample counter / block boundary, evaluated on the STAGE B
    // flags so it counts products, not raw inputs.
    // -------------------------------------------------------------------
    reg [$clog2(N_SAMPLES)-1:0] sample_cnt = 0;
    wire [$clog2(N_SAMPLES)-1:0] effective_index = rising_b ? {$clog2(N_SAMPLES){1'b0}} : sample_cnt;
    wire block_done_c = valid_b && (effective_index == N_SAMPLES - 1);

    always @(posedge clk) begin
        if (rising_b)
            sample_cnt <= valid_b ? (block_done_c ? {$clog2(N_SAMPLES){1'b0}}
                                                  : {{($clog2(N_SAMPLES)-1){1'b0}}, 1'b1})
                                  : {$clog2(N_SAMPLES){1'b0}};
        else if (valid_b)
            sample_cnt <= block_done_c ? 0 : sample_cnt + 1'b1;
    end

    // -------------------------------------------------------------------
    // Bank storage + write pointer.
    // write_ptr advances at block_done_c; write_ptr_d holds the bank whose
    // data is written one cycle later (STAGE D). Status registers update at
    // block_done_d, i.e. the SAME edge the data lands -- so software can
    // never observe a bank marked complete before its contents are valid.
    // ENABLE's rising edge does NOT touch write_ptr/block_seq/
    // latest_complete_bank: a restart resets only the sample counter and
    // in-flight accumulators, not the bank round-robin or block history.
    // -------------------------------------------------------------------
    reg signed [OUT_WIDTH-1:0] bank_mem [0:NUM_BANKS-1][0:NUM_VALUES-1];

    integer bi, vi;
    initial begin
      for (bi = 0; bi < NUM_BANKS; bi = bi + 1)
        for (vi = 0; vi < NUM_VALUES; vi = vi + 1)
          bank_mem[bi][vi] = {OUT_WIDTH{1'b0}};
    end

    reg [1:0] write_ptr = 0, write_ptr_d = 0;
    reg       block_done_d = 1'b0;

    always @(posedge clk) begin
        block_done_d <= block_done_c;
        if (block_done_c) begin
            write_ptr_d <= write_ptr;
            write_ptr   <= (write_ptr == NUM_BANKS - 1) ? 0 : write_ptr + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (block_done_d) begin
            latest_complete_bank <= write_ptr_d;
            block_seq            <= block_seq + 1'b1;
            block_done_toggle    <= ~block_done_toggle;
        end
    end

    // -------------------------------------------------------------------
    // Diagonal accumulators: I_ch^2, Q_ch^2  (2*NUM_CH DSPs)
    // -------------------------------------------------------------------
    genvar ch;
    generate
        for (ch = 0; ch < NUM_CH; ch = ch + 1) begin : g_diag
            reg signed [2*DATA_WIDTH-1:0] prod_ii_b = 0, prod_qq_b = 0;
            always @(posedge clk) begin
                prod_ii_b <= i_a[ch] * i_a[ch];
                prod_qq_b <= q_a[ch] * q_a[ch];
            end

            reg signed [PART_ACC_WIDTH-1:0] acc_ii = 0, acc_qq = 0;

            // Standard clear / hold / accumulate form -- DSP48E1 maps this
            // to OPMODE-controlled P feedback. rising_b selects "start from
            // this product" so the first sample of a block is never dropped.
            wire signed [PART_ACC_WIDTH-1:0] next_ii =
                rising_b ? (valid_b ? $signed(prod_ii_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                         : (valid_b ? acc_ii + prod_ii_b : acc_ii);
            wire signed [PART_ACC_WIDTH-1:0] next_qq =
                rising_b ? (valid_b ? $signed(prod_qq_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                         : (valid_b ? acc_qq + prod_qq_b : acc_qq);

            reg signed [PART_ACC_WIDTH-1:0] fin_ii = 0, fin_qq = 0;

            always @(posedge clk) begin
                acc_ii <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_ii;
                acc_qq <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_qq;
                if (block_done_c) begin
                    fin_ii <= next_ii;      // completed block, incl. this sample
                    fin_qq <= next_qq;
                end
            end

            // STAGE D: one adder, both operands straight off flops.
            always @(posedge clk)
                if (block_done_d)
                    bank_mem[write_ptr_d][ch] <= fin_ii + fin_qq;
        end
    endgenerate

    // -------------------------------------------------------------------
    // Off-diagonal accumulators: 4 per pair (i,j), i<j
    //   Re(R_ij) = I_i*I_j + Q_i*Q_j
    //   Im(R_ij) = Q_i*I_j - I_i*Q_j
    // Value-index mapping matches covar_golden.REGMAP_FIELD_ORDER exactly.
    // -------------------------------------------------------------------
    function integer pair_index;
        input integer i, j;
        begin
            pair_index = i*NUM_CH - i*(i+1)/2 + (j-i-1);
        end
    endfunction

    genvar gi, gj;
    generate
        for (gi = 0; gi < NUM_CH - 1; gi = gi + 1) begin : g_pi
            for (gj = gi + 1; gj < NUM_CH; gj = gj + 1) begin : g_pj
                localparam VRE = NUM_CH + 2 * pair_index(gi, gj);
                localparam VIM = VRE + 1;

                reg signed [2*DATA_WIDTH-1:0] prod_re1_b = 0, prod_re2_b = 0,
                                              prod_im1_b = 0, prod_im2_b = 0;
                always @(posedge clk) begin
                    prod_re1_b <= i_a[gi] * i_a[gj];   // Ii*Ij
                    prod_re2_b <= q_a[gi] * q_a[gj];   // Qi*Qj
                    prod_im1_b <= q_a[gi] * i_a[gj];   // Qi*Ij
                    prod_im2_b <= i_a[gi] * q_a[gj];   // Ii*Qj
                end

                reg signed [PART_ACC_WIDTH-1:0] acc_re1 = 0, acc_re2 = 0,
                                                acc_im1 = 0, acc_im2 = 0;

                wire signed [PART_ACC_WIDTH-1:0] next_re1 =
                    rising_b ? (valid_b ? $signed(prod_re1_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                             : (valid_b ? acc_re1 + prod_re1_b : acc_re1);
                wire signed [PART_ACC_WIDTH-1:0] next_re2 =
                    rising_b ? (valid_b ? $signed(prod_re2_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                             : (valid_b ? acc_re2 + prod_re2_b : acc_re2);
                wire signed [PART_ACC_WIDTH-1:0] next_im1 =
                    rising_b ? (valid_b ? $signed(prod_im1_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                             : (valid_b ? acc_im1 + prod_im1_b : acc_im1);
                wire signed [PART_ACC_WIDTH-1:0] next_im2 =
                    rising_b ? (valid_b ? $signed(prod_im2_b) : $signed({PART_ACC_WIDTH{1'b0}}))
                             : (valid_b ? acc_im2 + prod_im2_b : acc_im2);

                reg signed [PART_ACC_WIDTH-1:0] fin_re1 = 0, fin_re2 = 0,
                                                fin_im1 = 0, fin_im2 = 0;

                always @(posedge clk) begin
                    acc_re1 <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_re1;
                    acc_re2 <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_re2;
                    acc_im1 <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_im1;
                    acc_im2 <= block_done_c ? $signed({PART_ACC_WIDTH{1'b0}}) : next_im2;
                    if (block_done_c) begin
                        fin_re1 <= next_re1;  fin_re2 <= next_re2;
                        fin_im1 <= next_im1;  fin_im2 <= next_im2;
                    end
                end

                // STAGE D: one adder each, both operands straight off flops.
                always @(posedge clk) begin
                    if (block_done_d) begin
                        bank_mem[write_ptr_d][VRE] <= fin_re1 + fin_re2;
                        bank_mem[write_ptr_d][VIM] <= fin_im1 - fin_im2;
                    end
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------
    // Narrow readback mux: word_sel selects lo/hi 32-bit half of one value.
    // (word_sel = 2*value_index + {0=lo,1=hi})
    // -------------------------------------------------------------------
    wire [OUT_WIDTH-1:0] rd_value = bank_mem[rd_bank_sel][rd_word_sel[4:1]];
    assign rd_data = rd_word_sel[0] ? rd_value[63:32] : rd_value[31:0];

endmodule
