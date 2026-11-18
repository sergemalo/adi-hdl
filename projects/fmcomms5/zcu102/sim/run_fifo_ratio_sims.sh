#!/usr/bin/env bash
# ARGUS-91 step 6 (b): util_wfifo (RX) and util_upack2 + util_rfifo (TX) at the
# 2R2T clock ratio (divclk = l_clk / 4, one sample per 4 l_clk), with a sweep
# of divclk skew and sample phase, and negative controls.
#
#   ./run_fifo_ratio_sims.sh          (from anywhere; needs Vivado 2026.1)
#
# Each row: a build, the expected verdict, the verdict. Exit 0 only if every
# expectation holds. Logs stay in the printed temp directory.
set -u
cd "$(dirname "$0")"
SIM="$PWD"
LIB="$(cd ../../../../library && pwd)"
VIVADO_SETTINGS="$HOME/tools/Xilinx/2026.1/Vivado/settings64.sh"
[ -f "$VIVADO_SETTINGS" ] || { echo "Vivado 2026.1 not found at $VIVADO_SETTINGS"; exit 2; }
# shellcheck disable=SC1090
source "$VIVADO_SETTINGS" >/dev/null
GLBL="$XILINX_VIVADO/data/verilog/src/glbl.v"

RX_FILES="$SIM/tb_wfifo_ratio.v $LIB/util_wfifo/util_wfifo.v $LIB/common/ad_mem.v
  $LIB/xilinx/util_clkdiv/util_clkdiv.v $GLBL"
TX_FILES="$SIM/tb_rfifo_ratio.v $LIB/util_rfifo/util_rfifo.v
  $LIB/util_pack/util_upack2/util_upack2.v $LIB/util_pack/util_upack2/util_upack2_impl.v
  $LIB/util_pack/util_pack_common/pack_shell.v $LIB/util_pack/util_pack_common/pack_ctrl.v
  $LIB/util_pack/util_pack_common/pack_interconnect.v $LIB/util_pack/util_pack_common/pack_network.v
  $LIB/common/util_pipeline_stage.v $LIB/common/ad_perfect_shuffle.v $LIB/common/ad_mem.v
  $LIB/xilinx/util_clkdiv/util_clkdiv.v $GLBL"

W="$(mktemp -d -t fifo_ratio.XXXXXX)"
fail=0

build() {   # snapshot top define(or "") files...
  local snap="$1" top="$2" def="$3"; shift 3
  local d="$W/$snap"; mkdir -p "$d"
  # shellcheck disable=SC2086  (def may hold several "-d" options)
  ( cd "$d" && xvlog ${def:+-d $def} $* >xvlog.log 2>&1 \
      && xelab -L unisims_ver "$top" glbl -s "$snap" >xelab.log 2>&1 ) \
    || { echo "BUILD ERROR $snap (see $d)"; fail=1; return 1; }
}

row() {     # name snapshot expect skew phase [pattern the log must contain]
  local name="$1" snap="$2" expect="$3" skew="$4" phase="$5" pat="${6:-}" got note
  ( cd "$W/$snap" && xsim "$snap" -R --testplusarg "skew=$skew" --testplusarg "phase=$phase" \
      >"$W/$name.log" 2>&1 )
  if grep -q "RESULT: PASS" "$W/$name.log"; then got=PASS; else got=FAIL; fi
  note="$( (grep -m1 -E "MISMATCH|STALLED" "$W/$name.log" || grep -m1 "samples checked" "$W/$name.log") \
           | sed 's/^ *//' | cut -c1-90)"
  printf "%-26s %-6s %-6s %s\n" "$name" "$expect" "$got" "$note"
  [ "$got" = "$expect" ] || fail=1
  if [ -n "$pat" ] && ! grep -q -- "$pat" "$W/$name.log"; then
    echo "  ^ expected the log to contain: $pat"; fail=1; fi
}

build rx      tb_wfifo_ratio ""        $RX_FILES
build rx_fast tb_wfifo_ratio NEG_FAST  $RX_FILES
build rx_skip tb_wfifo_ratio NEG_SKIP  $RX_FILES
build rx_dup  tb_wfifo_ratio NEG_DUP   $RX_FILES
build tx       tb_rfifo_ratio ""        $TX_FILES
build tx_fast  tb_rfifo_ratio NEG_FAST  $TX_FILES
build tx_stall15  tb_rfifo_ratio "NEG_STALL -d STALL_LEN=15"  $TX_FILES
build tx_stall16  tb_rfifo_ratio "NEG_STALL -d STALL_LEN=16"  $TX_FILES
build tx_stall2x8 tb_rfifo_ratio "NEG_STALL2 -d STALL_LEN=8"  $TX_FILES
build tx_skip  tb_rfifo_ratio NEG_SKIP  $TX_FILES

printf "%-26s %-6s %-6s %s\n" run expect got note
# Negative controls first: each must FAIL.
row rx_neg_fast   rx_fast  FAIL 0 0
row rx_neg_skip   rx_skip  FAIL 0 0
row rx_neg_dup    rx_dup   FAIL 0 0
row tx_neg_fast   tx_fast  FAIL 0 0
# A DMA stall: 15 beats leave the DAC stream intact (underflow only), 16 slip
# it, and two stalls of 8 add up to a slip. The note shows which.
row tx_neg_stall15  tx_stall15  FAIL 0 0 "0 lane errors, 15 underflow cycles"
row tx_neg_stall16  tx_stall16  FAIL 0 0 "MISMATCH"
row tx_neg_stall2x8 tx_stall2x8 FAIL 0 0 "MISMATCH"

row tx_neg_skip   tx_skip  FAIL 0 0
# Sweep: skew over one l_clk period (4 ns) in 0.5 ns steps, every sample phase.
for sk in 0 0.5 1 1.5 2 2.5 3 3.5; do for ph in 0 1 2 3; do
  row "rx_skew${sk}_phase${ph}" rx PASS "$sk" "$ph"
done; done
for sk in 0 0.5 1 1.5 2 2.5 3 3.5; do for ph in 0 1 2 3; do
  row "tx_skew${sk}_phase${ph}" tx PASS "$sk" "$ph"
done; done

echo "logs: $W"
[ $fail -eq 0 ] && echo "FIFO RATIO: ALL EXPECTATIONS MET" || echo "FIFO RATIO: EXPECTATION VIOLATED"
exit $fail
