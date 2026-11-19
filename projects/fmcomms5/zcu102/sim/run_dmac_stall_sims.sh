#!/usr/bin/env bash
# ARGUS-97: RX and TX DMA tolerance to a memory stall, before and after the
# buffer change in ../system_bd.tcl (FIFO_SIZE 8 -> 32, MAX_BYTES_PER_BURST
# 128 -> 2048: 1 KiB -> 64 KiB). tb_dmac_stall.v (RX: samples lost) and
# tb_dmac_tx_stall.v (TX: the DAC starved), the real axi_dmac top.
#
#   ./run_dmac_stall_sims.sh          (from anywhere; needs Vivado 2026.1)
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

DEPS="$LIB/axi_dmac/*.v $LIB/common/up_axi.v $LIB/util_axis_fifo/util_axis_fifo.v
  $LIB/util_axis_fifo/util_axis_fifo_address_generator.v $LIB/util_cdc/*.v
  $LIB/common/ad_mem_asym.v $LIB/common/ad_mem.v"

W="$(mktemp -d -t dmac_stall.XXXXXX)"
fail=0

row() {     # name tb fifo_size max_bytes stall_ns expect [pattern the log must contain]
  local name="$1" tb="$2" fs="$3" mb="$4" st="$5" expect="$6" pat="${7:-}" d got note
  d="$W/$name"; mkdir -p "$d"
  # shellcheck disable=SC2086
  if ! ( cd "$d" && xvlog -d FIFO_SIZE="$fs" -d MAX_BYTES="$mb" -d STALL_NS="$st" \
           "$SIM/$tb.v" $DEPS >xvlog.log 2>&1 \
         && xelab "$tb" -s s >xelab.log 2>&1 && xsim s -R >run.log 2>&1 ); then
    echo "BUILD ERROR $name (see $d)"; fail=1; return
  fi
  if grep -q "RESULT: PASS" "$d/run.log"; then got=PASS; else got=FAIL; fi
  note="$( (grep -m1 -E "first (overflow|starved)" "$d/run.log" \
            || grep -m1 -E "words written|frames played" "$d/run.log") | sed 's/^ *//' | cut -c1-80)"
  printf "%-22s %-6s %-6s %s\n" "$name" "$expect" "$got" "$note"
  [ "$got" = "$expect" ] || fail=1
  if [ -n "$pat" ] && ! grep -q -- "$pat" "$d/run.log"; then
    echo "  ^ expected the log to contain: $pat"; fail=1; fi
}

printf "%-22s %-6s %-6s %s\n" run expect got note
# Negative controls first: a stall longer than the buffer must FAIL.
row rx_neg_1k_stall1us   tb_dmac_stall    8  128  1000 FAIL "first overflow"
row rx_neg_64k_stall70us tb_dmac_stall    32 2048 70000 FAIL "first overflow"
row tx_neg_1k_stall0.9us tb_dmac_tx_stall 8  128  900  FAIL "first starved"
row tx_neg_64k_stall70us tb_dmac_tx_stall 32 2048 70000 FAIL "first starved"
# Today's buffer (1 KiB): clean without a stall, holds 0.5 us.
row rx_1k_clean          tb_dmac_stall    8  128  0     PASS "BYTES_PER_BURST_WIDTH): 7"
row rx_1k_stall0.5us     tb_dmac_stall    8  128  500   PASS
row tx_1k_clean          tb_dmac_tx_stall 8  128  0     PASS "BYTES_PER_BURST_WIDTH): 7"
row tx_1k_stall0.5us     tb_dmac_tx_stall 8  128  500   PASS
# The new buffer (64 KiB): clean, holds 60 us. Register 0x010 reads 11.
row rx_64k_clean         tb_dmac_stall    32 2048 0     PASS "BYTES_PER_BURST_WIDTH): 11"
row rx_64k_stall60us     tb_dmac_stall    32 2048 60000 PASS
row tx_64k_clean         tb_dmac_tx_stall 32 2048 0     PASS "BYTES_PER_BURST_WIDTH): 11"
row tx_64k_stall60us     tb_dmac_tx_stall 32 2048 60000 PASS

echo "logs: $W"
[ $fail -eq 0 ] && echo "DMAC STALL: ALL EXPECTATIONS MET" || echo "DMAC STALL: EXPECTATION VIOLATED"
exit $fail
