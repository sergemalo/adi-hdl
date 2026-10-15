#!/usr/bin/env bash
# B3 rev 2 — BFW4 register file and full-chain simulations, with negative
# controls and a clock-phase sweep.
#
#   AD_HDL_DIR=~/w/adi-hdl ./run_bf_rtl_sims.sh        (run from the RTL dir)
#
# Each row: a build, the testbench that must judge it, the expected verdict.
# Exit 0 only if every expectation holds. Needs iverilog.
set -u
cd "$(dirname "$0")"
UPAXI="${AD_HDL_DIR:-$HOME/w/adi-hdl}/library/common/up_axi.v"
[ -f "$UPAXI" ] || { echo "up_axi.v not found at $UPAXI (set AD_HDL_DIR)"; exit 2; }
W="$(mktemp -d)"
CTRL="axi_bf_ctrl.v tb_axi_bf_ctrl.v $UPAXI"
CHAIN="axi_bf_ctrl.v bf_bank.v bf_dac_if.v bf_capture.v tb_bf_chain.v $UPAXI"

fail=0
printf "%-30s %-6s %-7s %-7s %s\n" build tb expect result note
row() {   # name tb expect flags plusargs files...
  local name="$1" tb="$2" expect="$3" flags="$4" plus="$5"; shift 5
  if ! iverilog -g2005 $flags -o "$W/$name.vvp" "$@" 2>"$W/$name.err"; then
    echo "COMPILE ERROR $name"; cat "$W/$name.err"; fail=1; return; fi
  vvp -n "$W/$name.vvp" $plus >"$W/$name.log"
  if grep -q "RESULT: PASS" "$W/$name.log"; then got=PASS; else got=FAIL; fi
  note="$( (grep -m1 MISMATCH "$W/$name.log" || grep -m1 -E "band|  [0-9]+ non-zero|FIFO word" "$W/$name.log") | sed "s/^ *//" | cut -c1-72)"
  printf "%-30s %-6s %-7s %-7s %s\n" "$name" "$tb" "$expect" "$got" "$note"
  [ "$got" = "$expect" ] || fail=1
}
row ctrl_nominal            ctrl  PASS ""                      "" $CTRL
row ctrl_alias_decode       ctrl  FAIL "-DMUT_ALIAS"           "" $CTRL
row ctrl_count_while_off    ctrl  FAIL "-DMUT_NO_GATE"         "" $CTRL
row ctrl_w1c_faulted_anytime ctrl FAIL "-DMUT_W1C_ANYTIME"     "" $CTRL
for sk in 0 4.9 7.5; do for r in 0 1 2 3; do
  row "chain_skew${sk}_phase${r}" chain PASS "-DSCLK_SKEW=$sk" "+rate0=$r" $CHAIN
done; done
row chain_no_priming        chain FAIL "-DMUT_NO_PRIME"        "" $CHAIN
row chain_no_rx_gap_check   chain FAIL "-DMUT_NO_GAP_CHECK"    "" $CHAIN
row chain_no_cadence_check  chain FAIL "-DMUT_NO_PERIOD_CHECK" "" $CHAIN
row chain_fifo_iq_swap      chain FAIL "-DMUT_FIFO_IQ"         "" $CHAIN
row chain_w1c_faulted_anytime chain FAIL "-DMUT_W1C_ANYTIME"   "" $CHAIN
echo "logs: $W"
[ $fail -eq 0 ] && echo "B3: ALL EXPECTATIONS MET" || echo "B3: EXPECTATION VIOLATED"
exit $fail
