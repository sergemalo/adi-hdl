#!/usr/bin/env bash
# B3 — BFW4 register file and full-chain simulations, with negative controls.
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
printf "%-26s %-12s %-7s %s\n" build testbench expect result
row() {   # name tb expect flag files...
  local name="$1" tb="$2" expect="$3" flag="$4"; shift 4
  if ! iverilog -g2005 $flag -o "$W/$name.vvp" "$@" 2>"$W/$name.err"; then
    echo "COMPILE ERROR $name"; cat "$W/$name.err"; fail=1; return; fi
  vvp -n "$W/$name.vvp" >"$W/$name.log"
  if grep -q "RESULT: PASS" "$W/$name.log"; then got=PASS; else got=FAIL; fi
  printf "%-26s %-12s %-7s %s\n" "$name" "$tb" "$expect" "$got"
  [ "$got" = "$expect" ] || fail=1
}
row ctrl_nominal        ctrl  PASS ""                  $CTRL
row ctrl_alias_decode   ctrl  FAIL "-DMUT_ALIAS"       $CTRL
row chain_nominal       chain PASS ""                  $CHAIN
row chain_no_prime      chain FAIL "-DMUT_NO_PRIME"    $CHAIN
row chain_tx_src_ignored chain FAIL "-DMUT_SRC_IGNORED" $CHAIN
row chain_fifo_iq_swap  chain FAIL "-DMUT_FIFO_IQ"     $CHAIN
echo "logs: $W"
[ $fail -eq 0 ] && echo "B3: ALL EXPECTATIONS MET" || echo "B3: EXPECTATION VIOLATED"
exit $fail
