#!/usr/bin/env bash
# Network impairment experiment, run inside the Jetson container (needs
# --privileged and --net=host, which it has). Adds delay, jitter or loss to
# everything the Jetson sends on the USB link, i.e. the arm commands, then
# runs a streamed step test per condition. The netem rule is always removed
# on exit, even on Ctrl+C.
#
#   bash netem_experiment.sh [trials]
set -euo pipefail
TRIALS=${1:-15}
IFACE=$(ip -o -4 addr show | awk '/192\.168\.55\.1\//{print $2; exit}')
[ -n "$IFACE" ] || { echo "no interface with 192.168.55.1"; exit 1; }
OUT=/ws/results/netem
mkdir -p "$OUT"
clear_netem() { tc qdisc del dev "$IFACE" root 2>/dev/null || true; }
trap clear_netem EXIT
echo "interface: $IFACE, trials per condition: $TRIALS"

run() {  # label, netem args (empty = none)
  local label=$1; shift
  clear_netem
  if [ $# -gt 0 ]; then tc qdisc add dev "$IFACE" root netem "$@"; fi
  echo "=== $label: ${*:-no impairment} ==="
  tc qdisc show dev "$IFACE" | head -1
  python3 /ws/step_test_v2.py --joint panda_joint1 --step 0.1 --trials "$TRIALS" --stream \
    --label "$label" --out "$OUT/$label.json" | tail -n +1 | grep -E '^trial|"(onset|seen|rise|t90)_ms"|p50' | grep -vE '^trial' || true
}

run baseline
run delay20 delay 20ms
run delay50 delay 50ms
run jitter50 delay 50ms 20ms distribution normal
run loss5 loss 5%
run loss20 loss 20%
clear_netem

python3 - "$OUT" << 'PY'
import json, sys, glob, os
rows = []
for name in ["baseline", "delay20", "delay50", "jitter50", "loss5", "loss20"]:
    p = os.path.join(sys.argv[1], name + ".json")
    if not os.path.exists(p):
        continue
    d = json.load(open(p))
    g = lambda k, q: d.get(k, {}).get(q, float("nan"))
    rows.append((name, d["trials"], d["no_motion"], g("onset_ms", "p50"), g("onset_ms", "p95"), g("onset_ms", "max"),
                 g("seen_ms", "p50"), g("t90_ms", "p50")))
print(f"{'condition':10} {'trials':>6} {'stalled':>7} {'onset p50':>9} {'p95':>7} {'max':>7} {'seen p50':>8} {'t90 p50':>8}")
for r in rows:
    print(f"{r[0]:10} {r[1]:6d} {r[2]:7d} {r[3]:9.1f} {r[4]:7.1f} {r[5]:7.1f} {r[6]:8.1f} {r[7]:8.1f}")
PY
