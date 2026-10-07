#!/usr/bin/env bash
# Network impairment experiment, run on the LAPTOP (the Jetson's kernel has
# no netem). Impairs both directions of the USB link:
#   laptop -> Jetson (joint states): netem on the USB interface egress
#   Jetson -> laptop (arm commands): ingress redirected to ifb0, netem there
# The step test runs on the Jetson, driven over Wi-Fi SSH so the control
# channel itself is not impaired. Rules are always removed on exit.
#
#   sudo -v && bash netem_experiment_v2.sh steps [trials]   # condition sweep
#   sudo -v && bash netem_experiment_v2.sh linkloss [seconds] # timed total cut
set -euo pipefail
MODE=${1:-steps}
JETSON_WIFI=${JETSON_WIFI:-192.168.4.83}
IF=$(ip -o -4 addr show | awk '/192\.168\.55\.100\//{print $2; exit}')
[ -n "$IF" ] || { echo "no interface with 192.168.55.100 (USB link down?)"; exit 1; }
OUT=$HOME/teleop-ik-stack/docs/results/netem
mkdir -p "$OUT"

setup_ifb() {
  sudo modprobe ifb numifbs=1
  sudo ip link set dev ifb0 up
  sudo tc qdisc add dev "$IF" handle ffff: ingress 2>/dev/null || true
  sudo tc filter replace dev "$IF" parent ffff: protocol all prio 1 u32 match u32 0 0 \
    action mirred egress redirect dev ifb0
}
clear_all() {
  sudo tc qdisc del dev "$IF" root 2>/dev/null || true
  sudo tc qdisc del dev ifb0 root 2>/dev/null || true
  sudo tc qdisc del dev "$IF" ingress 2>/dev/null || true
}
trap clear_all EXIT
impair() {  # netem args applied to both directions; none = clear
  sudo tc qdisc del dev "$IF" root 2>/dev/null || true
  sudo tc qdisc del dev ifb0 root 2>/dev/null || true
  if [ $# -gt 0 ]; then
    sudo tc qdisc add dev "$IF" root netem "$@"
    sudo tc qdisc add dev ifb0 root netem "$@"
  fi
}
jetson() {  # run a command in the Jetson container over Wi-Fi
  ssh -o BatchMode=yes "janmesh@$JETSON_WIFI" \
    "docker exec jazzy bash -c 'source /opt/ros/jazzy/setup.bash && $1'"
}

clear_all
setup_ifb
echo "USB interface: $IF (+ ifb0 for incoming), Jetson control over $JETSON_WIFI"

if [ "$MODE" = steps ]; then
  TRIALS=${2:-15}
  run() {  # label, extra step-test flags, netem args...
    local label=$1 flags=$2; shift 2
    impair "$@"
    echo "=== $label: ${*:-no impairment} (each direction) ==="
    jetson "python3 /ws/step_test_v2.py --joint panda_joint1 --step 0.1 --trials $TRIALS $flags --label $label --out /ws/results/netem_$label.json > /dev/null && cat /ws/results/netem_$label.json" \
      > "$OUT/$label.json"
    python3 -c "import json;d=json.load(open('$OUT/$label.json'));print('  onset p50', d['onset_ms'].get('p50'), 'p95', d['onset_ms'].get('p95'), '| stalled', d['no_motion'])"
  }
  run baseline_single ""
  run baseline "--stream"
  run delay20 "--stream" delay 20ms
  run delay50 "--stream" delay 50ms
  run jitter50 "--stream" delay 50ms 20ms distribution normal
  run loss5 "--stream" loss 5%
  run loss20 "--stream" loss 20%
  impair
  python3 - "$OUT" << 'PY'
import json, os, sys
names = ["baseline_single", "baseline", "delay20", "delay50", "jitter50", "loss5", "loss20"]
print(f"\n{'condition':16} {'trials':>6} {'stalled':>7} {'onset p50':>9} {'p95':>7} {'max':>7} {'seen p50':>8} {'t90 p50':>8}")
for n in names:
    p = os.path.join(sys.argv[1], n + ".json")
    if not os.path.exists(p):
        continue
    d = json.load(open(p))
    g = lambda k, q: d.get(k, {}).get(q, float("nan"))
    print(f"{n:16} {d['trials']:6d} {d['no_motion']:7d} {g('onset_ms','p50'):9.1f} {g('onset_ms','p95'):7.1f} "
          f"{g('onset_ms','max'):7.1f} {g('seen_ms','p50'):8.1f} {g('t90_ms','p50'):8.1f}")
PY
elif [ "$MODE" = linkloss ]; then
  SECS=${2:-2}
  echo "cutting the link in 3 s for $SECS s; teleop must be engaged and moving"
  sleep 3
  T0=$(date +%s.%N)
  impair loss 100%
  sleep "$SECS"
  impair
  T1=$(date +%s.%N)
  echo "cut start $T0, restored $T1" | tee "$OUT/linkloss_times.txt"
else
  echo "usage: $0 steps [trials] | linkloss [seconds]"; exit 1
fi
