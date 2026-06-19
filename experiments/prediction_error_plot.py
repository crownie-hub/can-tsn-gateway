# experiments/prediction_error_plot.py
#
# Evaluation 2 — Prediction Error Bound Validation
#
# For each message instance q > 0:
#   measured error = |a_tilde_i^q - a_i^q|
#   bound          = 2 * (R_i^src - C_i)
#
# Plot: per-instance prediction error vs bound (horizontal line per message)
# Shows: all errors stay within the theoretical bound

import os, sys
from collections import defaultdict

import matplotlib
#matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "examples"))

from src.can.message     import message_set
from src.can.bus         import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator   import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge  import build_instances_from_sim
from src.tsn.gcl         import GCL
from src.tsn.flow        import Flow
from src.tsn.wcrt        import TSNConfig, wcrt_ms
from src.gateways.fifo_zs        import compute_slacks
from src.gateways.fifo_zs_ap_sim import simulate_fifo_zs_ap, print_prediction_error_log
from src.gateways.decap          import decap_delay

BATCH_SIZE  = 5
SIM_HORIZON = 7000.0

msgset     = message_set()
priorities = [m.priority     for m in msgset]
periods    = [m.period       for m in msgset]
payloads   = [m.payload_size for m in msgset]

can_cfg = CANBusConfig(bus_type="CAN", tbit=0.002)
C_src   = compute_tx_times(payloads, can_cfg)
R_src   = compute_response_times(priorities, periods, payloads, can_cfg)
R_dst   = list(R_src)

tsn_cfg    = TSNConfig(link_speed_mbps=1000, num_switches=1,
                       switch_processing_us=3.0, propagation_delay_us=1.0)
gcl        = GCL.sample_uniform(cycle_us=1000, window_us=200)
wc_payload = 9 + sum(sorted(payloads, reverse=True)[:BATCH_SIZE])
tsn_wcrt   = wcrt_ms(Flow(0, min(periods), wc_payload, 0), [], gcl, tsn_cfg)
delta_dec  = decap_delay(BATCH_SIZE, max(C_src))
S          = compute_slacks(msgset, R_src, C_src, tsn_wcrt, delta_dec, R_dst)

grouped = defaultdict(list)
for m in msgset:
    grouped[m.source_id].append(MessageConfig(
        msg_id=m.priority, period=m.period,
        payload_bytes=m.payload_size, deadline=m.deadline, name=m.msg_id))
nodes = [NodeConfig(src, f"ECU{src}", msgs) for src, msgs in sorted(grouped.items())]

sim       = CANBusSimulator(nodes, can_cfg, SIM_HORIZON)
done      = sim.run()
instances = build_instances_from_sim(done, use_arrival="finish")

_, pred_log = simulate_fifo_zs_ap(
    instances, S, R_src, C_src, BATCH_SIZE,
    return_prediction_log=True)

print(f"Total predictions logged: {len(pred_log)}")

# per-message stats
by_msg = defaultdict(list)
for p in pred_log:
    by_msg[p["flow_id"]].append(p)

print(f"\nPer-message prediction error vs bound:")
print(f"  {'msg':>5}  {'n_pred':>7}  {'max_err':>9}  {'bound':>8}  {'ok':>4}")
print("  " + "-" * 42)
all_ok = True
for i, m in enumerate(msgset):
    preds = by_msg.get(i, [])
    if not preds:
        continue
    max_err = max(p["error"] for p in preds)
    bound   = preds[0]["bound"]
    ok      = all(p["ok"] for p in preds)
    if not ok: all_ok = False
    print(f"  {m.msg_id:>5}  {len(preds):>7}  {max_err:>9.4f}  "
          f"{bound:>8.4f}  {'✓' if ok else '✗':>4}")
print(f"\n  All within bound: {'✓' if all_ok else '✗'}")



import matplotlib.patheffects as pe
import matplotlib.ticker as ticker

plt.rcParams.update({
    "font.size": 25, "axes.titlesize": 25,
    "axes.labelsize": 25, "xtick.labelsize": 25,
    "ytick.labelsize": 25, "legend.fontsize": 23,
})

# sort by CAN ID (priority) for display
sort_order = sorted(range(len(msgset)), key=lambda i: msgset[i].priority)

# per-flow: max observed error and theoretical bound (sorted)
msg_labels  = []
max_errors  = []
per_bounds  = []

for i in sort_order:
    m     = msgset[i]
    preds = by_msg.get(i, [])
    if not preds:
        continue
    msg_labels.append(m.msg_id)
    max_errors.append(max(p["error"] for p in preds))
    per_bounds.append(2.0 * (R_src[i] - C_src[i]))

x = np.arange(len(msg_labels))

fig, ax = plt.subplots(figsize=(13, 6))


# per-flow theoretical bound 2*(R_i - C_i)
line1, = ax.plot(x, per_bounds, color="#e74c3c", linestyle="--",
                 linewidth=3.5, zorder=3,
                 label=r"Theoretical bound")
line1.set_path_effects([
    pe.Stroke(linewidth=4, foreground="white"), pe.Normal()])


# max observed error — consistent with theorem3 validation plot
line2, = ax.plot(x, max_errors, color="#3498db", linestyle="-",
                 linewidth=3.5, zorder=4,
                 label="Max observed prediction lateness")
line2.set_path_effects([
    pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])

ax.set_xticks(x)
ax.set_xticklabels(msg_labels, rotation=45, ha="right")
ax.set_xlabel("CAN ID")
ax.set_ylabel("Prediction lateness (ms)")
#ax.set_title("Prediction Error Bound Validation  (Lemma 1)")
ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=False))
ax.grid(axis="y", linestyle="--", alpha=0.4)
ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 0.93),
    ncol=2,
    frameon=False,
    columnspacing=0.6,
    handlelength=2.0,
    prop={"weight": "550"}
)
fig.tight_layout()
plt.show()