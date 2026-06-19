# experiments/theorem_12_violin.py
#
# Theorem 1 & 2 — Synthetic Worst-Case Workload
#
# Three plots:
#   Plot 1 — Per-message max enc delay (sorted by CAN ID)
#   Plot 2 — Prediction error per message (ZS-AP only)
#   Plot 3 — Per-message max E2E delay (full formula)

import os, sys
from dataclasses import dataclass
from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import matplotlib.ticker as ticker
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.bus               import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator         import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge        import build_instances_from_sim
from src.tsn.gcl               import GCL
from src.tsn.flow              import Flow
from src.tsn.wcrt              import TSNConfig, wcrt_ms
from src.gateways.fifo_bf        import FIFOBatchGateway
from src.gateways.fifo_bf_sim    import simulate_fifo_bf
from src.gateways.fifo_zs        import compute_slacks, FIFOZeroSlackGateway
from src.gateways.fifo_zs_sim    import simulate_fifo_zs
from src.gateways.fifo_zs_ap     import FIFOZeroSlackAPGateway
from src.gateways.fifo_zs_ap_sim import simulate_fifo_zs_ap
from src.gateways.decap          import decap_delay
from example.evaluation     import compute_max_enc_delay, compute_e2e

# ── Workload 
PERIODS     = [5,10,20,40,80,160]
PAYLOAD     = 8
BATCH_SIZE  = 4
N_RUNS      = 200
SIM_HORIZON = 500.0

@dataclass
class Msg:
    msg_id:       str
    period:       float
    deadline:     float
    priority:     int
    payload_size: int
    source_id:    int = 0

# Inverse RM: longest period = priority 1 (highest)
msgset = [
    Msg(
       msg_id = f"{i+1:X}F",
        period       = float(T),
        deadline     = float(T),
        priority     = i + 1,
        payload_size = PAYLOAD,
    )
    for i, T in enumerate(sorted(PERIODS, reverse=True))
]

can_cfg = CANBusConfig(bus_type="CAN", tbit=0.002)
C_src   = compute_tx_times([m.payload_size for m in msgset], can_cfg)
R_src   = compute_response_times(
    [m.priority for m in msgset],
    [m.period   for m in msgset],
    [m.payload_size for m in msgset], can_cfg)
R_dst   = list(R_src)

tsn_cfg    = TSNConfig(link_speed_mbps=1000, num_switches=1,
                       switch_processing_us=3.0, propagation_delay_us=1.0)
gcl        = GCL.sample_uniform(cycle_us=1000, window_us=200)
wc_payload = 9 + BATCH_SIZE * PAYLOAD
tsn_wcrt   = wcrt_ms(
    Flow(0, min(m.period for m in msgset), wc_payload, 0), [], gcl, tsn_cfg)
delta_dec  = decap_delay(BATCH_SIZE, max(C_src))
S          = compute_slacks(msgset, R_src, C_src, tsn_wcrt, delta_dec, R_dst)

gw_bf = FIFOBatchGateway(msgset, R_src, C_src, BATCH_SIZE, tsn_wcrt, max(C_src))
gw_zs = FIFOZeroSlackGateway(msgset, R_src, C_src, S, BATCH_SIZE, tsn_wcrt, max(C_src))
gw_ap = FIFOZeroSlackAPGateway(msgset, R_src, C_src, S, BATCH_SIZE, tsn_wcrt, max(C_src))

print("Workload (sorted by CAN ID = priority):")
sort_order = sorted(range(len(msgset)), key=lambda i: msgset[i].priority)
print(f"  {'msg':>5}  {'T':>6}  {'prio':>5}  {'C':>8}  {'R':>8}  {'S':>10}")
for i in sort_order:
    m = msgset[i]
    print(f"  {m.msg_id:>5}  {m.period:>6.0f}  {m.priority:>5}  "
          f"{C_src[i]:>8.4f}  {R_src[i]:>8.4f}  {S[i]:>10.4f}")

print(f"\nAnalytical bounds (n={BATCH_SIZE}):")
print(f"  FIFO-BF     delta_enc = {gw_bf.delta_enc:.4f} ms")
print(f"  FIFO-ZS     delta_enc = {gw_zs.delta_enc:.4f} ms")
print(f"  FIFO-ZS-AP  delta_enc in [{gw_ap.delta_enc_lower:.4f}, "
      f"{gw_ap.delta_enc_upper:.4f}] ms")

# ── Simulation   
nodes = [NodeConfig(0, "ECU0", [
    MessageConfig(m.priority, m.period, m.payload_size,
                  m.deadline, name=m.msg_id)
    for m in msgset])]

max_enc_bf   = defaultdict(float)
max_enc_zs   = defaultdict(float)
max_enc_ap   = defaultdict(float)
pred_log_all = []

print(f"\nRunning {N_RUNS} simulation runs...")
for run in range(N_RUNS):
    sim       = CANBusSimulator(nodes, can_cfg, SIM_HORIZON)
    done      = sim.run()
    instances = build_instances_from_sim(done, use_arrival="finish")
    if not instances:
        continue

    batches_bf = simulate_fifo_bf(instances, BATCH_SIZE)
    batches_zs = simulate_fifo_zs(instances, S, BATCH_SIZE)
    batches_ap, pred_log = simulate_fifo_zs_ap(
        instances, S, R_src, C_src, BATCH_SIZE,
        return_prediction_log=True)

    pred_log_all.extend(pred_log)

    for b in batches_bf:
        for inst in b.instances:
            fid = int(inst.flow_id)
            enc = b.fwd_time - float(inst.arrive_gw)
            max_enc_bf[fid] = max(max_enc_bf[fid], enc)

    for b in batches_zs:
        for inst in b.instances:
            fid = int(inst.flow_id)
            enc = b.fwd_time - float(inst.arrive_gw)
            max_enc_zs[fid] = max(max_enc_zs[fid], enc)

    for b in batches_ap:
        for inst in b.instances:
            fid = int(inst.flow_id)
            enc = b.fwd_time - float(inst.arrive_gw)
            max_enc_ap[fid] = max(max_enc_ap[fid], enc)

    if (run+1) % 50 == 0:
        print(f"  {run+1}/{N_RUNS}")

# ── Print summary   
print(f"\nPer-message max enc delay (ms):")
print(f"  {'msg':>5}  {'BF_obs':>8}  {'BF_bnd':>8}  "
      f"{'ZS_obs':>8}  {'ZS_bnd':>8}  "
      f"{'AP_obs':>8}  {'AP_bnd':>8}")
print("  " + "-" * 62)
for i in sort_order:
    m = msgset[i]
    print(f"  {m.msg_id:>5}  {max_enc_bf.get(i,0):>8.4f}  {gw_bf.delta_enc:>8.4f}  "
          f"{max_enc_zs.get(i,0):>8.4f}  {S[i]:>8.4f}  "
          f"{max_enc_ap.get(i,0):>8.4f}  {S[i]:>8.4f}")

# compute E2E from observed max enc delay via paper formula
def e2e_from_enc(enc_map):
    rows = compute_e2e(msgset, R_src, C_src, R_dst, enc_map, tsn_wcrt, delta_dec)
    return [r["e2e"] for r in rows]

e2e_bf = e2e_from_enc(max_enc_bf)
e2e_zs = e2e_from_enc(max_enc_zs)
e2e_ap = e2e_from_enc(max_enc_ap)

print(f"\nPer-message max E2E delay (ms):")
print(f"  {'msg':>5}  {'D':>8}  {'BF_e2e':>8}  {'ZS_e2e':>8}  "
      f"{'AP_e2e':>8}  {'miss_BF':>8}")
print("  " + "-" * 58)
for i in sort_order:
    m = msgset[i]
    miss = "✗" if e2e_bf[i] > m.deadline + 1e-9 else "✓"
    print(f"  {m.msg_id:>5}  {m.deadline:>8.0f}  {e2e_bf[i]:>8.4f}  "
          f"{e2e_zs[i]:>8.4f}  {e2e_ap[i]:>8.4f}  {miss:>8}")

# prediction error
by_flow = defaultdict(list)
for p in pred_log_all:
    by_flow[p["flow_id"]].append(p)

print(f"\nPrediction error summary:")
print(f"  {'msg':>5}  {'max_err':>9}  {'bound':>8}  {'ok':>4}")
for i in sort_order:
    preds = by_flow.get(i, [])
    if not preds: continue
    max_err = max(p["error"] for p in preds)
    bound   = preds[0]["bound"]
    ok      = "✓" if all(p["ok"] for p in preds) else "✗"
    print(f"  {msgset[i].msg_id:>5}  {max_err:>9.4f}  {bound:>8.4f}  {ok:>4}")

# ── Plot settings   
plt.rcParams.update({
    "font.size":       25,
    "axes.titlesize":  25,
    "axes.labelsize":  25,
    "xtick.labelsize": 25,
    "ytick.labelsize": 25,
    "legend.fontsize": 25,
})

COLORS = {
    "FIFO-BF":    "#2ecc71",
    "FIFO-ZS":    "#ff890b",
    "FIFO-ZS-AP": "#3498db",
}

msg_labels = [msgset[i].msg_id for i in sort_order]
x          = np.arange(len(msgset))

  
# PLOT 1 — Max enc delay
  
bf_obs_vals = [max_enc_bf.get(i, 0.0) for i in sort_order]
zs_obs_vals = [max_enc_zs.get(i, 0.0) for i in sort_order]
ap_obs_vals = [max_enc_ap.get(i, 0.0) for i in sort_order]

fig, ax = plt.subplots(figsize=(13, 6))
for vals, label, marker in [
    (bf_obs_vals, "FIFO-BF",    "v"),
    (zs_obs_vals, "FIFO-ZS",    "^"),
    (ap_obs_vals, "FIFO-ZS-AP", "P"),
]:
    color = COLORS[label]
    line, = ax.plot(x, vals, label=label, color=color,
                    linestyle="-", linewidth=3.5,
                    marker=marker, markersize=7)
    line.set_path_effects([
        pe.Stroke(linewidth=4, foreground="white"), pe.Normal()])



  
# PLOT 2 — Prediction error
  
pred_labels = []
max_errors  = []
per_bounds  = []

for i in sort_order:
    preds = by_flow.get(i, [])
    if not preds:
        continue
    pred_labels.append(msgset[i].msg_id)
    max_errors.append(max(p["error"] for p in preds))
    per_bounds.append(2.0 * (R_src[i] - C_src[i]))

x2 = np.arange(len(pred_labels))

fig, ax = plt.subplots(figsize=(12, 4))
line1, = ax.plot(x2, per_bounds, color="#e74c3c", linestyle="--",
                 linewidth=3.5, zorder=3,
                 label="Theoretical bound")
line1.set_path_effects([
    pe.Stroke(linewidth=4, foreground="white"), pe.Normal()])

line2, = ax.plot(x2, max_errors, color="#3498db", linestyle="-",
                 linewidth=3.5, zorder=4,
                 label="Max observed prediction lateness")
line2.set_path_effects([
    pe.Stroke(linewidth=4, foreground="white"), pe.Normal()])

ax.set_xticks(x2)
ax.set_xticklabels(
    msg_labels,
    rotation=35,
    ha="right",
    rotation_mode="anchor"
)

ax.set_xlabel("CAN ID")
ax.set_ylabel("Prediction lateness (ms)")
ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=False))
ax.grid(axis="y", linestyle="--", alpha=0.4)
ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 0.93),
    ncol=2,
    frameon=False,
    fontsize=25,
    columnspacing=0.5,
    handlelength=1.0,
    prop={"weight": "550"}
)
fig.tight_layout()
plt.show()

  
# PLOT 3 — Max E2E delay (paper formula)
  
  
# PLOT 1 — Maximum encapsulation delay
  

bf_obs_vals = [max_enc_bf.get(i, 0.0) for i in sort_order]
zs_obs_vals = [max_enc_zs.get(i, 0.0) for i in sort_order]
ap_obs_vals = [max_enc_ap.get(i, 0.0) for i in sort_order]

plot_styles = {
    "FIFO-BF": {
        "color": COLORS["FIFO-BF"],
        "linestyle": "-.",
        "linewidth": 3.5,
        "marker": "v",
        "zorder": 4,
    },

    "FIFO-ZS": {
        "color": COLORS["FIFO-ZS"],
        "linestyle": ":",
        "linewidth": 3.5,
        "marker": "^",
        "zorder": 7,
    },

    "FIFO-ZS-AP": {
        "color": COLORS["FIFO-ZS-AP"],
        "linestyle": "-",
        "linewidth": 4,
        "marker": "P",
        "zorder": 6,
    },
}

fig, ax = plt.subplots(figsize=(12, 5))

for vals, label in [
    (bf_obs_vals, "FIFO-BF"),
    (zs_obs_vals, "FIFO-ZS"),
    (ap_obs_vals, "FIFO-ZS-AP"),
]:

    s = plot_styles[label]

    line, = ax.plot(
        x,
        vals,
        label=label,
        color=s["color"],
        linestyle=s["linestyle"],
        linewidth=s["linewidth"],
        marker=s["marker"],
        markersize=7,
        zorder=s["zorder"],
    )

    line.set_path_effects([
        pe.Stroke(
            linewidth=s["linewidth"] + 0.5,
            foreground="white"
        ),
        pe.Normal()
    ])

ax.set_xticks(x)
ax.set_xticklabels(
    msg_labels,
    rotation=35,
    ha="right",
    rotation_mode="anchor"
)

ax.tick_params(axis="x", pad=1)
ax.set_xlabel("CAN ID")
ax.set_ylabel("End-to-end delay(ms)")
ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=False))
ax.grid(axis="y", linestyle="--", alpha=0.4)

ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 0.93),
    ncol=3,
    frameon=False,
    columnspacing=0.6,
    handlelength=1.2,
    prop={"weight": "medium"}
)

fig.tight_layout()
plt.show()