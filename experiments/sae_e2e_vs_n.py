# experiments/sae_result_max_n8.py
# SAE result:
#   1. Per-message max E2E at n=5 (line plot)
#   2. Mean batch size + deadline misses at n=5 and n=10 (bar plot)
#
# E2E follows paper formula exactly:
#   e2e_i = R_src + delta_enc + R_TSN + delta_dec + R_dst - C_i

import os, sys
import math
from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import matplotlib.ticker as ticker
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "examples"))

from src.can.message           import message_set
from src.can.bus               import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator         import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge        import build_instances_from_sim
from src.tsn.gcl               import GCL
from src.tsn.flow              import Flow
from src.tsn.wcrt              import TSNConfig, wcrt_ms
from src.gateways.fifo_bf_sim      import simulate_fifo_bf
from src.gateways.fifo_zs          import compute_slacks
from src.gateways.fifo_zs_sim      import simulate_fifo_zs
from src.gateways.fifo_zs_ap_sim   import simulate_fifo_zs_ap
from src.gateways.fifo_timeout     import compute_T_tsn
from src.gateways.fifo_timeout_sim import simulate_fifo_timeout
from example.evaluation       import (compute_max_enc_delay,
                                            compute_e2e, compute_e2e_mat)
from src.gateways.decap            import decap_delay

SIM_HORIZON = 2000.0

# ── Fixed setup (same for all n) ──────────────────────────────────
msgset   = message_set()
periods  = [m.period       for m in msgset]
payloads = [m.payload_size for m in msgset]

can_cfg = CANBusConfig(bus_type="CAN", tbit=0.002)
C_src   = compute_tx_times(payloads, can_cfg)
R_src   = compute_response_times(
    [m.priority for m in msgset], periods, payloads, can_cfg)
R_dst   = list(R_src)

tsn_cfg = TSNConfig(link_speed_mbps=100, num_switches=1,
                    switch_processing_us=0.0, propagation_delay_us=1.0)
gcl     = GCL.sample_uniform(cycle_us=500, window_us=200)

def build_nodes():
    grouped = defaultdict(list)
    for m in msgset:
        grouped[m.source_id].append(MessageConfig(
            msg_id=m.priority, period=m.period,
            payload_bytes=m.payload_size, deadline=m.deadline, name=m.msg_id))
    return [NodeConfig(src, f"ECU{src}", msgs)
            for src, msgs in sorted(grouped.items())]

# ── Run experiment for a given n ──────────────────────────────────
def run_n(N):
    GW_PERIOD  = compute_T_tsn(msgset, N)          # Eq. 1
    wc_payload = 9 + sum(sorted(payloads, reverse=True)[:N])
    tsn_wcrt   = wcrt_ms(Flow(0, min(periods), wc_payload, 0), [], gcl, tsn_cfg)
    delta_dec  = decap_delay(N, max(C_src))
    S          = compute_slacks(msgset, R_src, C_src, tsn_wcrt, delta_dec, R_dst)

    sim       = CANBusSimulator(build_nodes(), can_cfg, SIM_HORIZON)
    done      = sim.run()
    instances = build_instances_from_sim(done, use_arrival="finish")
    print(f"n={N}  GW_PERIOD={GW_PERIOD}ms  "
          f"frames={len(done)}  instances={len(instances)}")

    batches_to = simulate_fifo_timeout(instances, GW_PERIOD, N)
    batches_bf = simulate_fifo_bf(instances, N)
    batches_zs = simulate_fifo_zs(instances, S, N)
    batches_ap = simulate_fifo_zs_ap(instances, S, R_src, C_src, N)

    def e2e_vals(batches):
        enc  = compute_max_enc_delay(batches)
        rows = compute_e2e(msgset, R_src, C_src, R_dst, enc, tsn_wcrt, delta_dec)
        return [r["e2e"] for r in rows]

    def e2e_vals_mat(batches):
        rows = compute_e2e_mat(msgset, R_src, C_src, R_dst, batches,
                               tsn_wcrt, max(C_src))
        return [r["e2e"] for r in rows]

    schemes = {
        "FIFO-TO + FIFO":    e2e_vals(batches_to),
        "FIFO-TO + MAT":     e2e_vals_mat(batches_to),
        "FIFO-BF + FIFO":    e2e_vals(batches_bf),
        "FIFO-BF + MAT":     e2e_vals_mat(batches_bf),
        "FIFO-ZS + FIFO":    e2e_vals(batches_zs),
        "FIFO-ZS-AP + FIFO": e2e_vals(batches_ap),
    }

    # misses per policy (count messages with any instance violating deadline)
    misses = {
        name: sum(1 for v, m in zip(vals, msgset) if v > m.deadline + 1e-9)
        for name, vals in schemes.items()
    }

    # mean batch size per source gateway
    mean_batch = {
        "FIFO-TO":    np.mean([len(b.instances) for b in batches_to]),
        "FIFO-BF":    np.mean([len(b.instances) for b in batches_bf]),
        "FIFO-ZS":    np.mean([len(b.instances) for b in batches_zs]),
        "FIFO-ZS-AP": np.mean([len(b.instances) for b in batches_ap]),
    }

    # deadline misses per source gateway (take worst of FIFO/MAT pair)
    gw_misses = {
        "FIFO-TO":    max(misses["FIFO-TO + FIFO"],
                          misses["FIFO-TO + MAT"]),
        "FIFO-BF":    max(misses["FIFO-BF + FIFO"],
                          misses["FIFO-BF + MAT"]),
        "FIFO-ZS":    misses["FIFO-ZS + FIFO"],
        "FIFO-ZS-AP": misses["FIFO-ZS-AP + FIFO"],
    }

    return schemes, mean_batch, gw_misses

# ── Run n=5 and n=10 ──────────────────────────────────────────────
schemes_5,  batch_5,  misses_5  = run_n(5)
schemes_10, batch_10, misses_10 = run_n(10)
schemes_15, batch_15, misses_15 = run_n(15)

def print_per_message_deadlines(N, schemes):

    sort_order = sorted(
        range(len(msgset)),
        key=lambda i: msgset[i].priority
    )

    print("\n" + "=" * 100)
    print(f"Per-message maximum E2E delay and deadline misses — n={N}")
    print("=" * 100)

    for scheme, vals in schemes.items():

        print(f"\n{scheme}")
        print("-" * 100)

        print(
            f"{'Msg':<10}"
            f"{'Priority':>10}"
            f"{'Max E2E (ms)':>18}"
            f"{'Deadline (ms)':>18}"
            f"{'Status':>14}"
        )

        print("-" * 100)

        for i in sort_order:

            m = msgset[i]
            e2e = vals[i]

            status = (
                "MISS"
                if e2e > m.deadline + 1e-9
                else "OK"
            )

            print(
                f"{m.msg_id:<10}"
                f"{m.priority:>10}"
                f"{e2e:>18.4f}"
                f"{m.deadline:>18.4f}"
                f"{status:>14}"
            )

        missed = [
            msgset[i].msg_id
            for i in sort_order
            if vals[i] > msgset[i].deadline + 1e-9
        ]

        if missed:
            print(f"\nMissed deadline: {', '.join(missed)}")
        else:
            print("\nMissed deadline: None")

print_per_message_deadlines(5, schemes_5)

# ── Print summary for n=5 ─────────────────────────────────────────
print(f"\n{'Overall Comparison n=5':=<65}")
print(f"  {'scheme':<28}{'mean_e2e':>12}{'worst_e2e':>14}{'misses':>10}")
print("  " + "-" * 62)
for name, vals in schemes_5.items():
    mean  = sum(vals)/len(vals)
    worst = max(vals)
    miss  = sum(1 for v,m in zip(vals,msgset) if v > m.deadline+1e-9)
    print(f"  {name:<28}{mean:>12.4f}{worst:>14.4f}{miss:>10}")

# ── Styles ────────────────────────────────────────────────────────
styles = {
    "FIFO-TO + FIFO":    {"color": "#95a5a6", "linestyle": "-",
                               "linewidth": 3.5, "marker": "s", "zorder": 2},
    "FIFO-TO + MAT":     {"color": "#040305", "linestyle": ":",
                               "linewidth": 3.5, "marker": "v", "zorder": 3},
    "FIFO-BF + FIFO":    {"color": "#2ecc71", "linestyle": "-.",
                               "linewidth": 3.5, "marker": "o", "zorder": 4},
    "FIFO-BF + MAT":     {"color": "#516C3E", "linestyle": "--",
                               "linewidth": 3.5, "marker": "x", "zorder": 6},
   "FIFO-ZS + FIFO": {
    "color": "#ff890b",
    "linestyle": ":",
    "linewidth": 3,
    "marker": "^",
    "zorder": 9
    },

    "FIFO-ZS-AP + FIFO": {
        "color": "#3498db",
        "linestyle": "--",
        "linewidth": 4,
        "marker": "P",
        "zorder": 7
    },
    }

sort_order = sorted(range(len(msgset)), key=lambda i: msgset[i].priority)
msg_labels = [msgset[i].msg_id for i in sort_order]
x          = np.arange(len(msgset))

# ── Plot 1: E2E line graph n=5 ────────────────────────────────────
plt.rcParams.update({
    "font.size": 30, "axes.titlesize": 30,
    "axes.labelsize": 30, "xtick.labelsize": 30,
    "ytick.labelsize": 30, "legend.fontsize": 25,
})

fig, ax = plt.subplots(figsize=(15, 8))
for scheme, vals in schemes_5.items():
    s    = styles[scheme]
    line, = ax.plot(x, [vals[i] for i in sort_order],
                    label=scheme, color=s["color"],
                    linestyle=s["linestyle"], linewidth=s["linewidth"],
                    marker=s["marker"], markersize=7, zorder=s["zorder"])
    line.set_path_effects([
        pe.Stroke(linewidth=s["linewidth"] + 1.5, foreground="white"),
        pe.Normal()])

ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
ax.set_xticks(x)
ax.set_xticklabels(msg_labels, rotation=45, ha="right")
ax.set_xlabel("CAN ID")
ax.set_ylabel("End-to-end delay (ms)")
ax.grid(axis="y", linestyle="--", alpha=0.4)
ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 1.02),
    ncol=3,
    frameon=False,
    columnspacing=0.6,
    handlelength=2.0,
    prop={"weight": "550"}
)


fig.tight_layout(rect=[0, 0, 0.82, 1])
fig.tight_layout()
plt.show()

# ── Plot 2: Bar graph
plt.rcParams.update({
    "font.size": 30, "axes.titlesize": 30,
    "axes.labelsize": 30, "xtick.labelsize": 30,
    "ytick.labelsize": 30, "legend.fontsize": 15,
})

policies = ["FIFO-TO", "FIFO-BF", "FIFO-ZS", "FIFO-ZS-AP"]
x_bar    = np.arange(len(policies))
width    = 0.2

fig, ax = plt.subplots(figsize=(12, 6))

# 4 bar groups: batch_n5, miss_n5, batch_n10, miss_n10
MIN_VISIBLE = 1.2  

miss5_raw  = [misses_5[p]  for p in policies]
miss10_raw = [misses_10[p] for p in policies]

def vis(vals):
    return [max(v, MIN_VISIBLE) for v in vals]

b1 = ax.bar(x_bar - 1.5*width, [batch_5[p]  for p in policies],
            width, label="Mean batch size (n=5)",
            color="#3498db", alpha=0.85)
b2 = ax.bar(
    x_bar - 0.5*width,
    miss5_raw,
    width,
    label="Deadline misses (n=5)",
    color="#e74c3c",
    alpha=0.85,
    hatch="//",
    edgecolor="white",
    linewidth=1.5,
)
b3 = ax.bar(x_bar + 0.5*width, [batch_10[p] for p in policies],
            width, label="Mean batch size (n=10)",
            color="#2ecc71", alpha=0.85)

b4 = ax.bar(
    x_bar + 1.5*width,
    miss10_raw,
    width,
    label="Deadline misses (n=10)",
    color="#e67e22",
    alpha=0.85,
    hatch="//",
    edgecolor="white",
    linewidth=1.5,
)


for bars, raw_vals in [(b2, miss5_raw), (b4, miss10_raw)]:

    for bar, val in zip(bars, raw_vals):

        if val == 0:

            ax.hlines(
                y=0,
                xmin=bar.get_x(),
                xmax=bar.get_x() + bar.get_width(),
                colors=bar.get_facecolor(),
                linewidth=5,
                zorder=6,
            )

ax.set_xticks(x_bar)
ax.set_xticklabels(policies)
ax.set_ylabel("Number")
#ax.set_title("Mean Batch Size and Deadline Misses — n=5 vs n=10")
ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
ax.set_ylim(bottom=0)
ax.grid(axis="y", linestyle="--", alpha=0.4)
ax.legend(loc="upper right")
fig.tight_layout()
plt.show()

  
# Print table: avg buffer utilization + deadline misses n=5, 10, 15
  

print("\n")
print("=" * 92)
print("Average Buffer Utilization (%) and Deadline Misses  —  n=5, n=10, n=15")
print("=" * 92)

header = (
    f"{'Policy':<15}"
    f"{'Util n=5':>12}"
    f"{'Util n=10':>13}"
    f"{'Util n=15':>13}"
    f"{'Miss n=5':>12}"
    f"{'Miss n=10':>12}"
    f"{'Miss n=15':>12}"
)

print(header)
print("-" * 92)

for p in policies:
    util5  = (batch_5[p]  / 5)  * 100
    util10 = (batch_10[p] / 10) * 100
    util15 = (batch_15[p] / 15) * 100

    miss5  = misses_5[p]
    miss10 = misses_10[p]
    miss15 = misses_15[p]

    print(
        f"{p:<15}"
        f"{util5:>12.1f}"
        f"{util10:>13.1f}"
        f"{util15:>13.1f}"
        f"{miss5:>12}"
        f"{miss10:>12}"
        f"{miss15:>12}"
    )

print("=" * 92)