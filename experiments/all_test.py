# experiments/schedulability_sweep.py
#
# Analytical schedulability evaluation for CAN-TSN gateway policies.

import os, sys, random
from dataclasses import dataclass

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.bus        import CANBusConfig, compute_tx_times, compute_response_times
from src.tsn.gcl        import GCL
from src.tsn.flow       import Flow
from src.tsn.wcrt       import TSNConfig, wcrt_ms
from src.gateways.fifo_bf      import FIFOBatchGateway
from src.gateways.fifo_zs      import compute_slacks, FIFOZeroSlackGateway
from src.gateways.fifo_zs_ap   import FIFOZeroSlackAPGateway
from src.gateways.fifo_timeout import FIFOTimeoutGateway, compute_T_tsn
from src.gateways.decap        import decap_delay
from example.evaluation        import compute_e2e

SEED          = 42
SETS_PER_UTIL = 100
UTIL_LEVELS   = [round(u * 0.1, 1) for u in range(1, 10)]
TBIT          = 0.002
DEADLINE_MULT = 1.0
PAYLOAD_RANGE = (1, 8)
PERIODS_HIGH  = [5, 10, 20, 50]
PERIODS_LOW   = [100, 200, 500]
PERIODS_LOCAL = [5, 10, 20, 50, 100, 200, 500, 1000]
TSN_LINK_MBPS = 100
TSN_SWITCHES  = 2
TSN_PROC_US   = 3.0
TSN_PROP_US   = 1.0
WINDOW_FRAC   = 0.50
DEFAULT_FWD_FRACTION = 1
POLICIES      = ["TO", "BF", "ZS", "ZS-AP"]
DEFAULT_N     = 5
MIN_BATCH_MULT = 2

@dataclass
class Msg:
    msg_id: str; period: float; deadline: float
    priority: int; payload_size: int; band: int; source_id: int = 0

def generate_banded_msgset(target_util, can_cfg, rng, fwd_fraction=0.4):
    msgs_raw = []; util_total = 0.0
    u_fwd = target_util * fwd_fraction
    u_local = target_util * (1 - fwd_fraction)

    u1 = u_fwd * 0.6; u_acc = 0.0
    while u_acc < u1:
        T = rng.choice(PERIODS_HIGH); payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]; contrib = C / T
        if u_acc + contrib > u1 * 1.1: break
        msgs_raw.append((T, payload, 1)); u_acc += contrib; util_total += contrib
        if len(msgs_raw) > 200: break

    u3 = u_fwd * 0.4; u_acc = 0.0
    while u_acc < u3:
        T = rng.choice(PERIODS_LOW); payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]; contrib = C / T
        if u_acc + contrib > u3 * 1.1: break
        msgs_raw.append((T, payload, 3)); u_acc += contrib; util_total += contrib
        if len(msgs_raw) > 300: break

    u_acc = 0.0; band_cycle = [0, 2, 4]; band_idx = 0
    while u_acc < u_local:
        T = rng.choice(PERIODS_LOCAL); payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]; contrib = C / T
        if u_acc + contrib > u_local * 1.1: break
        band = band_cycle[band_idx % 3]
        msgs_raw.append((T, payload, band)); u_acc += contrib; util_total += contrib
        band_idx += 1
        if len(msgs_raw) > 500: break

    if not msgs_raw: return [], [], 0.0
    msgs_raw.sort(key=lambda x: x[0])
    msgset = [Msg(f"m{i}", float(T), float(T)*DEADLINE_MULT, i+1, py, band)
              for i, (T, py, band) in enumerate(msgs_raw)]
    fwd_msgs = [m for m in msgset if m.band in (1, 3)]
    return msgset, fwd_msgs, util_total

def build_dst_msgset(fwd_msgs, target_util_dst, can_cfg, rng):
    C_fwd = compute_tx_times([m.payload_size for m in fwd_msgs], can_cfg)
    u_fwd = sum(c / m.period for c, m in zip(C_fwd, fwd_msgs))
    u_rem = target_util_dst - u_fwd
    dst_local = []; util = 0.0; idx = len(fwd_msgs)
    while util < u_rem:
        T = rng.choice(PERIODS_LOCAL); payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]; contrib = C / T
        if util + contrib > u_rem * 1.1: break
        dst_local.append(Msg(f"d{idx}", float(T), float(T)*DEADLINE_MULT,
                             idx+1, payload, -1))
        util += contrib; idx += 1
        if idx > 600: break
    all_dst = list(fwd_msgs) + dst_local
    all_dst.sort(key=lambda m: m.period)
    for i, m in enumerate(all_dst): m.priority = i + 1
    return all_dst

def evaluate_schedulability(all_src_msgs, fwd_msgs, dst_msgset,
                             can_cfg, tsn_cfg, gcl, n, gateway):
    if not fwd_msgs: return False
    payloads_src = [m.payload_size for m in all_src_msgs]
    R_src_all = compute_response_times(
        [m.priority for m in all_src_msgs], [m.period for m in all_src_msgs],
        payloads_src, can_cfg)
    src_id_to_R = {m.msg_id: R_src_all[i] for i, m in enumerate(all_src_msgs)}
    src_id_to_C = {m.msg_id: compute_tx_times([m.payload_size], can_cfg)[0]
                   for m in all_src_msgs}
    R_src = [src_id_to_R[m.msg_id] for m in fwd_msgs]
    C_src = [src_id_to_C[m.msg_id] for m in fwd_msgs]

    payloads_dst = [m.payload_size for m in dst_msgset]
    R_dst_all = compute_response_times(
        [m.priority for m in dst_msgset], [m.period for m in dst_msgset],
        payloads_dst, can_cfg)
    dst_id_to_R = {m.msg_id: R_dst_all[i] for i, m in enumerate(dst_msgset)}
    R_dst = [dst_id_to_R[m.msg_id] for m in fwd_msgs]

    if any(R_dst_all[i] > dst_msgset[i].deadline for i in range(len(dst_msgset))):
        return False

    wc_payload = 17 + sum(sorted([m.payload_size for m in fwd_msgs],
                                  reverse=True)[:n])
    tsn_wcrt = wcrt_ms(Flow(0, min(m.period for m in fwd_msgs), wc_payload, 0),
                       [], gcl, tsn_cfg)
    delta_dec = decap_delay(n, max(C_src))
    S = compute_slacks(fwd_msgs, R_src, C_src, tsn_wcrt, delta_dec, R_dst)

    if gateway == "BF":
        gw = FIFOBatchGateway(fwd_msgs, R_src, C_src, n, tsn_wcrt, max(C_src))
        delta_enc = gw.delta_enc
    elif gateway == "ZS":
        gw = FIFOZeroSlackGateway(fwd_msgs, R_src, C_src, S, n, tsn_wcrt, max(C_src))
        delta_enc = gw.delta_enc
    elif gateway == "ZS-AP":
        gw = FIFOZeroSlackAPGateway(fwd_msgs, R_src, C_src, S, n, tsn_wcrt, max(C_src))
        delta_enc = gw.delta_enc_upper
    elif gateway == "TO":
        T_gw = compute_T_tsn(fwd_msgs, n)
        if T_gw <= 0: T_gw = max(tsn_wcrt, 1.0)
        gw = FIFOTimeoutGateway(fwd_msgs, R_src, C_src, T_gw, n, tsn_wcrt, max(C_src))
        delta_enc = gw.delta_enc
    else:
        raise ValueError(gateway)

    enc_map = {i: delta_enc for i in range(len(fwd_msgs))}
    rows = compute_e2e(fwd_msgs, R_src, C_src, R_dst, enc_map, tsn_wcrt, delta_dec)
    return all(r["feasible"] for r in rows)

def sweep(cycle_us, n, fwd_fraction, policies=POLICIES, seed=SEED):
    rng     = random.Random(seed)
    can_cfg = CANBusConfig(bus_type="CAN-FD", tbit=TBIT)
    tsn_cfg = TSNConfig(link_speed_mbps=TSN_LINK_MBPS, num_switches=TSN_SWITCHES,
                        switch_processing_us=TSN_PROC_US, propagation_delay_us=TSN_PROP_US)
    gcl     = GCL.sample_uniform(cycle_us=cycle_us, window_us=int(cycle_us*WINDOW_FRAC))
    results = {p: [] for p in policies}

    for util in UTIL_LEVELS:
        if util >= 1.0:
            msgset, fwd_msgs, actual_util = generate_banded_msgset(
                util, can_cfg, rng, fwd_fraction)
            if fwd_msgs:
                dst_msgset = build_dst_msgset(fwd_msgs, actual_util, can_cfg, rng)
                try:
                    verified = evaluate_schedulability(
                        msgset, fwd_msgs, dst_msgset, can_cfg, tsn_cfg, gcl, n, "ZS")
                    print(f"U=1.0  gate check (ZS): "
                          f"{'PASS — gate FAILED' if verified else 'correctly returns False'}"
                          f"  → hardcoding 0")
                except RuntimeError:
                    print("U=1.0  gate check (ZS): RTA diverged  → hardcoding 0")
            for p in policies: results[p].append(0.0)
            continue

        counts = {p: 0 for p in policies}; total = 0
        for _ in range(SETS_PER_UTIL):
            msgset, fwd_msgs, actual_util = generate_banded_msgset(
                util, can_cfg, rng, fwd_fraction)
            if len(fwd_msgs) < MIN_BATCH_MULT * n: continue
            dst_msgset = build_dst_msgset(fwd_msgs, actual_util, can_cfg, rng)
            total += 1
            for p in policies:
                try:
                    ok = evaluate_schedulability(msgset, fwd_msgs, dst_msgset,
                                                  can_cfg, tsn_cfg, gcl, n, p)
                    if ok: counts[p] += 1
                except Exception:
                    pass
        for p in policies:
            results[p].append(counts[p] / total if total > 0 else 0.0)
        print(f"U={util:.1f}  " +
              "  ".join(f"{p}={counts[p]/max(total,1):.2f}" for p in policies))
    return results

# ── Plotting ─────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.size": 22,
    "axes.labelsize": 22,
    "axes.titlesize": 21,
    "xtick.labelsize": 22,
    "ytick.labelsize": 22,
    "legend.fontsize": 22,
})

# Policy styles are kept consistent with the line identities used in the paper.
STYLES = {
    "TO": {
        "color": "#95a5a6",
        "linestyle": "-",
        "linewidth": 4.0,
        "marker": "s",
        "zorder": 4,
    },
    "BF": {
        "color": "#2ecc71",
        "linestyle": "-.",
        "linewidth": 3.5,
        "marker": "o",
        "zorder": 5,
    },
    "ZS": {
        "color": "#ff890b",
        "linestyle": ":",
        "linewidth": 4.0,
        "marker": "^",
        "zorder": 8,
    },
    "ZS-AP": {
        "color": "#3498db",
        "linestyle": "-",
        "linewidth": 4.5,
        "marker": "P",
        "zorder": 6,
    },
}

LABELS = {
    "TO": "FIFO-TO",
    "BF": "FIFO-BF",
    "ZS": "FIFO-ZS",
    "ZS-AP": "FIFO-ZS-AP",
}


def plot_ax(ax, results, title, legend_loc="upper left", legend_anchor=(0.8, 0.8)):
    for p, vals in results.items():
        s = STYLES[p]
        line, = ax.plot(UTIL_LEVELS, vals, label=LABELS[p], color=s["color"],
                        linestyle=s["linestyle"], marker=s["marker"],
                        linewidth=3, markersize=5)
        line.set_path_effects([
            pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
    ax.set_xlim(0.08, 0.9); ax.margins(x=0.03); ax.set_ylim(0.0, 1.05)
    ax.set_xticks(UTIL_LEVELS)
    ax.set_xlabel(r"$U$"); ax.set_ylabel("Schedulability ratio")
    ax.set_title(title); ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc=legend_loc, bbox_to_anchor=legend_anchor)


  
# FIGURE (e1) — 30% forwarded: n=5 vs n=15
  
"""
print("\n=== Fig (e1): 30% forwarded, n=5 vs n=15 ===")
res_n5_50  = sweep(cycle_us=500, n=10,  fwd_fraction=0.5)
res_n15_50 = sweep(cycle_us=500, n=15, fwd_fraction=0.3)

fig_e1, ax_e1 = plt.subplots(figsize=(13, 6))
for p in POLICIES:
    s = STYLES[p]
    line5, = ax_e1.plot(UTIL_LEVELS, res_n5_50[p], color=s["color"],
                        linestyle=s["linestyle"], marker=s["marker"],
                        linewidth=2.2, markersize=5, label=f"{LABELS[p]} (n=5)")
    line5.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
    line15, = ax_e1.plot(UTIL_LEVELS, res_n15_50[p], color=s["color"],
                         linestyle=":", linewidth=4, marker="",
                         label=f"{LABELS[p]} (n=15)")
    line15.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
ax_e1.set_xlim(0.08, 0.9); ax_e1.set_ylim(0.0, 1.05)
ax_e1.set_xticks(UTIL_LEVELS)
ax_e1.set_xlabel("Utilization"); ax_e1.set_ylabel("Schedulability (%)")
ax_e1.set_title(""); ax_e1.grid(True, linestyle="--", alpha=0.4)
ax_e1.legend(loc="upper left", bbox_to_anchor=(0.37, 0.93))
#fig_e1.tight_layout(); plt.show()




print("\n=== Fig (e2): 100% forwarded, n=5 vs n=15 ===")
res_n5_100  = sweep(cycle_us=500, n=5,  fwd_fraction=5.0)
res_n15_100 = sweep(cycle_us=500, n=15, fwd_fraction=1.0)

fig_e2, ax_e2 = plt.subplots(figsize=(13, 6))
for p in POLICIES:
    s = STYLES[p]
    line5, = ax_e2.plot(UTIL_LEVELS, res_n5_100[p], color=s["color"],
                        linestyle=s["linestyle"], marker=s["marker"],
                        linewidth=2.2, markersize=5, label=f"{LABELS[p]} (n=5)")
    line5.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
    line15, = ax_e2.plot(UTIL_LEVELS, res_n15_100[p], color=s["color"],
                         linestyle=":", linewidth=4, marker="",
                         label=f"{LABELS[p]} (n=15)")
    line15.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
ax_e2.set_xlim(0.08, 0.9); ax_e2.set_ylim(0.0, 1.05)
ax_e2.set_xticks(UTIL_LEVELS)
ax_e2.set_xlabel("Utilization"); ax_e2.set_ylabel("Schedulability (%)")
ax_e2.set_title(""); ax_e2.grid(True, linestyle="--", alpha=0.4)
ax_e2.legend(loc="upper left", bbox_to_anchor=(0.15, 0.93))
#fig_e2.tight_layout(); plt.show()


"""

  
# FIGURE 6 — Baseline: vary U_src, n=5, fwd=30%
  

print("\n=== Figure 6  n=5  fwd=30% ===")
res_fig6 = sweep(cycle_us=500, n=10,fwd_fraction=0.3)

fig6, ax6 = plt.subplots(figsize=(13, 6))
for p in POLICIES:
    s = STYLES[p]
    line, = ax6.plot(UTIL_LEVELS, res_fig6[p], label=LABELS[p], color=s["color"],
                     linestyle=s["linestyle"], marker=s["marker"],
                     linewidth=4, markersize=5)
    line.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
ax6.set_xlim(0.08, 0.9); ax6.set_ylim(0.0, 1.05)
ax6.set_xticks(UTIL_LEVELS)
ax6.set_xlabel("Utilization"); ax6.set_ylabel("Schedulability ratio")
ax6.set_title(""); ax6.grid(True, linestyle="--", alpha=0.4)
ax6.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 0.93),
    ncol=3,
    frameon=False,
    columnspacing=0.6,
    handlelength=1.2,
    prop={"weight": "medium"}
)

fig6.tight_layout()
plt.show()
  
# FIGURE 7a/7b — Vary forwarding fraction, n=5
# 7a: U=0.5    7b: U=0.7   x-axis: 10%→100%
  
"""
FWD_LEVELS = [round(f * 0.1, 1) for f in range(1, 11)]
fwd_pct    = [int(f * 100) for f in FWD_LEVELS]

can_cfg_7 = CANBusConfig(bus_type="CAN", tbit=TBIT)
tsn_cfg_7 = TSNConfig(link_speed_mbps=TSN_LINK_MBPS, num_switches=TSN_SWITCHES,
                      switch_processing_us=TSN_PROC_US, propagation_delay_us=TSN_PROP_US)
gcl_7 = GCL.sample_uniform(cycle_us=500, window_us=int(500 * WINDOW_FRAC))

for util, fig_label in [(0.5, "7a"), (0.7, "7b")]:
    print(f"\n=== Figure {fig_label}  U={util}  n=10===")
    res_7 = {p: [] for p in POLICIES}
    rng_7 = random.Random(SEED)
    for fwd in FWD_LEVELS:
        counts = {p: 0 for p in POLICIES}; total = 0
        for _ in range(SETS_PER_UTIL):
            msgset, fwd_msgs, actual_util = generate_banded_msgset(
                util, can_cfg_7, rng_7, fwd)
            if len(fwd_msgs) < MIN_BATCH_MULT * 10: continue
            dst_msgset = build_dst_msgset(fwd_msgs, actual_util, can_cfg_7, rng_7)
            total += 1
            for p in POLICIES:
                try:
                    ok = evaluate_schedulability(msgset, fwd_msgs, dst_msgset,
                                                  can_cfg_7, tsn_cfg_7, gcl_7, 10, p)
                    if ok: counts[p] += 1
                except Exception: pass
        for p in POLICIES:
            res_7[p].append(counts[p] / total if total > 0 else 0.0)
        print(f"  fwd={int(fwd*100):>3}%  sets={total}  " +
              "  ".join(f"{p}={counts[p]/max(total,1):.2f}" for p in POLICIES))

    fig7, ax7 = plt.subplots(figsize=(13, 6))
    for p in POLICIES:
        s = STYLES[p]
        line, = ax7.plot(fwd_pct, res_7[p], label=LABELS[p], color=s["color"],
                         linestyle=s["linestyle"], marker=s["marker"],
                         linewidth=2.2, markersize=5)
        line.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
    ax7.set_xlim(8, 102); ax7.set_ylim(0.0, 1.05)
    ax7.set_xticks(fwd_pct)
    ax7.set_xlabel("Forwarded traffic (%)"); ax7.set_ylabel("Schedulability (%)")
    ax7.set_title(""); ax7.grid(True, linestyle="--", alpha=0.4)
    ax7.legend(loc="upper left", bbox_to_anchor=(0.37, 0.93))  # Fig 7 — adjust here
    fig7.tight_layout(); plt.show()

"""

  
# FIGURE 8a/8b — Vary batch size n, fwd=30%
# 8a: U=0.5    8b: U=0.7   x-axis: n ∈ {1,5,15,20,25}
  
"""
N_LEVELS = [1, 5, 15, 20, 25]

can_cfg_8 = CANBusConfig(bus_type="CAN", tbit=TBIT)
tsn_cfg_8 = TSNConfig(link_speed_mbps=TSN_LINK_MBPS, num_switches=TSN_SWITCHES,
                      switch_processing_us=TSN_PROC_US, propagation_delay_us=TSN_PROP_US)
gcl_8 = GCL.sample_uniform(cycle_us=500, window_us=int(500 * WINDOW_FRAC))

for util, fig_label in [(0.5, "8a"), (0.7, "8b")]:
    print(f"\n=== Figure {fig_label}  U={util}  fwd=30% ===")
    res_8 = {p: [] for p in POLICIES}
    rng_8 = random.Random(SEED)
    for n in N_LEVELS:
        counts = {p: 0 for p in POLICIES}; total = 0
        for _ in range(SETS_PER_UTIL):
            msgset, fwd_msgs, actual_util = generate_banded_msgset(
                util, can_cfg_8, rng_8, 0.5)
            if len(fwd_msgs) < MIN_BATCH_MULT * n: continue
            dst_msgset = build_dst_msgset(fwd_msgs, actual_util, can_cfg_8, rng_8)
            total += 1
            for p in POLICIES:
                try:
                    ok = evaluate_schedulability(msgset, fwd_msgs, dst_msgset,
                                                  can_cfg_8, tsn_cfg_8, gcl_8, n, p)
                    if ok: counts[p] += 1
                except Exception: pass
        for p in POLICIES:
            res_8[p].append(counts[p] / total if total > 0 else 0.0)
        print(f"  n={n:>3}  sets={total}  " +
              "  ".join(f"{p}={counts[p]/max(total,1):.2f}" for p in POLICIES))

    fig8, ax8 = plt.subplots(figsize=(13, 6))
    for p in POLICIES:
        s = STYLES[p]
        line, = ax8.plot(N_LEVELS, res_8[p], label=LABELS[p], color=s["color"],
                         linestyle=s["linestyle"], marker=s["marker"],
                         linewidth=2.2, markersize=5)
        line.set_path_effects([pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])
    ax8.set_xlim(-0.5, 26); ax8.set_ylim(0.0, 1.05)
    ax8.set_xticks(N_LEVELS)
    ax8.set_xlabel("Batch size $n$"); ax8.set_ylabel("Schedulability (%)")
    ax8.set_title(""); ax8.grid(True, linestyle="--", alpha=0.4)
    ax8.legend(loc="upper left", bbox_to_anchor=(0.37, 0.93))  # Fig 8 — adjust here
    fig8.tight_layout(); plt.show()
"""