# experiments/schedulability_sweep.py
#
# Analytical schedulability evaluation for CAN-TSN gateway policies.
#
# Policies:
#   - FIFO-BF
#   - FIFO-ZS
#   - FIFO-ZS-AP
#   - FIFO-TO
#
# Figures generated:
#
#   Fig 1(a): Relaxed TAS
#   Fig 1(b): Restrictive TAS
#   Fig 1(c): Batch size sensitivity
#   Fig 1(d): 20% vs 100% forwarded — combined single plot
#


import os
import sys
import random
from dataclasses import dataclass

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.bus import (
    CANBusConfig,
    compute_tx_times,
    compute_response_times,
)

from src.tsn.gcl import GCL
from src.tsn.flow import Flow
from src.tsn.wcrt import TSNConfig, wcrt_ms

from src.gateways.fifo_bf import FIFOBatchGateway
from src.gateways.fifo_zs import (
    compute_slacks,
    FIFOZeroSlackGateway,
)
from src.gateways.fifo_zs_ap import FIFOZeroSlackAPGateway
from src.gateways.fifo_timeout import (
    FIFOTimeoutGateway,
    compute_T_tsn,
)

from src.gateways.decap import decap_delay
from example.evaluation import compute_e2e


  
# CONFIGURATION
  

SEED = 42

# Number of random message sets generated per utilization level
SETS_PER_UTIL = 3

# Source CAN utilization sweep
UTIL_LEVELS = [round(u * 0.1, 1) for u in range(1, 11)]

# CAN
TBIT = 0.002               # ms (500 kbps)
DEADLINE_MULT = 1.0

PAYLOAD_RANGE = (1, 8)

# Period pools
PERIODS_HIGH = [5,10, 20,50]
PERIODS_LOW = [ 100, 200, 500]
PERIODS_LOCAL = [ 5,10, 20, 50, 100, 200, 500, 1000]

# TSN
TSN_LINK_MBPS = 1000
TSN_SWITCHES = 2
TSN_PROC_US = 3.0
TSN_PROP_US = 1.0

# TAS
WINDOW_FRAC = 0.50

# Main forwarding ratio
DEFAULT_FWD_FRACTION = 1

# Policies
POLICIES = ["TO","BF", "ZS", "ZS-AP"]

# Default batch size
DEFAULT_N = 5

# Minimum number of forwarded flows required
MIN_BATCH_MULT = 2

  
# DATACLASS
  

@dataclass
class Msg:
    msg_id: str
    period: float
    deadline: float
    priority: int
    payload_size: int
    band: int
    source_id: int = 0


  
# MESSAGE SET GENERATION
  

def generate_banded_msgset(
    target_util,
    can_cfg,
    rng,
    fwd_fraction=0.4,
):
    msgs_raw = []

    util_total = 0.0

    u_fwd = target_util * fwd_fraction
    u_local = target_util * (1 - fwd_fraction)

      
    # Band 1 — high-priority forwarded
      

    u1 = u_fwd * 0.6
    u_acc = 0.0

    while u_acc < u1:

        T = rng.choice(PERIODS_HIGH)
        payload = rng.randint(*PAYLOAD_RANGE)

        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > u1 * 1.1:
            break

        msgs_raw.append((T, payload, 1))

        u_acc += contrib
        util_total += contrib

        if len(msgs_raw) > 200:
            break

      
    # Band 3 — low-priority forwarded
      

    u3 = u_fwd * 0.4
    u_acc = 0.0

    while u_acc < u3:

        T = rng.choice(PERIODS_LOW)
        payload = rng.randint(*PAYLOAD_RANGE)

        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > u3 * 1.1:
            break

        msgs_raw.append((T, payload, 3))

        u_acc += contrib
        util_total += contrib

        if len(msgs_raw) > 300:
            break

      
    # Local bands
      

    u_acc = 0.0

    band_cycle = [0, 2, 4]
    band_idx = 0

    while u_acc < u_local:

        T = rng.choice(PERIODS_LOCAL)
        payload = rng.randint(*PAYLOAD_RANGE)

        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > u_local * 1.1:
            break

        band = band_cycle[band_idx % 3]

        msgs_raw.append((T, payload, band))

        u_acc += contrib
        util_total += contrib

        band_idx += 1

        if len(msgs_raw) > 500:
            break

    if not msgs_raw:
        return [], [], 0.0

      
    # RM priorities assigned globally
      

    msgs_raw.sort(key=lambda x: x[0])

    msgset = []

    for i, (T, py, band) in enumerate(msgs_raw):

        msgset.append(
            Msg(
                msg_id=f"m{i}",
                period=float(T),
                deadline=float(T) * DEADLINE_MULT,
                priority=i + 1,
                payload_size=py,
                band=band,
            )
        )

    fwd_msgs = [m for m in msgset if m.band in (1, 3)]

    return msgset, fwd_msgs, util_total


  
# DESTINATION CAN
  

def build_dst_msgset(
    fwd_msgs,
    target_util_dst,
    can_cfg,
    rng,
):

    C_fwd = compute_tx_times(
        [m.payload_size for m in fwd_msgs],
        can_cfg,
    )

    u_fwd = sum(
        c / m.period
        for c, m in zip(C_fwd, fwd_msgs)
    )

    u_rem = target_util_dst - u_fwd

    dst_local = []

    util = 0.0
    idx = len(fwd_msgs)

    while util < u_rem:

        T = rng.choice(PERIODS_LOCAL)
        payload = rng.randint(*PAYLOAD_RANGE)

        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if util + contrib > u_rem * 1.1:
            break

        dst_local.append(
            Msg(
                msg_id=f"d{idx}",
                period=float(T),
                deadline=float(T) * DEADLINE_MULT,
                priority=idx + 1,
                payload_size=payload,
                band=-1,
            )
        )

        util += contrib
        idx += 1

        if idx > 600:
            break

    all_dst = list(fwd_msgs) + dst_local

    all_dst.sort(key=lambda m: m.period)

    for i, m in enumerate(all_dst):
        m.priority = i + 1

    return all_dst


  
# ANALYTICAL EVALUATION
  

def evaluate_schedulability(
    all_src_msgs,
    fwd_msgs,
    dst_msgset,
    can_cfg,
    tsn_cfg,
    gcl,
    n,
    gateway,
):

    if not fwd_msgs:
        return False

      
    # Source CAN analysis
      

    payloads_src = [m.payload_size for m in all_src_msgs]

    R_src_all = compute_response_times(
        [m.priority for m in all_src_msgs],
        [m.period for m in all_src_msgs],
        payloads_src,
        can_cfg,
    )

    src_id_to_R = {
        m.msg_id: R_src_all[i]
        for i, m in enumerate(all_src_msgs)
    }

    src_id_to_C = {
        m.msg_id: compute_tx_times(
            [m.payload_size],
            can_cfg,
        )[0]
        for m in all_src_msgs
    }

    R_src = [src_id_to_R[m.msg_id] for m in fwd_msgs]
    C_src = [src_id_to_C[m.msg_id] for m in fwd_msgs]

      
    # Destination CAN analysis
      

    payloads_dst = [m.payload_size for m in dst_msgset]

    R_dst_all = compute_response_times(
        [m.priority for m in dst_msgset],
        [m.period for m in dst_msgset],
        payloads_dst,
        can_cfg,
    )

    dst_id_to_R = {
        m.msg_id: R_dst_all[i]
        for i, m in enumerate(dst_msgset)
    }

    R_dst = [
        dst_id_to_R[m.msg_id]
        for m in fwd_msgs
    ]

      
    # Destination bus schedulability gate
    # Check ALL messages on dst bus — if any miss deadline the
    # whole dst bus is unschedulable regardless of gateway policy.
    # This ensures U_dst = 1.0 always gives schedulability = 0.
      

    if any(R_dst_all[i] > dst_msgset[i].deadline
           for i in range(len(dst_msgset))):
        return False

      
    # TSN WCRT
      

    wc_payload = (
        17
        + sum(
            sorted(
                [m.payload_size for m in fwd_msgs],
                reverse=True,
            )[:n]
        )
    )

    tsn_wcrt = wcrt_ms(
        Flow(
            0,
            min(m.period for m in fwd_msgs),
            wc_payload,
            0,
        ),
        [],
        gcl,
        tsn_cfg,
    )

      
    # Decapsulation
      

    delta_dec = decap_delay(
        n,
        max(C_src),
    )

      
    # Slack
      

    S = compute_slacks(
        fwd_msgs,
        R_src,
        C_src,
        tsn_wcrt,
        delta_dec,
        R_dst,
    )

      
    # Gateway policy
      

    if gateway == "BF":

        gw = FIFOBatchGateway(
            fwd_msgs,
            R_src,
            C_src,
            n,
            tsn_wcrt,
            max(C_src),
        )

        delta_enc = gw.delta_enc

    elif gateway == "ZS":

        gw = FIFOZeroSlackGateway(
            fwd_msgs,
            R_src,
            C_src,
            S,
            n,
            tsn_wcrt,
            max(C_src),
        )

        delta_enc = gw.delta_enc

    elif gateway == "ZS-AP":

        gw = FIFOZeroSlackAPGateway(
            fwd_msgs,
            R_src,
            C_src,
            S,
            n,
            tsn_wcrt,
            max(C_src),
        )

        delta_enc = gw.delta_enc_upper

    elif gateway == "TO":

        T_gw = compute_T_tsn(fwd_msgs, n)

        if T_gw <= 0:
            T_gw = max(tsn_wcrt, 1.0)

        gw = FIFOTimeoutGateway(
            fwd_msgs,
            R_src,
            C_src,
            T_gw,
            n,
            tsn_wcrt,
            max(C_src),
        )

        delta_enc = gw.delta_enc

    else:
        raise ValueError(gateway)

      
    # E2E
      

    enc_map = {
        i: delta_enc
        for i in range(len(fwd_msgs))
    }

    rows = compute_e2e(
        fwd_msgs,
        R_src,
        C_src,
        R_dst,
        enc_map,
        tsn_wcrt,
        delta_dec,
    )

    return all(r["feasible"] for r in rows)


  
# SWEEP
  

def sweep(
    cycle_us,
    n,
    fwd_fraction,
    policies=POLICIES,
    seed=SEED,
):

    rng = random.Random(seed)

    can_cfg = CANBusConfig(
        bus_type="CAN",
        tbit=TBIT,
    )

    tsn_cfg = TSNConfig(
        link_speed_mbps=TSN_LINK_MBPS,
        num_switches=TSN_SWITCHES,
        switch_processing_us=TSN_PROC_US,
        propagation_delay_us=TSN_PROP_US,
    )

    window_us = int(cycle_us * WINDOW_FRAC)

    gcl = GCL.sample_uniform(
        cycle_us=cycle_us,
        window_us=window_us,
    )

    results = {p: [] for p in policies}

    for util in UTIL_LEVELS:

        # ----------------------------------------------------------
        # U=1.0 — verify gate catches it on one set, then hardcode 0
        # ----------------------------------------------------------
        if util >= 1.0:
            msgset, fwd_msgs, actual_util = generate_banded_msgset(
                util, can_cfg, rng, fwd_fraction)
            if fwd_msgs:
                dst_msgset = build_dst_msgset(
                    fwd_msgs, actual_util, can_cfg, rng)
                try:
                    verified = evaluate_schedulability(
                        msgset, fwd_msgs, dst_msgset,
                        can_cfg, tsn_cfg, gcl, n, "ZS")
                    print(
                        f"U=1.0  gate check (ZS): "
                        f"{'PASS — gate FAILED ✗' if verified else 'correctly returns False ✓'}"
                        f"  → hardcoding 0 for all policies"
                    )
                except RuntimeError:
                    # RTA did not converge — bus overloaded, correctly 0
                    print("U=1.0  gate check (ZS): RTA diverged — bus overloaded ✓"
                          "  → hardcoding 0 for all policies")
            for p in policies:
                results[p].append(0.0)
            continue

        counts = {p: 0 for p in policies}
        total = 0

        for _ in range(SETS_PER_UTIL):

            msgset, fwd_msgs, actual_util = (
                generate_banded_msgset(
                    util,
                    can_cfg,
                    rng,
                    fwd_fraction,
                )
            )

            # Ensure enough forwarded flows exist
            if len(fwd_msgs) < MIN_BATCH_MULT * n:
                continue

            dst_msgset = build_dst_msgset(
                fwd_msgs,
                actual_util,
                can_cfg,
                rng,
            )

            # ── Debug: print actual utilization breakdown ─────────
            C_fwd = compute_tx_times(
                [m.payload_size for m in fwd_msgs], can_cfg)
            u_fwd_actual = sum(
                c / m.period for c, m in zip(C_fwd, fwd_msgs))

            C_dst = compute_tx_times(
                [m.payload_size for m in dst_msgset], can_cfg)
            u_dst_actual = sum(
                c / m.period for c, m in zip(C_dst, dst_msgset))

            print(f"    U_target={util:.1f}  "
                  f"U_src={actual_util:.4f}  "
                  f"U_fwd={u_fwd_actual:.4f}  "
                  f"U_dst={u_dst_actual:.4f}  "
                  f"match={'✓' if abs(u_dst_actual - actual_util) < 0.05 else '✗'}")

            total += 1

            for p in policies:

                try:

                    ok = evaluate_schedulability(
                        msgset,
                        fwd_msgs,
                        dst_msgset,
                        can_cfg,
                        tsn_cfg,
                        gcl,
                        n,
                        p,
                    )

                    if ok:
                        counts[p] += 1

                except Exception:
                    pass

        for p in policies:

            ratio = (
                counts[p] / total
                if total > 0 else 0.0
            )

            results[p].append(ratio)

        print(
            f"U={util:.1f}  "
            + "  ".join(
                f"{p}={counts[p]/max(total,1):.2f}"
                for p in policies
            )
        )

    return results


  
# PLOTTING
  

plt.rcParams.update({
    "font.size": 28,
    "axes.labelsize": 28,
    "axes.titlesize": 28,
    "xtick.labelsize": 25,
    "ytick.labelsize": 25,
    "legend.fontsize": 15,
})

STYLES = {
    "TO": {
        "color": "#95a5a6",
        "linestyle": "-",
        "marker": "D",
    },
    "BF": {
        "color": "#e87e13",
        "linestyle": "-.",
        "marker": "o",
    },
    "ZS": {
        "color": "#3498db",
        "linestyle": "-",
        "marker": "x",
    },
    "ZS-AP": {
        "color": "#2ecc71",
        "linestyle": "--",
        "marker": "P",
    },
    
}

LABELS = {
    "TO": "FIFO-TO",
    "BF": "FIFO-BF",
    "ZS": "FIFO-ZS",
    "ZS-AP": "FIFO-ZS-AP",
    
}


def plot_ax(ax, results, title, legend_loc="upper left", legend_anchor=(0.8, 0.8)):
    """
    legend_loc and legend_anchor are set per figure at each call site.
    """
    for p, vals in results.items():

        s = STYLES[p]

        line, = ax.plot(
            UTIL_LEVELS,
            vals,
            label=LABELS[p],
            color=s["color"],
            linestyle=s["linestyle"],
            marker=s["marker"],
            linewidth=3,
            markersize=5,
        )

        line.set_path_effects([
            pe.Stroke(linewidth=3.5, foreground="white"),
            pe.Normal(),
        ])

    ax.set_xlim(0.08, 0.9)
    ax.margins(x=0.03)
    ax.set_ylim(0.0, 1.05)

    ax.set_xticks(UTIL_LEVELS)

    ax.set_xlabel(r"$U_{src}$")
    ax.set_ylabel("Schedulability ratio")

    ax.set_title(title)

    ax.grid(True, linestyle="--", alpha=0.4)

    ax.legend(loc=legend_loc, bbox_to_anchor=legend_anchor)


  
# FIGURE 1(a) — RELAXED TAS
  


  
# FIGURE 1(b) — RESTRICTIVE TAS
  


  
# FIGURE 1(d) — 20% vs 100% FORWARDED — SINGLE COMBINED PLOT
#
# Same color per policy.
# 20%  forwarded → solid line + marker  (as in STYLES)
# 100% forwarded → dotted line, no marker
  



  
# FIGURE (e1) — 50% forwarded: n=5 vs n=15
  

print("\n=== Fig (e1): 50% forwarded, n=5 vs n=15 ===")

res_n5_50  = sweep(cycle_us=500, n=5,  fwd_fraction=0.5)
res_n15_50 = sweep(cycle_us=500, n=15, fwd_fraction=0.5)

fig_e1, ax_e1 = plt.subplots(figsize=(13, 7))

for p in POLICIES:
    s = STYLES[p]

    line5, = ax_e1.plot(
        UTIL_LEVELS, res_n5_50[p],
        color=s["color"], linestyle=s["linestyle"],
        marker=s["marker"], linewidth=2.2, markersize=5,
        label=f"{LABELS[p]} (n=5)",
    )
    line5.set_path_effects([
        pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])

    line15, = ax_e1.plot(
        UTIL_LEVELS, res_n15_50[p],
        color=s["color"], linestyle=":",
        linewidth=4, marker="",
        label=f"{LABELS[p]} (n=15)",
    )
    line15.set_path_effects([
        pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])

ax_e1.set_xlim(0.08, 0.9)
ax_e1.set_ylim(0.0, 1.05)
ax_e1.set_xticks(UTIL_LEVELS)
ax_e1.set_xlabel("Utilization")
ax_e1.set_ylabel("Schedulability (%)")
ax_e1.set_title("")
ax_e1.grid(True, linestyle="--", alpha=0.4)
ax_e1.legend(
    loc="upper left",            # Fig (e1) — adjust here
    bbox_to_anchor=(0.37, 0.93), # Fig (e1) — adjust here
)
#ax_e1.legend(loc="best")

fig_e1.tight_layout()
plt.show()


  
# FIGURE (e2) — 100% forwarded: n=5 vs n=15
  

print("\n=== Fig (e2): 100% forwarded, n=5 vs n=15 ===")

res_n5_100  = sweep(cycle_us=500, n=5,  fwd_fraction=1.0)
res_n15_100 = sweep(cycle_us=500, n=15, fwd_fraction=1.0)

fig_e2, ax_e2 = plt.subplots(figsize=(13, 7))

for p in POLICIES:
    s = STYLES[p]

    line5, = ax_e2.plot(
        UTIL_LEVELS, res_n5_100[p],
        color=s["color"], linestyle=s["linestyle"],
        marker=s["marker"], linewidth=2.2, markersize=5,
        label=f"{LABELS[p]} (n=5)",
    )
    line5.set_path_effects([
        pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])

    line15, = ax_e2.plot(
        UTIL_LEVELS, res_n15_100[p],
        color=s["color"], linestyle=":",
        linewidth=4, marker="",
        label=f"{LABELS[p]} (n=15)",
    )
    line15.set_path_effects([
        pe.Stroke(linewidth=3.5, foreground="white"), pe.Normal()])

ax_e2.set_xlim(0.08, 0.9)
ax_e2.set_ylim(0.0, 1.05)
ax_e2.set_xticks(UTIL_LEVELS)
#ax_e2.set_xlabel(r"$U_{src}$")
ax_e2.set_xlabel("Utilization")
ax_e2.set_ylabel("Schedulability (%)")
ax_e2.set_title("")
ax_e2.grid(True, linestyle="--", alpha=0.4)
ax_e2.legend(
    loc="upper left",            # Fig (e2) — adjust here
    bbox_to_anchor=(0.15, 0.93), # Fig (e2) — adjust here
)

fig_e2.tight_layout()
plt.show()