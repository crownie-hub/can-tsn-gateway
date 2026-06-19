# experiments/schedulability_sweep.py
#
# Analytical schedulability evaluation for CAN-TSN gateway policies.
#
# Metric:
#   A message set is schedulable only if ALL forwarded messages satisfy:
#       R_i^E2E <= D_i
#
# E2E:
#   R_i^E2E = R_src + delta_enc + R_TSN + delta_dec + R_dst - C_i
#
# Figures:
#   Fig 6  : U_src sweep, n=10, fwd_fraction=0.3
#   Fig 7a : forwarding fraction sweep, U_src=0.5, n=10
#   Fig 7b : forwarding fraction sweep, U_src=0.7, n=10
#   Fig 8a : batch size sweep, U_src=0.5, fwd_fraction=0.3
#   Fig 8b : batch size sweep, U_src=0.7, fwd_fraction=0.3

import os
import sys
import random
from dataclasses import dataclass, replace

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.bus import CANBusConfig, compute_tx_times, compute_response_times
from src.tsn.gcl import GCL
from src.tsn.flow import Flow
from src.tsn.wcrt import TSNConfig, wcrt_ms

from src.gateways.fifo_bf import FIFOBatchGateway
from src.gateways.fifo_zs import compute_slacks, FIFOZeroSlackGateway
from src.gateways.fifo_zs_ap import FIFOZeroSlackAPGateway
from src.gateways.fifo_timeout import FIFOTimeoutGateway, compute_T_tsn
from src.gateways.decap import decap_delay
from example.evaluation import compute_e2e


  
# CONFIGURATION — CHANGE VALUES HERE
  

SEED = 42
SETS_PER_UTIL = 100

UTIL_LEVELS = [round(u * 0.1, 1) for u in range(1, 6)]  # 0.1 ... 1.0

# CAN
TBIT = 0.002       # ms, 500 kbps
DEADLINE_MULT = 1.0   # D_i = T_i

PAYLOAD_RANGE = (1, 64)

# RM-style period bands
PERIODS_HIGH = [5, 10, 20, 50]
PERIODS_LOW = [100, 200, 500]
PERIODS_LOCAL = [5, 10, 20, 50, 100, 200, 500, 1000]

# TSN
TSN_LINK_MBPS = 100
TSN_SWITCHES = 2
TSN_PROC_US = 3.0
TSN_PROP_US = 1.0

# TAS
WINDOW_FRAC = 0.50

# Policies
POLICIES = ["TO", "BF", "ZS", "ZS-AP"]

# Batch validity guard
MIN_BATCH_MULT = 2
MAX_ATTEMPTS_FACTOR = 20

# Figure configs
CYCLE_US = 500

FIG6_N = 10
FIG6_FWD = 0.3

FIG7_N = 5
FIG7_UTILS = [0.5, 0.7]
FIG7_FWD_FRACS = [round(i * 0.1, 1) for i in range(1, 11)]

FIG8_FWD = 0.5
FIG8_UTILS = [0.5, 0.7]
FIG8_N_VALUES = [1, 5, 10, 15, 20, 25]


  
# DATA STRUCTURE
  

@dataclass
class Msg:
    msg_id: str
    period: float
    deadline: float
    priority: int
    payload_size: int
    band: int
    source_id: int = 0


  
# MESSAGE GENERATION
  

def generate_banded_msgset(target_util, can_cfg, rng, fwd_fraction=0.3):
    """
    Generate one source CAN message set.

    Bands:
      0: high/local
      1: high/forwarded
      2: medium/local
      3: low/forwarded
      4: low/local

    Only Bands 1 and 3 are forwarded.
    """
    msgs_raw = []
    util_total = 0.0

    u_fwd = target_util * fwd_fraction
    u_local = target_util * (1.0 - fwd_fraction)

    # Band 1: high-priority forwarded
    u1 = 0.6 * u_fwd
    u_acc = 0.0

    while u_acc < u1:
        T = rng.choice(PERIODS_HIGH)
        payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > 1.1 * u1:
            break

        msgs_raw.append((T, payload, 1))
        u_acc += contrib
        util_total += contrib

        if len(msgs_raw) > 300:
            break

    # Band 3: low-priority forwarded
    u3 = 0.4 * u_fwd
    u_acc = 0.0

    while u_acc < u3:
        T = rng.choice(PERIODS_LOW)
        payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > 1.1 * u3:
            break

        msgs_raw.append((T, payload, 3))
        u_acc += contrib
        util_total += contrib

        if len(msgs_raw) > 500:
            break

    # Local traffic: Bands 0, 2, 4
    u_acc = 0.0
    band_cycle = [0, 2, 4]
    band_idx = 0

    while u_acc < u_local:
        T = rng.choice(PERIODS_LOCAL)
        payload = rng.randint(*PAYLOAD_RANGE)
        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if u_acc + contrib > 1.1 * u_local:
            break

        band = band_cycle[band_idx % len(band_cycle)]
        msgs_raw.append((T, payload, band))

        u_acc += contrib
        util_total += contrib
        band_idx += 1

        if len(msgs_raw) > 700:
            break

    if not msgs_raw:
        return [], [], 0.0

    # Global RM priority assignment
    msgs_raw.sort(key=lambda x: x[0])

    msgset = []
    for i, (T, payload, band) in enumerate(msgs_raw):
        msgset.append(
            Msg(
                msg_id=f"m{i}",
                period=float(T),
                deadline=float(T) * DEADLINE_MULT,
                priority=i + 1,
                payload_size=payload,
                band=band,
            )
        )

    fwd_msgs = [m for m in msgset if m.band in (1, 3)]

    return msgset, fwd_msgs, util_total


def build_dst_msgset(fwd_msgs, target_util_dst, can_cfg, rng):
    """
    Destination CAN workload:
        U_dst = U_fwd + U_dst_local

    IMPORTANT:
    Forwarded messages are copied so destination priority assignment
    does not overwrite the source-side priority of fwd_msgs.
    """
    dst_fwd = [replace(m) for m in fwd_msgs]

    C_fwd = compute_tx_times([m.payload_size for m in dst_fwd], can_cfg)
    u_fwd = sum(c / m.period for c, m in zip(C_fwd, dst_fwd))

    u_rem = max(0.0, target_util_dst - u_fwd)

    dst_local = []
    util = 0.0
    idx = len(dst_fwd)

    while util < u_rem:
        T = rng.choice(PERIODS_LOCAL)
        payload = rng.randint(*PAYLOAD_RANGE)

        C = compute_tx_times([payload], can_cfg)[0]
        contrib = C / T

        if util + contrib > 1.1 * u_rem:
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

        if idx > 800:
            break

    dst_msgset = dst_fwd + dst_local
    dst_msgset.sort(key=lambda m: m.period)

    for i, m in enumerate(dst_msgset):
        m.priority = i + 1

    return dst_msgset


  
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

    # Source CAN WCRT
    payloads_src = [m.payload_size for m in all_src_msgs]

    R_src_all = compute_response_times(
        [m.priority for m in all_src_msgs],
        [m.period for m in all_src_msgs],
        payloads_src,
        can_cfg,
    )

    src_id_to_R = {m.msg_id: R_src_all[i] for i, m in enumerate(all_src_msgs)}
    src_id_to_C = {
        m.msg_id: compute_tx_times([m.payload_size], can_cfg)[0]
        for m in all_src_msgs
    }

    R_src = [src_id_to_R[m.msg_id] for m in fwd_msgs]
    C_src = [src_id_to_C[m.msg_id] for m in fwd_msgs]

    # Destination CAN WCRT
    payloads_dst = [m.payload_size for m in dst_msgset]

    R_dst_all = compute_response_times(
        [m.priority for m in dst_msgset],
        [m.period for m in dst_msgset],
        payloads_dst,
        can_cfg,
    )

    dst_id_to_R = {m.msg_id: R_dst_all[i] for i, m in enumerate(dst_msgset)}
    R_dst = [dst_id_to_R[m.msg_id] for m in fwd_msgs]

    # Destination bus must itself be schedulable
    if any(R_dst_all[i] > dst_msgset[i].deadline for i in range(len(dst_msgset))):
        return False

    # TSN WCRT for worst-case aggregate payload
    wc_payload = 9 + sum(
        sorted([m.payload_size for m in fwd_msgs], reverse=True)[:n]
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
    delta_dec = decap_delay(n, max(C_src))

    # Slack for ZS / ZS-AP
    S = compute_slacks(
        fwd_msgs,
        R_src,
        C_src,
        tsn_wcrt,
        delta_dec,
        R_dst,
    )

    # Gateway encapsulation delay
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

        # fallback for invalid timeout values
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
        raise ValueError(f"Unknown gateway policy: {gateway}")

    enc_map = {i: delta_enc for i in range(len(fwd_msgs))}

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


  
# COMMON SWEEP UTILITIES
  

def make_analysis_objects(cycle_us):
    can_cfg = CANBusConfig(bus_type="CAN-FD", tbit=TBIT,dtbit=0.00025)

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

    return can_cfg, tsn_cfg, gcl


def evaluate_point(util, cycle_us, n, fwd_fraction, rng, policies=POLICIES):
    """
    Evaluate all policies for one (U_src, fwd_fraction) point.
    """
    can_cfg, tsn_cfg, gcl = make_analysis_objects(cycle_us)

    counts = {p: 0 for p in policies}
    total = 0

    attempts = 0
    max_attempts = SETS_PER_UTIL * MAX_ATTEMPTS_FACTOR

    while total < SETS_PER_UTIL and attempts < max_attempts:
        attempts += 1

        msgset, fwd_msgs, actual_util = generate_banded_msgset(
            util,
            can_cfg,
            rng,
            fwd_fraction,
        )

        if len(fwd_msgs) < MIN_BATCH_MULT * n:
            continue

        dst_msgset = build_dst_msgset(
            fwd_msgs,
            actual_util,
            can_cfg,
            rng,
        )

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

    ratios = {
        p: counts[p] / total if total > 0 else 0.0
        for p in policies
    }

    return ratios, total


def sweep_utilization(cycle_us, n, fwd_fraction, seed=SEED, policies=POLICIES):
    """
    Sweep U_src over UTIL_LEVELS.
    """
    rng = random.Random(seed)

    results = {p: [] for p in policies}

    for util in UTIL_LEVELS:
        ratios, total = evaluate_point(
            util,
            cycle_us,
            n,
            fwd_fraction,
            rng,
            policies,
        )

        for p in policies:
            results[p].append(ratios[p])

        print(
            f"U={util:.1f}  total={total}  "
            + "  ".join(f"{p}={ratios[p]:.2f}" for p in policies)
        )

    return results


# Keep this name for compatibility with your older code
def sweep(cycle_us, n, fwd_fraction, policies=POLICIES, seed=SEED):
    return sweep_utilization(cycle_us, n, fwd_fraction, seed, policies)


  
# PLOTTING
  

plt.rcParams.update({
    "font.size": 28,
    "axes.labelsize": 28,
    "axes.titlesize": 28,
    "xtick.labelsize": 25,
    "ytick.labelsize": 25,
    "legend.fontsize": 22,
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


def style_line(line):
    line.set_path_effects([
        pe.Stroke(linewidth=3.5, foreground="white"),
        pe.Normal(),
    ])


def finish_axis(
    ax,
    xlabel,
    ylabel="Schedulability (%)",
    xlim=None,
    xticks=None,
):
    if xlim is not None:
        ax.set_xlim(*xlim)

    ax.set_ylim(0.0, 1.05)

    if xticks is not None:
        ax.set_xticks(xticks)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    ax.grid(True, linestyle="--", alpha=0.4)

    ax.legend(
        loc="upper left",
        bbox_to_anchor=(0.51, 0.93),
    )


def plot_policy_lines(ax, x_values, results):
    for p in POLICIES:
        s = STYLES[p]

        line, = ax.plot(
            x_values,
            results[p],
            color=s["color"],
            linestyle=s["linestyle"],
            marker=s["marker"],
            linewidth=4,
            markersize=5,
            label=LABELS[p],
        )

        style_line(line)


  
# FIGURE 6 — U_src SWEEP
  
"""
print("\n=== Fig 6: U_src sweep, n=10, fwd=30% ===")

res_fig6 = sweep(
    cycle_us=CYCLE_US,
    n=FIG6_N,
    fwd_fraction=FIG6_FWD,
)

fig6, ax6 = plt.subplots(figsize=(13, 6))

plot_policy_lines(ax6, UTIL_LEVELS, res_fig6)

finish_axis(
    ax6,
    xlabel=r"$U_{src}$",
    xlim=(0.08, 1.02),
    xticks=UTIL_LEVELS,
)

fig6.tight_layout()
plt.show()

"""


  
# FIGURE 7a / 7b — FORWARDING FRACTION SWEEP ("7a", 0.5),


for fig_label, util_fixed in [ ("7b", 0.7)]:

    print(
        f"\n=== Fig {fig_label}: forwarding fraction sweep, "
        f"U_src={util_fixed}, n={FIG7_N} ==="
    )

    rng = random.Random(SEED)

    x_pct = [int(f * 100) for f in FIG7_FWD_FRACS]
    results = {p: [] for p in POLICIES}

    for fwd_fraction in FIG7_FWD_FRACS:

        ratios, total = evaluate_point(
            util=util_fixed,
            cycle_us=CYCLE_US,
            n=FIG7_N,
            fwd_fraction=fwd_fraction,
            rng=rng,
            policies=POLICIES,
        )

        for p in POLICIES:
            results[p].append(ratios[p])

        print(
            f"fwd={int(fwd_fraction * 100)}%  total={total}  "
            + "  ".join(f"{p}={ratios[p]:.2f}" for p in POLICIES)
        )

    fig, ax = plt.subplots(figsize=(13, 6))

    plot_policy_lines(ax, x_pct, results)

    finish_axis(
        ax,
        xlabel="Forwarded traffic (%)",
        xlim=(8, 102),
        xticks=x_pct,
    )

    fig.tight_layout()
    plt.show()


# FIGURE 8a / 8b — BATCH SIZE SWEEP
  

for fig_label, util_fixed in [("8a", 0.5), ("8b", 0.7)]:

    print(
        f"\n=== Fig {fig_label}: batch size sweep, "
        f"U_src={util_fixed}, fwd=30% ==="
    )

    rng = random.Random(SEED)

    results = {p: [] for p in POLICIES}

    for n in FIG8_N_VALUES:

        ratios, total = evaluate_point(
            util=util_fixed,
            cycle_us=CYCLE_US,
            n=n,
            fwd_fraction=FIG8_FWD,
            rng=rng,
            policies=POLICIES,
        )

        for p in POLICIES:
            results[p].append(ratios[p])

        print(
            f"n={n}  total={total}  "
            + "  ".join(f"{p}={ratios[p]:.2f}" for p in POLICIES)
        )

    fig, ax = plt.subplots(figsize=(15, 7))

    plot_policy_lines(ax, FIG8_N_VALUES, results)

    finish_axis(
        ax,
        xlabel=r"Batch size $(n)$",
        xlim=(0, max(FIG8_N_VALUES) + 1),
        xticks=FIG8_N_VALUES,
    )

    fig.tight_layout()
    plt.show()