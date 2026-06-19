# experiments/combined_theorem_plots.py
#
# Combined theorem validation plots.
#
# Left  : Synthetic worst-case workload, per-message prediction lateness.
# Right : Randomized Theorem 3 validation across simulation runs.
#
# This file is intentionally separate from:
#   - theorem_12_violin.py
#   - theorem3_random_validation.py
#
# It keeps the experiments independent but combines the final plots into
# one side-by-side figure with consistent styling.

import os
import sys
import random
from dataclasses import dataclass, replace
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import matplotlib.ticker as ticker
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.bus import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge import build_instances_from_sim
from src.tsn.gcl import GCL
from src.tsn.flow import Flow
from src.tsn.wcrt import TSNConfig, wcrt_ms
from src.gateways.fifo_zs import compute_slacks
from src.gateways.fifo_zs_ap_sim import simulate_fifo_zs_ap
from src.gateways.decap import decap_delay



plt.rcParams.update({
    "font.size": 25,
    "axes.titlesize": 25,
    "axes.labelsize": 25,
    "xtick.labelsize": 23,
    "ytick.labelsize": 23,
    "legend.fontsize": 22,
})

BOUND_STYLE = {
    "color": "#e74c3c",
    "linestyle": "--",
    "linewidth": 3.0,
    "zorder": 3,
    "label": "Theoretical bound",
}

OBS_STYLE = {
    "color": "#3498db",
    "linestyle": "-",
    "linewidth": 3.0,
    "zorder": 4,
    "label": "Max observed prediction lateness",
}


def apply_line_outline(line, extra=1.0):
    line.set_path_effects([
        pe.Stroke(
            linewidth=line.get_linewidth() + extra,
            foreground="white",
        ),
        pe.Normal(),
    ])


def finish_prediction_axis(ax, xlabel):
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Prediction lateness (ms)")
    ax.yaxis.set_major_locator(
    ticker.MaxNLocator(integer=False))
    ax.grid(axis="y", linestyle="--", alpha=0.4)




PERIODS_SYN = [5, 10, 20, 40, 80, 160]
PAYLOAD_SYN = 8
BATCH_SIZE_SYN = 4
N_RUNS_SYN = 200
SIM_HORIZON_SYN = 500.0


@dataclass
class Msg:
    msg_id: str
    period: float
    deadline: float
    priority: int
    payload_size: int
    source_id: int = 0


def run_synthetic_prediction_experiment():
    """
    Synthetic worst-case workload from theorem_12_violin.py.

    Returns:
        pred_labels : list[str]
        max_errors  : list[float]
        per_bounds  : list[float]
    """

    msgset = [
        Msg(
            msg_id=f"{i + 1:X}F",
            period=float(T),
            deadline=float(T),
            priority=i + 1,
            payload_size=PAYLOAD_SYN,
        )
        for i, T in enumerate(sorted(PERIODS_SYN, reverse=True))
    ]

    can_cfg = CANBusConfig(bus_type="CAN", tbit=0.002)

    C_src = compute_tx_times(
        [m.payload_size for m in msgset],
        can_cfg,
    )

    R_src = compute_response_times(
        [m.priority for m in msgset],
        [m.period for m in msgset],
        [m.payload_size for m in msgset],
        can_cfg,
    )

    R_dst = list(R_src)

    tsn_cfg = TSNConfig(
        link_speed_mbps=1000,
        num_switches=1,
        switch_processing_us=3.0,
        propagation_delay_us=1.0,
    )

    gcl = GCL.sample_uniform(
        cycle_us=1000,
        window_us=200,
    )

    wc_payload = 9 + BATCH_SIZE_SYN * PAYLOAD_SYN

    tsn_wcrt = wcrt_ms(
        Flow(
            0,
            min(m.period for m in msgset),
            wc_payload,
            0,
        ),
        [],
        gcl,
        tsn_cfg,
    )

    delta_dec = decap_delay(
        BATCH_SIZE_SYN,
        max(C_src),
    )

    S = compute_slacks(
        msgset,
        R_src,
        C_src,
        tsn_wcrt,
        delta_dec,
        R_dst,
    )

    nodes = [
        NodeConfig(
            0,
            "ECU0",
            [
                MessageConfig(
                    m.priority,
                    m.period,
                    m.payload_size,
                    m.deadline,
                    name=m.msg_id,
                )
                for m in msgset
            ],
        )
    ]

    pred_log_all = []

    print(f"\nRunning synthetic prediction experiment: {N_RUNS_SYN} runs")

    for run in range(N_RUNS_SYN):
        sim = CANBusSimulator(
            nodes,
            can_cfg,
            SIM_HORIZON_SYN,
        )

        done = sim.run()

        instances = build_instances_from_sim(
            done,
            use_arrival="finish",
        )

        if not instances:
            continue

        _, pred_log = simulate_fifo_zs_ap(
            instances,
            S,
            R_src,
            C_src,
            BATCH_SIZE_SYN,
            return_prediction_log=True,
        )

        pred_log_all.extend(pred_log)

        if (run + 1) % 50 == 0:
            print(f"  synthetic: {run + 1}/{N_RUNS_SYN}")

    sort_order = sorted(
        range(len(msgset)),
        key=lambda i: msgset[i].priority,
    )

    by_flow = defaultdict(list)

    for p in pred_log_all:
        by_flow[p["flow_id"]].append(p)

    pred_labels = []
    max_errors = []
    per_bounds = []

    for i in sort_order:
        preds = by_flow.get(i, [])

        if not preds:
            continue

        pred_labels.append(msgset[i].msg_id)
        max_errors.append(max(p["error"] for p in preds))
        per_bounds.append(2.0 * (R_src[i] - C_src[i]))

    return pred_labels, max_errors, per_bounds


  

# Based on theorem3_random_validation.py.
  

N_RUNS_RAND = 10000
N_MSGS_RAND = 10
BATCH_SIZE_RAND = 5
SIM_HORIZON_RAND = 200.0
SEED_RAND = 42
PERIOD_CHOICES_RAND = [5, 10, 15, 20, 30, 50, 100]
PAYLOAD_RANGE_RAND = (1, 8)


@dataclass
class RandMsg:
    msg_id: str
    period: float
    deadline: float
    priority: int
    payload_size: int
    source_id: int = 0


def random_workload(n):
    periods = sorted(
        random.choices(
            PERIOD_CHOICES_RAND,
            k=n,
        )
    )

    payloads = [
        random.randint(*PAYLOAD_RANGE_RAND)
        for _ in range(n)
    ]

    priorities = random.sample(
        range(1, 512),
        n,
    )

    return [
        RandMsg(
            msg_id=f"m{i}",
            period=float(T),
            deadline=float(T),
            priority=p,
            payload_size=py,
            source_id=0,
        )
        for i, (T, p, py) in enumerate(zip(periods, priorities, payloads))
    ]


def run_random_theorem3_experiment():
    """
    Randomized Theorem 3 validation from theorem3_random_validation.py.

    Returns:
        sorted_err  : np.ndarray
        sorted_bnd  : np.ndarray
        sorted_viol : np.ndarray[bool]
    """

    random.seed(SEED_RAND)
    np.random.seed(SEED_RAND)

    can_cfg = CANBusConfig(
        bus_type="CAN",
        tbit=0.002,
    )

    tsn_cfg = TSNConfig(
        link_speed_mbps=1000,
        num_switches=1,
        switch_processing_us=3.0,
        propagation_delay_us=1.0,
    )

    gcl = GCL.sample_uniform(
        cycle_us=1000,
        window_us=200,
    )

    run_max_errors = []
    run_bounds = []
    violations = []

    print(f"\nRunning randomized Theorem 3 experiment: {N_RUNS_RAND} trials")

    for run in range(N_RUNS_RAND):
        msgset = random_workload(N_MSGS_RAND)

        payloads = [m.payload_size for m in msgset]
        periods = [m.period for m in msgset]
        priorities = [m.priority for m in msgset]

        C_src = compute_tx_times(
            payloads,
            can_cfg,
        )

        R_src = compute_response_times(
            priorities,
            periods,
            payloads,
            can_cfg,
        )

        R_dst = list(R_src)

        per_bound = [
            2.0 * (R_src[i] - C_src[i])
            for i in range(len(msgset))
        ]

        max_bound = max(per_bound)

        wc_payload = (
            17
            + sum(
                sorted(
                    payloads,
                    reverse=True,
                )[:BATCH_SIZE_RAND]
            )
        )

        tsn_wcrt = wcrt_ms(
            Flow(
                0,
                min(periods),
                wc_payload,
                0,
            ),
            [],
            gcl,
            tsn_cfg,
        )

        delta_dec = decap_delay(
            BATCH_SIZE_RAND,
            max(C_src),
        )

        S = compute_slacks(
            msgset,
            R_src,
            C_src,
            tsn_wcrt,
            delta_dec,
            R_dst,
        )

        msgs_cfg = [
            MessageConfig(
                msg_id=m.priority,
                period=m.period,
                payload_bytes=m.payload_size,
                deadline=m.deadline,
                name=m.msg_id,
            )
            for m in msgset
        ]

        nodes = [
            NodeConfig(
                0,
                "ECU0",
                msgs_cfg,
            )
        ]

        sim = CANBusSimulator(
            nodes,
            can_cfg,
            SIM_HORIZON_RAND,
        )

        done = sim.run()

        instances = build_instances_from_sim(
            done,
            use_arrival="finish",
        )

        analytical_instances = []

        for inst in instances:
            fid = int(inst.flow_id)

            analytical_arrival = (
                float(inst.release)
                + float(R_src[fid])
            )

            analytical_instances.append(
                replace(
                    inst,
                    arrive_gw=analytical_arrival,
                )
            )

        instances = analytical_instances

        _, pred_log = simulate_fifo_zs_ap(
            instances,
            S,
            R_src,
            C_src,
            BATCH_SIZE_RAND,
            return_prediction_log=True,
        )

        if not pred_log:
            continue

        max_err = max(
            p["error"]
            for p in pred_log
        )

        violated = any(
            not p["ok"]
            for p in pred_log
        )

        run_max_errors.append(max_err)
        run_bounds.append(max_bound)
        violations.append(violated)

        if (run + 1) % 1000 == 0:
            print(f"  randomized: {run + 1}/{N_RUNS_RAND}")

    n_valid = len(run_max_errors)
    n_viol = sum(violations)

    print(f"\nRandomized Theorem 3 Summary")
    print(f"  Total runs : {N_RUNS_RAND}")
    print(f"  Valid runs : {n_valid}")
    print(f"  Violations : {n_viol}")

    order_err = np.argsort(run_max_errors)

    sorted_err = np.array(run_max_errors)[order_err]
    sorted_viol = np.array(violations)[order_err]

    # Keep the same presentation choice as the original script:
    # independently sort theorem bounds.
    sorted_bnd = np.sort(np.array(run_bounds))

    return sorted_err, sorted_bnd, sorted_viol


  
# COMBINED FIGURE
  

if __name__ == "__main__":

    sorted_err, sorted_bnd, sorted_viol = run_random_theorem3_experiment()
    pred_labels, max_errors, per_bounds = run_synthetic_prediction_experiment()

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(17, 6),
        sharey=False,
    )

    # (a) Randomized Theorem 3 validation
   

    ax = axes[0]

    x_err = np.arange(len(sorted_err))
    x_bnd = np.arange(len(sorted_bnd))

    line_bound, = ax.plot(
        x_bnd,
        sorted_bnd,
        color=BOUND_STYLE["color"],
        linestyle=BOUND_STYLE["linestyle"],
        linewidth=BOUND_STYLE["linewidth"],
        zorder=BOUND_STYLE["zorder"],
        label=BOUND_STYLE["label"],
    )
    apply_line_outline(line_bound)

    line_obs, = ax.plot(
        x_err,
        sorted_err,
        color=OBS_STYLE["color"],
        linestyle=OBS_STYLE["linestyle"],
        linewidth=OBS_STYLE["linewidth"],
        zorder=OBS_STYLE["zorder"],
        label=OBS_STYLE["label"],
    )
    apply_line_outline(line_obs)

    viol_x = x_err[sorted_viol]
    viol_y = sorted_err[sorted_viol]

    if len(viol_x) > 0:
        ax.scatter(
            viol_x,
            viol_y,
            color="#e74c3c",
            s=60,
            zorder=5,
            label=f"Violations ({len(viol_x)})",
        )

    ax.set_title("(a) Randomized message sets")
    finish_prediction_axis(ax, "Simulation index")


    # (b) Synthetic per-message prediction lateness
   

    ax = axes[1]

    x1 = np.arange(len(pred_labels))

    line_bound, = ax.plot(
        x1,
        per_bounds,
        color=BOUND_STYLE["color"],
        linestyle=BOUND_STYLE["linestyle"],
        linewidth=BOUND_STYLE["linewidth"],
        zorder=BOUND_STYLE["zorder"],
        label=BOUND_STYLE["label"],
    )
    apply_line_outline(line_bound)

    line_obs, = ax.plot(
        x1,
        max_errors,
        color=OBS_STYLE["color"],
        linestyle=OBS_STYLE["linestyle"],
        linewidth=OBS_STYLE["linewidth"],
        zorder=OBS_STYLE["zorder"],
        label=OBS_STYLE["label"],
    )
    apply_line_outline(line_obs)

    ax.set_xticks(x1)
    ax.set_xticklabels(
        pred_labels,
        rotation=35,
        ha="right",
        rotation_mode="anchor",
    )
    ax.tick_params(axis="x", pad=1)

    ax.set_title("(b) Worst-case message")
    finish_prediction_axis(ax, "CAN ID")

    # Remove duplicate y-label on second subplot
    axes[1].set_ylabel("")

    # -------------------------------------------------------------
    # Shared legend and layout
    # -------------------------------------------------------------

    handles, labels = axes[0].get_legend_handles_labels()

    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.98),
        ncol=2,
        frameon=False,
        columnspacing=0.8,
        handlelength=1.4,
        prop={"weight": "medium"},
    )

    fig.subplots_adjust(
        top=0.78,
        bottom=0.21,
        left=0.08,
        right=0.98,
        wspace=0.15,
    )

    plt.show()
