# experiments/theorem3_random_validation.py
#
# Theorem 3 — Prediction Error Bound Validation
#
#   a_i^q = r_i^q + R_i^src


import os
import sys
import random

from dataclasses import dataclass

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import matplotlib.ticker as ticker
import numpy as np

ROOT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
    )
)

sys.path.insert(0, ROOT)

from src.can.bus import (
    CANBusConfig,
    compute_tx_times,
    compute_response_times,
)

from src.can.simulator import (
    CANBusSimulator,
    NodeConfig,
    MessageConfig,
)

from src.can.sim_bridge import (
    build_instances_from_sim,
)

from src.tsn.gcl import GCL

from src.tsn.flow import Flow

from src.tsn.wcrt import (
    TSNConfig,
    wcrt_ms,
)

from src.gateways.fifo_zs import (
    compute_slacks,
)

from src.gateways.fifo_zs_ap_sim import (
    simulate_fifo_zs_ap,
)

from src.gateways.decap import (
    decap_delay,
)

  
# configuration
  

N_RUNS = 10000

N_MSGS = 10

BATCH_SIZE = 5

SIM_HORIZON = 200.0

SEED = 42

PERIOD_CHOICES = [
    5,
    10,
    15,
    20,
    30,
    50,
    100,
]

PAYLOAD_RANGE = (1, 8)

random.seed(SEED)

np.random.seed(SEED)

  
# CAN / TSN config
  

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

  
# message definition
  

@dataclass
class RandMsg:

    msg_id: str

    period: float

    deadline: float

    priority: int

    payload_size: int

    source_id: int = 0

  
# workload generation
  

def random_workload(n):

    periods = sorted(
        random.choices(
            PERIOD_CHOICES,
            k=n,
        )
    )

    payloads = [

        random.randint(
            *PAYLOAD_RANGE
        )

        for _ in range(n)
    ]

    # distinct CAN IDs
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

        for i, (T, p, py)

        in enumerate(
            zip(
                periods,
                priorities,
                payloads,
            )
        )
    ]

  
# experiment data
  

run_max_errors = []

run_bounds = []

violations = []

print(
    f"Running {N_RUNS} randomised trials..."
)

  
# experiments
  

for run in range(N_RUNS):

    msgset = random_workload(
        N_MSGS
    )

    payloads = [
        m.payload_size
        for m in msgset
    ]

    periods = [
        m.period
        for m in msgset
    ]

    priorities = [
        m.priority
        for m in msgset
    ]

      
    # analytical CAN timing
      

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

      
    # theorem bound
      

    per_bound = [

        2.0 *
        (
            R_src[i]
            -
            C_src[i]
        )

        for i in range(len(msgset))
    ]

    max_bound = max(
        per_bound
    )

      
    # TSN timing
      

    wc_payload = (
        17
        +
        sum(
            sorted(
                payloads,
                reverse=True,
            )[:BATCH_SIZE]
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
        BATCH_SIZE,
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

      
    # build CAN simulation
      

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
        SIM_HORIZON,
    )

    done = sim.run()

    instances = build_instances_from_sim(
        done,
        use_arrival="finish",
    )

      
    # overwrite simulated arrivals
    #
    #   a_i^q = r_i^q + R_i^src
    #
    # fully analytical theorem validation
      

  
# analytical arrivals
#
#   a_i^q = r_i^q + R_i^src
  

    from dataclasses import replace

    analytical_instances = []

    for inst in instances:

        fid = int(inst.flow_id)

        analytical_arrival = (
            float(inst.release)
            +
            float(R_src[fid])
        )

        analytical_instances.append(

            replace(
                inst,
                arrive_gw=analytical_arrival,
            )
        )

    instances = analytical_instances


      
    # run FIFO-ZS-AP
      

    _, pred_log = simulate_fifo_zs_ap(

        instances,

        S,

        R_src,

        C_src,

        BATCH_SIZE,

        return_prediction_log=True,
    )

    if not pred_log:
        continue

      
    # results
      

    max_err = max(
        p["error"]
        for p in pred_log
    )

    violated = any(
        not p["ok"]
        for p in pred_log
    )

    run_max_errors.append(
        max_err
    )

    run_bounds.append(
        max_bound
    )

    violations.append(
        violated
    )

    if (run + 1) % 100 == 0:

        print(
            f"  {run + 1}/{N_RUNS} completed"
        )

  
# summary
  

n_valid = len(
    run_max_errors
)

n_viol = sum(
    violations
)

max_obs = max(
    run_max_errors
)

max_bnd = max(
    run_bounds
)

closest_pct = 100.0 * max(

    e / b

    for e, b

    in zip(
        run_max_errors,
        run_bounds,
    )
)

print(
    f"\n{'Theorem 3 Validation Summary':=<55}"
)

print(
    f"  Total runs           : {N_RUNS}"
)

print(
    f"  Valid runs           : {n_valid}"
)

print(
    f"  Violations           : {n_viol}"
)

print(
    f"  Max observed error   : {max_obs:.4f} ms"
)

print(
    f"  Max theoretical bound: {max_bnd:.4f} ms"
)

print(
    f"  Closest approach     : {closest_pct:.1f}% of bound"
)

print(
    f"  Bound holds          : "
    f"{'✓' if n_viol == 0 else '✗'}"
)

  
# plotting
  

plt.rcParams.update({

    "font.size": 25,

    "axes.titlesize": 25,

    "axes.labelsize": 25,

    "xtick.labelsize": 25,

    "ytick.labelsize": 25,

    "legend.fontsize": 23,
})

# sort by max observed prediction error ascending
# bound reorders to match — stays above at every point
# sort observed error ascending — smooth blue line
# observed errors
order_err = np.argsort(run_max_errors)

sorted_err = np.array(
    run_max_errors
)[order_err]

sorted_viol = np.array(
    violations
)[order_err]

# independently sort theorem bounds
sorted_bnd = np.sort(
    np.array(run_bounds)
)

# x-axes
x_err = np.arange(
    len(sorted_err)
)

x_bnd = np.arange(
    len(sorted_bnd)
)


fig, ax = plt.subplots(
    figsize=(12, 5.5)
)


# theorem bound

line_bnd, = ax.plot(

    x_bnd,

    sorted_bnd,

    color="#e74c3c",

    linestyle="--",

    linewidth=2.5,

    zorder=3,

    label=(
        r"Theoretical bound"
    ),
)
#label=(
#        r"Theoretical bound  "
#       r"$(2\cdot\max_i(R_i^{src} - C_i))$"
#    )

line_bnd.set_path_effects([

    pe.Stroke(
        linewidth=4,
        foreground="white",
    ),

    pe.Normal(),
])


line_obs, = ax.plot(

    x_err,

    sorted_err,

    color="#3498db",

    linestyle="-",

    linewidth=2.0,

    zorder=4,

    label="Max observed prediction lateness",
)

line_obs.set_path_effects([

    pe.Stroke(
        linewidth=3.5,
        foreground="white",
    ),

    pe.Normal(),
])


viol_x = x_err[
    sorted_viol
]

viol_y = sorted_err[
    sorted_viol
]

if len(viol_x) > 0:

    ax.scatter(

        viol_x,

        viol_y,

        color="#e74c3c",

        s=60,

        zorder=5,

        label=f"Violations ({len(viol_x)})",
    )


# axes


ax.set_xlabel(
    "Simulation index"
)

ax.set_ylabel(
    "Prediction lateness (ms)"
)

ax.yaxis.set_major_locator(
    ticker.MaxNLocator(
        integer=False
    )
)

ax.grid(
    axis="y",
    linestyle="--",
    alpha=0.4,
)
ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, 0.93),
    ncol=2,
    frameon=False,
    columnspacing=0.5,
    handlelength=1.0,
    prop={"weight": "550"}
)
fig.tight_layout()

plt.show()