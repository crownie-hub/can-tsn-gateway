# example/fifo_zs_sim_example.py
#
# Simulation example for FIFO-ZS and FIFO-ZS-AP.
#
# All times are in milliseconds (ms).

import os
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.message import message_set
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
from src.can.sim_bridge import build_instances_from_sim

from src.tsn.gcl import GCL
from src.tsn.flow import Flow
from src.tsn.wcrt import TSNConfig, wcrt_ms

from src.gateways.fifo_zs import (
    compute_slacks,
    FIFOZeroSlackGateway,
)
from src.gateways.fifo_zs_sim import (
    simulate_fifo_zs,
    print_zs_batch_trace,
)
from src.gateways.fifo_zs_ap import FIFOZeroSlackAPGateway
from src.gateways.fifo_zs_ap_sim import (
    simulate_fifo_zs_ap,
    print_prediction_error_log,
)
from src.gateways.decap import decap_delay

from example.evaluation import (
    compute_max_enc_delay,
    compute_e2e,
    verify_theorem_bound,
)


# ============================================================
# Configuration
# ============================================================

BATCH_SIZE = 5
SIM_HORIZON = 200.0  # ms


# ============================================================
# Helper: convert CANMessage objects into simulator NodeConfig
# ============================================================

def build_nodes(msgset):
    grouped = defaultdict(list)

    for m in msgset:
        grouped[m.source_id].append(
            MessageConfig(
                msg_id=m.priority,
                period=m.period,
                payload_bytes=m.payload_size,
                deadline=m.deadline,
                name=m.msg_id,
            )
        )

    return [
        NodeConfig(
            node_id=src_id,
            node_name=f"ECU{src_id}",
            messages=msgs,
        )
        for src_id, msgs in sorted(grouped.items())
    ]


def print_can_sim_summary(done, msgset):
    print(f"\nCAN simulation: {len(done)} frames in {SIM_HORIZON} ms")

    by_msg = defaultdict(list)
    for f in done:
        by_msg[f.msg_index].append(f)

    print(f"\n{'CAN simulation observed delays':=<65}")
    print(
        f"  {'msg':>5}  {'count':>6}  "
        f"{'max_src_delay':>14}  {'max_wait':>10}"
    )
    print(f"  {'-' * 45}")

    for i, m in enumerate(msgset):
        frames = by_msg.get(i, [])

        if not frames:
            print(
                f"  {m.msg_id:>5}  {0:>6}  "
                f"{0.0:>14.4f}  {0.0:>10.4f}"
            )
            continue

        max_src = max(
            f.response_time for f in frames
            if f.response_time is not None
        )

        max_wait = max(
            f.waiting_time for f in frames
            if f.waiting_time is not None
        )

        print(
            f"  {m.msg_id:>5}  {len(frames):>6}  "
            f"{max_src:>14.4f}  {max_wait:>10.4f}"
        )


def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else 0.0


# ============================================================
# Message set
# ============================================================

msgset = message_set()

priorities = [m.priority for m in msgset]
periods = [m.period for m in msgset]
payloads = [m.payload_size for m in msgset]


# ============================================================
# CAN analytical setup
# ============================================================

can_cfg = CANBusConfig(
    bus_type="CAN",
    tbit=0.002,  # 500 kbps
)

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

# Use the same CAN model at destination for now.
R_dst = list(R_src)


# ============================================================
# TSN analytical setup
# ============================================================

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

wc_payload = 17 + sum(
    sorted(payloads, reverse=True)[:BATCH_SIZE]
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
    msgset=msgset,
    R_src=R_src,
    C=C_src,
    tsn_wcrt=tsn_wcrt,
    delta_dec=delta_dec,
    R_dst=R_dst,
)


gw_zs = FIFOZeroSlackGateway(
    msgset=msgset,
    R=R_src,
    C=C_src,
    S=S,
    n=BATCH_SIZE,
    tsn_wcrt=tsn_wcrt,
    c_can_max=max(C_src),
)

gw_ap = FIFOZeroSlackAPGateway(
    msgset=msgset,
    R=R_src,
    C=C_src,
    S=S,
    n=BATCH_SIZE,
    tsn_wcrt=tsn_wcrt,
    c_can_max=max(C_src),
)

print("=" * 65)
print("Analytical Bounds")
print("=" * 65)

print(f"  FIFO-ZS    delta_enc = {gw_zs.delta_enc:.4f} ms")
print(
    f"  FIFO-ZS-AP delta_enc in "
    f"[{gw_ap.delta_enc_lower:.4f}, {gw_ap.delta_enc_upper:.4f}] ms"
)
print(
    f"  prediction_gain      = {gw_ap.prediction_gain:.4f} ms "
    f"[= 2*max(R-C)]"
)

print(f"\n{'Slack values':=<65}")
print(
    f"  {'msg':>5}  {'R_src':>8}  {'C':>8}  "
    f"{'R_dst':>8}  {'S_i':>10}"
)
print(f"  {'-' * 50}")

for m, r, c, rd, s in zip(msgset, R_src, C_src, R_dst, S):
    print(
        f"  {m.msg_id:>5}  {r:>8.4f}  {c:>8.4f}  "
        f"{rd:>8.4f}  {s:>10.4f}"
    )


# CAN bus simulation

nodes = build_nodes(msgset)

sim = CANBusSimulator(
    nodes,
    can_cfg,
    SIM_HORIZON,
)

done = sim.run()

print_can_sim_summary(done, msgset)

instances = build_instances_from_sim(
    done,
    use_arrival="finish",
)

print(f"\nGateway instances: {len(instances)}")


# FIFO-ZS simulation


batches_zs = simulate_fifo_zs(
    instances=instances,
    S=S,
    batch_size=BATCH_SIZE,
)

enc_zs = compute_max_enc_delay(
    batches_zs,
)


# FIFO-ZS-AP simulation


batches_ap, prediction_log = simulate_fifo_zs_ap(
    instances=instances,
    S=S,
    R_src=R_src,
    C=C_src,
    batch_size=BATCH_SIZE,
    return_prediction_log=True,
)

enc_ap = compute_max_enc_delay(
    batches_ap,
)



# Batch trace


print_zs_batch_trace(
    batches_zs,
    S,
    msgset=msgset,
    n_show=5,
)


result = verify_theorem_bound(
    enc_zs,
    enc_ap,
    R_src,
    C_src,
)

print(f"\n{'Theorem Verification':=<65}")
print(f"  gain = 2*max(R-C) = {result['gain']:.4f} ms")
print(f"  max observed improvement = {result['max_improvement']:.4f} ms")
print(f"  bound holds: {'✓' if result['bound_holds'] else '✗'}")

print(
    f"\n  {'flow':>5}  {'delta_zs':>10}  {'delta_ap':>10}  "
    f"{'improvement':>12}  {'ok':>4}"
)
print(f"  {'-' * 50}")

for r in result["rows"]:
    m = msgset[r["flow_id"]]
    ok = "ok" if r["within_bound"] else "no"

    print(
        f"  {m.msg_id:>5}  "
        f"{r['delta_zs']:>10.4f}  "
        f"{r['delta_ap']:>10.4f}  "
        f"{r['improvement']:>12.4f}  "
        f"{ok:>4}"
    )




print_prediction_error_log(
    prediction_log,
    msgset=msgset,
    n_show=40,
)


# Prediction error summary per flow


print(f"\n{'Prediction Error Summary Per Flow':=<65}")
print(f"  {'msg':>5}  {'max_error':>10}  {'bound':>10}  {'ok':>4}")
print(f"  {'-' * 38}")

pred_by_flow = defaultdict(list)

for row in prediction_log:
    pred_by_flow[row["flow_id"]].append(row)

for i, m in enumerate(msgset):
    rows = pred_by_flow.get(i, [])

    bound = 2.0 * (R_src[i] - C_src[i])

    if not rows:
        print(
            f"  {m.msg_id:>5}  "
            f"{'--':>10}  "
            f"{bound:>10.4f}  "
            f"{'--':>4}"
        )
        continue

    max_error = max(r["error"] for r in rows)
    ok = max_error <= bound + 1e-9

    print(
        f"  {m.msg_id:>5}  "
        f"{max_error:>10.4f}  "
        f"{bound:>10.4f}  "
        f"{'✓' if ok else '✗':>4}"
    )


# Correct simulation check


print(f"\n{'Zero-Slack Check: Sim Enc Delay <= S_i':=<65}")
print(
    f"  {'msg':>5}  {'sim_zs':>8}  {'sim_ap':>8}  "
    f"{'S_i':>8}  {'zs_ok':>6}  {'ap_ok':>6}"
)
print(f"  {'-' * 55}")

for i, m in enumerate(msgset):
    d_zs = enc_zs.get(i, 0.0)
    d_ap = enc_ap.get(i, 0.0)
    bound = S[i]

    zs_ok = "ok" if d_zs <= bound + 1e-9 else "no"
    ap_ok = "ok" if d_ap <= bound + 1e-9 else "no"

    print(
        f"  {m.msg_id:>5}  "
        f"{d_zs:>8.4f}  "
        f"{d_ap:>8.4f}  "
        f"{bound:>8.4f}  "
        f"{zs_ok:>6}  "
        f"{ap_ok:>6}"
    )



# E2E using simulated encapsulation delay


rows_zs = compute_e2e(
    msgset,
    R_src,
    C_src,
    R_dst,
    enc_zs,
    tsn_wcrt,
    delta_dec,
)

rows_ap = compute_e2e(
    msgset,
    R_src,
    C_src,
    R_dst,
    enc_ap,
    tsn_wcrt,
    delta_dec,
)

print(f"\n{'E2E Delay: Analytical CAN/TSN + Simulated Encapsulation':=<65}")
print(
    f"  {'msg':>5}  {'e2e_zs':>9}  {'e2e_ap':>9}  "
    f"{'D':>8}  {'zs':>4}  {'ap':>4}"
)
print(f"  {'-' * 55}")

for rz, ra in zip(rows_zs, rows_ap):
    zs = "✓" if rz["feasible"] else "✗"
    ap = "✓" if ra["feasible"] else "✗"

    print(
        f"  {rz['msg_id']:>5}  "
        f"{rz['e2e']:>9.4f}  "
        f"{ra['e2e']:>9.4f}  "
        f"{rz['deadline']:>8.1f}  "
        f"{zs:>4}  "
        f"{ap:>4}"
    )

n_zs = sum(1 for r in rows_zs if r["feasible"])
n_ap = sum(1 for r in rows_ap if r["feasible"])

print(
    f"\n  ZS feasible: {n_zs}/{len(msgset)}"
    f"  AP feasible: {n_ap}/{len(msgset)}"
)



# Latency characterization summary


zs_vals = list(enc_zs.values())
ap_vals = list(enc_ap.values())

print(f"\n{'Latency Characterization Summary':=<65}")

print("  FIFO-ZS:")
print(f"    mean max-enc per flow : {mean(zs_vals):.4f} ms")
print(f"    worst max-enc         : {max(zs_vals):.4f} ms")

print()
print("  FIFO-ZS-AP:")
print(f"    mean max-enc per flow : {mean(ap_vals):.4f} ms")
print(f"    worst max-enc         : {max(ap_vals):.4f} ms")

print()
print("  Improvement:")
print(
    f"    mean improvement      : "
    f"{mean(z - a for z, a in zip(zs_vals, ap_vals)):.4f} ms"
)
print(f"    max improvement       : {result['max_improvement']:.4f} ms")