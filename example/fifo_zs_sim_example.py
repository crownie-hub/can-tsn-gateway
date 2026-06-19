# example/fifo_zs_sim_example.py
#
# Simulation comparison for FIFO-BF, FIFO-ZS, and FIFO-ZS-AP.
#


import os
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.message import message_set
from src.can.bus import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge import build_instances_from_sim

from src.tsn.gcl import GCL
from src.tsn.flow import Flow
from src.tsn.wcrt import TSNConfig, wcrt_ms

from src.gateways.decap import decap_delay

from src.gateways.fifo_bf_sim import (
    simulate_fifo_bf,
    print_bf_batch_trace,
)

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

from example.evaluation import (
    compute_max_enc_delay,
    compute_e2e,
    verify_theorem_bound,
)


  
# Configuration
  

BATCH_SIZE = 5
SIM_HORIZON = 200.0


  
# Small helpers
  

def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else 0.0


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
    by_msg = defaultdict(list)

    for f in done:
        by_msg[f.msg_index].append(f)

    print(f"\nCAN simulation: {len(done)} frames in {SIM_HORIZON} ms")

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


def print_slacks(msgset, R_src, C_src, R_dst, S):
    print(f"\n{'Slack values':=<65}")
    print(
        f"  {'msg':>5}  {'R_src':>8}  {'C':>8}  "
        f"{'R_dst':>8}  {'S_i':>10}"
    )
    print(f"  {'-' * 50}")

    for m, r, c, rd, s in zip(msgset, R_src, C_src, R_dst, S):
        print(
            f"  {m.msg_id:>5}  "
            f"{r:>8.4f}  "
            f"{c:>8.4f}  "
            f"{rd:>8.4f}  "
            f"{s:>10.4f}"
        )


def print_zero_slack_safety(msgset, enc_zs, enc_ap, S):
    print(f"\n{'Zero-Slack Safety Check: Sim Enc Delay <= S_i':=<65}")
    print(
        f"  {'msg':>5}  "
        f"{'sim_zs':>8}  "
        f"{'sim_ap':>8}  "
        f"{'S_i':>8}  "
        f"{'zs_ok':>6}  "
        f"{'ap_ok':>6}"
    )
    print(f"  {'-' * 55}")

    for i, m in enumerate(msgset):
        d_zs = enc_zs.get(i, 0.0)
        d_ap = enc_ap.get(i, 0.0)
        bound = S[i]

        zs_ok = d_zs <= bound + 1e-9
        ap_ok = d_ap <= bound + 1e-9

        print(
            f"  {m.msg_id:>5}  "
            f"{d_zs:>8.4f}  "
            f"{d_ap:>8.4f}  "
            f"{bound:>8.4f}  "
            f"{'✓' if zs_ok else '✗':>6}  "
            f"{'✓' if ap_ok else '✗':>6}"
        )


def print_prediction_summary(prediction_log, msgset, R_src, C_src):
    print(f"\n{'Prediction Error Summary Per Flow':=<65}")
    print(f"  {'msg':>5}  {'max_error':>10}  {'bound':>10}  {'ok':>4}")
    print(f"  {'-' * 38}")

    by_flow = defaultdict(list)

    for row in prediction_log:
        by_flow[row["flow_id"]].append(row)

    for i, m in enumerate(msgset):
        rows = by_flow.get(i, [])
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


def print_improvement_bound(enc_zs, enc_ap, R_src, C_src, msgset):
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
        f"\n  {'flow':>5}  "
        f"{'delta_zs':>10}  "
        f"{'delta_ap':>10}  "
        f"{'improvement':>12}  "
        f"{'ok':>4}"
    )
    print(f"  {'-' * 50}")

    for row in result["rows"]:
        m = msgset[row["flow_id"]]
        ok = "✓" if row["within_bound"] else "✗"

        print(
            f"  {m.msg_id:>5}  "
            f"{row['delta_zs']:>10.4f}  "
            f"{row['delta_ap']:>10.4f}  "
            f"{row['improvement']:>12.4f}  "
            f"{ok:>4}"
        )

    return result


def print_enc_comparison(msgset, enc_bf, enc_zs, enc_ap, S):
    print(f"\n{'Encapsulation Delay Comparison':=<65}")
    print(
        f"  {'msg':>5}  "
        f"{'FIFO-BF':>9}  "
        f"{'FIFO-ZS':>9}  "
        f"{'ZS-AP':>9}  "
        f"{'S_i':>9}"
    )
    print(f"  {'-' * 55}")

    for i, m in enumerate(msgset):
        print(
            f"  {m.msg_id:>5}  "
            f"{enc_bf.get(i, 0.0):>9.4f}  "
            f"{enc_zs.get(i, 0.0):>9.4f}  "
            f"{enc_ap.get(i, 0.0):>9.4f}  "
            f"{S[i]:>9.4f}"
        )


def print_e2e_comparison(rows_bf, rows_zs, rows_ap):
    print(f"\n{'E2E Delay: BF vs ZS vs ZS-AP':=<65}")
    print(
        f"  {'msg':>5}  "
        f"{'BF':>9}  "
        f"{'ZS':>9}  "
        f"{'ZS-AP':>9}  "
        f"{'D':>8}  "
        f"{'bf':>4}  "
        f"{'zs':>4}  "
        f"{'ap':>4}"
    )
    print(f"  {'-' * 70}")

    for rb, rz, ra in zip(rows_bf, rows_zs, rows_ap):
        bf = "✓" if rb["feasible"] else "✗"
        zs = "✓" if rz["feasible"] else "✗"
        ap = "✓" if ra["feasible"] else "✗"

        print(
            f"  {rb['msg_id']:>5}  "
            f"{rb['e2e']:>9.4f}  "
            f"{rz['e2e']:>9.4f}  "
            f"{ra['e2e']:>9.4f}  "
            f"{rb['deadline']:>8.1f}  "
            f"{bf:>4}  "
            f"{zs:>4}  "
            f"{ap:>4}"
        )

    n_bf = sum(1 for r in rows_bf if r["feasible"])
    n_zs = sum(1 for r in rows_zs if r["feasible"])
    n_ap = sum(1 for r in rows_ap if r["feasible"])

    total = len(rows_bf)

    print(
        f"\n  BF feasible: {n_bf}/{total}"
        f"  ZS feasible: {n_zs}/{total}"
        f"  AP feasible: {n_ap}/{total}"
    )


def print_latency_summary(enc_bf, enc_zs, enc_ap, result):
    bf_vals = list(enc_bf.values())
    zs_vals = list(enc_zs.values())
    ap_vals = list(enc_ap.values())

    print(f"\n{'Latency Characterization Summary':=<65}")

    print("  FIFO-BF:")
    print(f"    mean max-enc per flow : {mean(bf_vals):.4f} ms")
    print(f"    worst max-enc         : {max(bf_vals):.4f} ms")

    print()
    print("  FIFO-ZS:")
    print(f"    mean max-enc per flow : {mean(zs_vals):.4f} ms")
    print(f"    worst max-enc         : {max(zs_vals):.4f} ms")

    print()
    print("  FIFO-ZS-AP:")
    print(f"    mean max-enc per flow : {mean(ap_vals):.4f} ms")
    print(f"    worst max-enc         : {max(ap_vals):.4f} ms")

    print()
    print("  Improvement over FIFO-ZS:")
    improvements = [
        enc_zs.get(i, 0.0) - enc_ap.get(i, 0.0)
        for i in enc_zs.keys()
    ]
    print(f"    mean improvement      : {mean(improvements):.4f} ms")
    print(f"    max improvement       : {result['max_improvement']:.4f} ms")


  
# Message set and analytical setup
  

msgset = message_set()

priorities = [m.priority for m in msgset]
periods = [m.period for m in msgset]
payloads = [m.payload_size for m in msgset]

can_cfg = CANBusConfig(
    bus_type="CAN",
    tbit=0.002,
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
    msgset,
    R_src,
    C_src,
    tsn_wcrt,
    delta_dec,
    R_dst,
)

gw_zs = FIFOZeroSlackGateway(
    msgset,
    R_src,
    C_src,
    S,
    BATCH_SIZE,
    tsn_wcrt,
    max(C_src),
)

gw_ap = FIFOZeroSlackAPGateway(
    msgset,
    R_src,
    C_src,
    S,
    BATCH_SIZE,
    tsn_wcrt,
    max(C_src),
)


  
# Print analytical setup
  

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

print_slacks(
    msgset,
    R_src,
    C_src,
    R_dst,
    S,
)


  
# CAN simulation
  

nodes = build_nodes(msgset)

sim = CANBusSimulator(
    nodes,
    can_cfg,
    SIM_HORIZON,
)

done = sim.run()

print_can_sim_summary(
    done,
    msgset,
)

instances = build_instances_from_sim(
    done,
    use_arrival="finish",
)

print(f"\nGateway instances: {len(instances)}")


  
# Gateway simulations
  

batches_bf = simulate_fifo_bf(
    instances,
    BATCH_SIZE,
)

enc_bf = compute_max_enc_delay(
    batches_bf,
)

batches_zs = simulate_fifo_zs(
    instances,
    S,
    BATCH_SIZE,
)

enc_zs = compute_max_enc_delay(
    batches_zs,
)

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


  
# Batch traces
  

print_bf_batch_trace(
    batches_bf,
    msgset=msgset,
    n_show=5,
)

print_zs_batch_trace(
    batches_zs,
    S,
    msgset=msgset,
    n_show=5,
)


  
# Validation and metrics
  

result = print_improvement_bound(
    enc_zs,
    enc_ap,
    R_src,
    C_src,
    msgset,
)

print_prediction_error_log(
    prediction_log,
    msgset=msgset,
    n_show=40,
)

print_prediction_summary(
    prediction_log,
    msgset,
    R_src,
    C_src,
)

print_zero_slack_safety(
    msgset,
    enc_zs,
    enc_ap,
    S,
)

print_enc_comparison(
    msgset,
    enc_bf,
    enc_zs,
    enc_ap,
    S,
)


  
# E2E delay
  

rows_bf = compute_e2e(
    msgset,
    R_src,
    C_src,
    R_dst,
    enc_bf,
    tsn_wcrt,
    delta_dec,
)

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

print_e2e_comparison(
    rows_bf,
    rows_zs,
    rows_ap,
)

print_latency_summary(
    enc_bf,
    enc_zs,
    enc_ap,
    result,
)