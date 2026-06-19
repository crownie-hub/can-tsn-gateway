# example/fifo_zs_example.py
import os, sys
from collections import defaultdict
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.message    import message_set
from src.can.bus        import CANBusConfig, compute_tx_times, compute_response_times
from src.can.simulator  import CANBusSimulator, NodeConfig, MessageConfig
from src.can.sim_bridge import build_instances_from_sim
from src.tsn.gcl        import GCL
from src.tsn.flow       import Flow
from src.tsn.wcrt       import TSNConfig, wcrt_ms
from src.gateways.fifo_bf       import FIFOBatchGateway
from src.gateways.fifo_bf_sim   import simulate_fifo_bf, print_bf_batch_trace
from src.gateways.fifo_zs       import compute_slacks, FIFOZeroSlackGateway
from src.gateways.fifo_zs_sim   import simulate_fifo_zs, print_zs_batch_trace
from src.gateways.fifo_zs_ap    import FIFOZeroSlackAPGateway
from src.gateways.fifo_zs_ap_sim import simulate_fifo_zs_ap
from src.gateways.fifo_timeout     import FIFOTimeoutGateway
from src.gateways.fifo_timeout_sim import simulate_fifo_timeout, print_timeout_batch_trace
from src.gateways.decap         import decap_delay
from src.gateways.decap_sim     import fifo_decap_releases
from src.gateways.dest_can_sim   import (compute_mat_values,
                                         decap_releases_to_dst_events,
                                         FIFODestGateway, MATDestGateway,
                                         summarize_dst_results,
                                         summarize_dst_per_flow)
from example.evaluation    import compute_max_enc_delay, verify_theorem_bound

BATCH_SIZE    = 10
SIM_HORIZON   = 200.0
DECAP_UNIT_MS = 0.0
T_TSN         = 6.0    # GCL cycle ms — FIFO-Timeout fires every T_TSN

msgset     = message_set()
priorities = [m.priority     for m in msgset]
periods    = [m.period       for m in msgset]
payloads   = [m.payload_size for m in msgset]

def build_nodes(msgset):
    grouped = defaultdict(list)
    for m in msgset:
        grouped[m.source_id].append(MessageConfig(
            msg_id=m.priority, period=m.period,
            payload_bytes=m.payload_size, deadline=m.deadline, name=m.msg_id))
    return [NodeConfig(src, f"ECU{src}", msgs)
            for src, msgs in sorted(grouped.items())]

def print_slacks(msgset, R_src, C_src, R_dst, S):
    print(f"\n{'Slack values':=<65}")
    print(f"  {'msg':>5}  {'R_src':>8}  {'C':>8}  {'R_dst':>8}  {'S_i':>10}")
    print("  " + "-" * 52)
    for m, r, c, rd, s in zip(msgset, R_src, C_src, R_dst, S):
        print(f"  {m.msg_id:>5}  {r:>8.4f}  {c:>8.4f}  {rd:>8.4f}  {s:>10.4f}")

def print_overall_summary(results_map):
    print(f"\n{'Overall Comparison':=<65}")
    print(f"  {'scheme':<28}{'mean_e2e':>12}{'worst_e2e':>14}{'misses':>10}")
    print("  " + "-" * 62)
    for name, results in results_map.items():
        if not results: continue
        mean  = sum(r["total_e2e_ms"] for r in results) / len(results)
        worst = max(r["total_e2e_ms"] for r in results)
        miss  = sum(1 for r in results if not r["met_deadline"])
        print(f"  {name:<28}{mean:>12.4f}{worst:>14.4f}{miss:>10}")

# ── CAN setup 
can_cfg   = CANBusConfig(bus_type="CAN", tbit=0.002)
C_src     = compute_tx_times(payloads, can_cfg)
R_src     = compute_response_times(priorities, periods, payloads, can_cfg)
R_dst     = list(R_src)

# ── TSN setup 
tsn_cfg    = TSNConfig(link_speed_mbps=1000, num_switches=1,
                       switch_processing_us=3.0, propagation_delay_us=1.0)
gcl        = GCL.sample_uniform(cycle_us=1000, window_us=200)
wc_payload = 17 + sum(sorted(payloads, reverse=True)[:BATCH_SIZE])
tsn_wcrt   = wcrt_ms(Flow(0, min(periods), wc_payload, 0), [], gcl, tsn_cfg)
delta_dec  = decap_delay(BATCH_SIZE, max(C_src))
S          = compute_slacks(msgset, R_src, C_src, tsn_wcrt, delta_dec, R_dst)

# ── Analytical gateways  
gw_bf = FIFOBatchGateway(msgset, R_src, C_src, BATCH_SIZE, tsn_wcrt, max(C_src))
gw_zs = FIFOZeroSlackGateway(msgset, R_src, C_src, S, BATCH_SIZE, tsn_wcrt, max(C_src))
gw_ap = FIFOZeroSlackAPGateway(msgset, R_src, C_src, S, BATCH_SIZE, tsn_wcrt, max(C_src))

gw_to = FIFOTimeoutGateway(msgset, R_src, C_src, T_TSN, BATCH_SIZE,
                            tsn_wcrt, max(C_src))

print("=" * 65)
print("Analytical Bounds")
print("=" * 65)
print(f"  FIFO-BF     delta_enc = {gw_bf.delta_enc:.4f} ms")
print(f"  FIFO-ZS     delta_enc = {gw_zs.delta_enc:.4f} ms")
print(f"  FIFO-ZS-AP  delta_enc in [{gw_ap.delta_enc_lower:.4f}, "
      f"{gw_ap.delta_enc_upper:.4f}] ms")
print(f"  prediction_gain       = {gw_ap.prediction_gain:.4f} ms")
print(f"  FIFO-TO     delta_enc = {gw_to.delta_enc:.4f} ms  "
      f"[mu={gw_to.mu}, T_tsn={T_TSN}ms]")
print_slacks(msgset, R_src, C_src, R_dst, S)

# ── CAN simulation 
nodes     = build_nodes(msgset)
sim       = CANBusSimulator(nodes, can_cfg, SIM_HORIZON)
done      = sim.run()
instances = build_instances_from_sim(done, use_arrival="finish")
print(f"\nCAN simulation: {len(done)} frames → {len(instances)} instances")

# ── Gateway simulations  
batches_bf                  = simulate_fifo_bf(instances, BATCH_SIZE)
batches_zs                  = simulate_fifo_zs(instances, S, BATCH_SIZE)
batches_ap, prediction_log  = simulate_fifo_zs_ap(
    instances, S, R_src, C_src, BATCH_SIZE, return_prediction_log=True)

batches_to = simulate_fifo_timeout(instances, T_TSN, BATCH_SIZE)

enc_bf = compute_max_enc_delay(batches_bf)
enc_zs = compute_max_enc_delay(batches_zs)
enc_ap = compute_max_enc_delay(batches_ap)
enc_to = compute_max_enc_delay(batches_to)

# Theorem verification
result = verify_theorem_bound(enc_zs, enc_ap, R_src, C_src)
print(f"\n{'Theorem Verification':=<65}")
print(f"  gain={result['gain']:.4f}  "
      f"max_improvement={result['max_improvement']:.4f}  "
      f"bound_holds={'✓' if result['bound_holds'] else '✗'}")

# Batch traces
print_bf_batch_trace(batches_bf, msgset=msgset, n_show=2)
print_zs_batch_trace(batches_zs, S, msgset=msgset, n_show=2)

print_timeout_batch_trace(batches_to, msgset=msgset, n_show=2)

#Destination simulation 
mat_values = compute_mat_values(msgset, tsn_wcrt, R_dst)

def make_dst_results(batches):
    releases = fifo_decap_releases(batches, msgset, tsn_wcrt, DECAP_UNIT_MS)
    events   = decap_releases_to_dst_events(releases, C_src, mat_values)
    return FIFODestGateway().run(events), MATDestGateway().run(events)

res_bf_fifo, res_bf_mat = make_dst_results(batches_bf)
res_zs_fifo, _          = make_dst_results(batches_zs)
res_ap_fifo, _          = make_dst_results(batches_ap)
res_to_fifo, res_to_mat = make_dst_results(batches_to)

print("\n" + "=" * 65)
print("Destination-Side Simulation Results")
print("=" * 65)
for label, res in [("FIFO-BF + FIFO-DST",    res_bf_fifo),
                    ("FIFO-BF + MAT-DST",     res_bf_mat),
                    ("FIFO-ZS + FIFO-DST",    res_zs_fifo),
                    ("FIFO-ZS-AP + FIFO-DST", res_ap_fifo),
                    ("FIFO-TO + FIFO-DST",    res_to_fifo),
                    ("FIFO-TO + MAT-DST",     res_to_mat)]:
    summarize_dst_results(res, label)

for label, res in [("FIFO-BF + FIFO-DST",    res_bf_fifo),
                    ("FIFO-BF + MAT-DST",     res_bf_mat),
                    ("FIFO-ZS + FIFO-DST",    res_zs_fifo),
                    ("FIFO-ZS-AP + FIFO-DST", res_ap_fifo),
                    ("FIFO-TO + FIFO-DST",    res_to_fifo),
                    ("FIFO-TO + MAT-DST",     res_to_mat)]:
    summarize_dst_per_flow(res, msgset, label)

print_overall_summary({
    "FIFO-BF + FIFO-DST":    res_bf_fifo,
    "FIFO-BF + MAT-DST":     res_bf_mat,
    "FIFO-ZS + FIFO-DST":    res_zs_fifo,
    "FIFO-ZS-AP + FIFO-DST": res_ap_fifo,
    "FIFO-TO + FIFO-DST":    res_to_fifo,
    "FIFO-TO + MAT-DST":     res_to_mat,
})