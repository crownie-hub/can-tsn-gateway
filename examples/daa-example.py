# examples/run_daa_gtw.py
from collections import defaultdict
import math

from src.timing.can_bus import CANBusConfig, compute_response_times_ms, compute_tx_times_ms
from src.can_message import message_set
from src.pipeline.can_instances import build_instances_to_gateway
from src.gateways.dest_gateway import decap_delay_ms
from src.tsn.gcl import GCL
from src.timing.tsn_wcrt import TSNConfig, TSNWCRTCalculator
from src.gateways.deadline_aware_gateway import PredictiveDAAGateway


def print_worst_per_message(results):
    rows = results["per_message"]
    by_flow = defaultdict(list)
    for r in rows:
        by_flow[int(r["flow_id"])].append(r)

    print("\nWorst-case E2E delay per CAN flow (max over instances)")
    print(f"{'Msg':>6}  {'WorstInst':>10}  {'E2E(ms)':>10}  {'GW+TSN':>10}  {'Decap':>8}  {'DestCAN':>8}")
    print("-" * 68)

    for flow_id in sorted(by_flow.keys()):
        worst = max(by_flow[flow_id], key=lambda x: x["total_e2e_ms"])
        print(
            f"{flow_id:6d}  {worst['message_id']:>10}  {worst['total_e2e_ms']:10.2f}  "
            f"{worst['total_delay_ms']:10.2f}  {worst['decap_ms_wc']:8.2f}  {worst['dest_can_wcrt_ms']:8.2f}"
        )


def main(dst_model: str = "tx"):
    msgset = message_set()

    priorities = [int(m.msg_id, 16) for m in msgset]
    periods_ms = [float(m.period) for m in msgset]
    payloads = [int(m.payload_size) for m in msgset]

    cfg = CANBusConfig(bus_type="CAN", tbit_ms=0.004, dtbit_ms=0.0005)

    C_ms = compute_tx_times_ms(payloads, cfg)
    R_ms = compute_response_times_ms(priorities, periods_ms, payloads, cfg)

    hyperperiod = math.lcm(*map(int, periods_ms)) *2
    instances = build_instances_to_gateway(msgset=msgset, R_ms=R_ms, horizon_ms=hyperperiod)

    gcl = GCL.sample_uniform(cycle_us=1000, window_us=200)
    tsn_cfg = TSNConfig(link_speed_mbps=1000, num_switches=2, switch_processing_us=3, propagation_delay_us=1)
    tsn_calc = TSNWCRTCalculator(tsn_cfg, gcl)

    decap_ms = decap_delay_ms(batch_size=5)  # or match your expected avg batch size

    gw = PredictiveDAAGateway(
        tsn_calc=tsn_calc,
        decap_delay_ms=decap_ms,
        R_ms=R_ms,
        C_ms=C_ms,
        dst_model=dst_model,  # "none" or "tx"
        default_tsn_priority=0,
        batch_header_bytes=0,
        per_can_header_bytes=0,
    )

    results = gw.run_on_instances(instances, length=hyperperiod)

    # E2E = (gateway wait + TSN) + decap + dest CAN WCRT
    for r in results["per_message"]:
        flow = int(r["flow_id"])
        r["decap_ms_wc"] = decap_ms
        r["dest_can_wcrt_ms"] = float(R_ms[flow])
        r["total_e2e_ms"] = r["total_delay_ms"] + r["decap_ms_wc"] + r["dest_can_wcrt_ms"]

    print(f"Predictive DAA Results (dst_model={dst_model})")
    print("-" * 44)
    print(f"length={hyperperiod:.1f}ms frames={results['num_frames']} instances={len(instances)}")



    print("\nPer-instance (first 30)")
    print(
    f"{'ArrGW':>8}  {'MsgID':>10}  {'Prio':>4}  {'Frame':>7}  {'Pos':>3}  "
    f"{'GW+TSN':>9}  {'Decap':>7}  {'DestCAN':>8}  {'E2E(ms)':>9}")   
    print("-" * 90)

    for r in results["per_message"][:30]:
        print(
            f"{r['arrive_gw_ms']:8.2f}  "
            f"{r['message_id']:>10}  "
            f"{r['frame_priority']:4d}  "
            f"{r['frame_id']:>7}  "
            f"{r['position']:3d}  "
            f"{r['total_delay_ms']:9.2f}  "
            f"{r['decap_ms_wc']:7.2f}  "
            f"{r['dest_can_wcrt_ms']:8.2f}  "
            f"{r['total_e2e_ms']:9.2f}"
        )



    print_worst_per_message(results)


if __name__ == "__main__":
    main(dst_model="tx")   # try "none" too