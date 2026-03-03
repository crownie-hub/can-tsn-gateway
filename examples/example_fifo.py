# examples/example_fifo_gateway_tsn.py
from collections import defaultdict
import math

from src.timing.can_bus import CANBusConfig, compute_response_times_ms
from src.can_message import message_set
from src.pipeline.can_instances import build_instances_full_batches
from src.gateways.dest_gateway import decap_delay_ms
from src.tsn.gcl import GCL
from src.timing.tsn_wcrt import TSNConfig, TSNWCRTCalculator
from src.gateways.fifo_gateway import FIFOBatchGateway


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


def main():
    msgset = message_set()

    priorities = [int(m.msg_id, 16) for m in msgset]
    periods_ms = [m.period for m in msgset]
    payloads = [m.payload_size for m in msgset]

    cfg = CANBusConfig(bus_type="CAN", tbit_ms=0.004, dtbit_ms=0.0005)
    R_ms = compute_response_times_ms(priorities, periods_ms, payloads, cfg)

    hyperperiod = math.lcm(*map(int, periods_ms))
    batch_size = 5

    instances, extended_horizon, base_count, remainder = build_instances_full_batches(
        msgset=msgset, R_ms=R_ms, base_horizon_ms=hyperperiod, batch_size=batch_size
    )

    gcl = GCL.sample_uniform(cycle_us=1000, window_us=200)

    tsn_cfg = TSNConfig(
        link_speed_mbps=1000,
        num_switches=2,
        switch_processing_us=3,
        propagation_delay_us=1,
    )
    tsn_calc = TSNWCRTCalculator(tsn_cfg, gcl)

    gw = FIFOBatchGateway(
        batch_size=batch_size,
        tsn_calc=tsn_calc,
        default_priority=0,
        batch_header_bytes=0,
        per_can_header_bytes=0,
    )

    results = gw.run_on_instances(instances, length=hyperperiod)

    decap_ms = decap_delay_ms(batch_size=batch_size)
    for r in results["per_message"]:
        flow = int(r["flow_id"])
        r["decap_ms_wc"] = decap_ms
        r["dest_can_wcrt_ms"] = float(R_ms[flow])
        r["total_e2e_ms"] = r["total_delay_ms"] + r["decap_ms_wc"] + r["dest_can_wcrt_ms"]

    print("FIFO Gateway Results")
    print("-" * 32)
    print(
        f"length={hyperperiod:.1f}ms"
        f" instances={base_count} remainder={remainder} batches={results['num_batches']}"
    )

    print("\nPer-instance (first 10)")
    print(
        f"{'ArrGW':>8}  {'MsgID':>10}  {'Prio':>4}  {'Batch':>5}  {'Pos':>3}  "
        f"{'GW+TSN':>9}  {'Decap':>7}  {'DestCAN':>8}  {'E2E(ms)':>9}"
    )
    print("-" * 80)

    for r in results["per_message"][:10]:
        print(
            f"{r['arrive_gw_ms']:8.2f}  {r['message_id']:>10}  {r['frame_priority']:4d}  "
            f"{r['batch_number']:5d}  {r['position']:3d}  "
            f"{r['total_delay_ms']:9.2f}  {r['decap_ms_wc']:7.2f}  {r['dest_can_wcrt_ms']:8.2f}  {r['total_e2e_ms']:9.2f}"
        )

    print_worst_per_message(results)


if __name__ == "__main__":
    main()