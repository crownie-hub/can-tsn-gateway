# src/gateways/dst_can_sim.py
#
# Destination CAN gateway schedulers:
#   FIFO, FP, MAT
#
# All times in milliseconds (ms).
#
# MAT reference:
#   Xie et al.
#   "A High-Flexibility CAN-TSN Gateway With a
#    Low-Congestion TSN-to-CAN Scheduler"
#   IEEE TCAD 2023.
#
#  MAT implementation:
#
#   MAT(m_i)
#       =
#   D_i
#   - R_TSN
#   - delta_dec_i
#   - R_dst_i
#
# where:
#
#   delta_dec_i =
#       decap_release_time - tsn_arrive_dst
#
# Smallest MAT = least slack = most urgent.

from abc import ABC, abstractmethod
from collections import defaultdict


  
# base scheduler
  

class DestGateway(ABC):

    def __init__(self, default_tx_time_ms=0.5):
        self.default_tx_time_ms = float(default_tx_time_ms)

    @abstractmethod
    def priority(self, item, now):
        pass

    def run(self, events):
        if not events:
            return []

        events  = sorted(events, key=lambda e: float(e["queue_insert_ms"]))
        ready   = []
        results = []
        seq     = 0
        idx     = 0
        now     = float(events[0]["queue_insert_ms"])

        while idx < len(events) or ready:

            while (idx < len(events) and
                   float(events[idx]["queue_insert_ms"]) <= now + 1e-9):
                ready.append((seq, events[idx]))
                seq += 1
                idx += 1

            if not ready:
                now = float(events[idx]["queue_insert_ms"])
                continue

            ready.sort(key=lambda x: (self.priority(x[1], now), x[0]))
            _, event = ready.pop(0)

            tx_time  = float(event.get("can_tx_time_ms", self.default_tx_time_ms))
            tx_start = now
            tx_end   = tx_start + tx_time

            row = dict(event)
            row["dest_gw_wait_ms"]  = tx_start - float(event["queue_insert_ms"])
            row["dest_tx_start_ms"] = tx_start
            row["dest_tx_end_ms"]   = tx_end
            row["dest_complete_ms"] = tx_end
            row["total_e2e_ms"]     = tx_end - float(event["src_release"])
            row["met_deadline"]     = (row["total_e2e_ms"] <=
                                       float(event["deadline_ms"]) + 1e-9)
            results.append(row)
            now = tx_end

        return results


  
# FIFO scheduler
  

class FIFODestGateway(DestGateway):

    def priority(self, item, now):
        return (
            float(item["queue_insert_ms"]),
            int(item.get("can_id", item.get("flow_id", 0))),
            int(item.get("inst_id", 0)),
        )


  
# FP scheduler
  

class FPDestGateway(DestGateway):

    def priority(self, item, now):
        return (
            int(item.get("can_id", item.get("flow_id", 0))),
            float(item["queue_insert_ms"]),
            int(item.get("inst_id", 0)),
        )


  
# MAT scheduler
  

class MATDestGateway(DestGateway):
    """
    Paper-faithful MAT scheduler.

    MAT(m_i) = D_i - R_TSN - delta_dec_i - R_dst_i

    Smaller MAT = less remaining slack = more urgent.
    """

    def priority(self, item, now):
        mat = float(item.get("mat_ms", float("inf")))
        return (
            mat,
            int(item.get("can_id", item.get("flow_id", 0))),
            float(item["queue_insert_ms"]),
            int(item.get("inst_id", 0)),
        )


  
# decap release -> destination events
  

def decap_releases_to_dst_events(
    releases,
    C,
    tsn_delay_ms,
    R_dst,
):
    """
    Convert DecapRelease objects into destination gateway events.
    """
    events = []

    for r in releases:
        fid = int(r.flow_id)

        # actual per-message decap serialization delay
        delta_dec_i = (float(r.decap_release_time)
                       - float(r.tsn_arrive_dst))

        mat = (float(r.deadline)
               - float(tsn_delay_ms)
               - float(delta_dec_i)
               - float(R_dst[fid]))

        events.append({
            "flow_id":          fid,
            "inst_id":          int(r.inst_id),
            "can_id":           int(r.can_id),
            "msg_name":         r.msg_name,
            "src_release":      float(r.src_release),
            "src_arrive_gw":    float(r.src_arrive_gw),
            "gateway_fwd_time": float(r.gateway_fwd_time),
            "tsn_arrive_dst":   float(r.tsn_arrive_dst),
            "queue_insert_ms":  float(r.decap_release_time),
            "deadline_ms":      float(r.deadline),
            "period_ms":        float(r.period),
            "payload_size":     int(r.payload_size),
            "batch_id":         int(r.batch_id),
            "position":         int(r.position),
            "can_tx_time_ms":   float(C[fid]),
            "mat_ms":           float(mat),
        })

    return sorted(events,
                  key=lambda e: (e["queue_insert_ms"],
                                 e["can_id"],
                                 e["inst_id"]))


  
# summaries
  

def summarize_dst_results(results, label):
    if not results:
        print(f"\n{label}: no results")
        return

    misses     = sum(1 for r in results if not r["met_deadline"])
    mean_e2e   = sum(r["total_e2e_ms"]    for r in results) / len(results)
    worst_e2e  = max(r["total_e2e_ms"]    for r in results)
    mean_wait  = sum(r["dest_gw_wait_ms"] for r in results) / len(results)
    worst_wait = max(r["dest_gw_wait_ms"] for r in results)

    print(f"\n{label} Destination Results=================================")
    print(f"  completed  : {len(results)}")
    print(f"  misses     : {misses}")
    print(f"  mean wait  : {mean_wait:.4f} ms  worst: {worst_wait:.4f} ms")
    print(f"  mean e2e   : {mean_e2e:.4f} ms  worst: {worst_e2e:.4f} ms")


def summarize_dst_per_flow(results, msgset, label):
    print(f"\n{label} Per-Flow Results=====================================")
    print(f"  {'msg':>5}  {'n':>5}  {'mean_e2e':>10}  "
          f"{'max_e2e':>10}  {'mean_wait':>10}  {'miss':>5}")
    print("  " + "-" * 58)

    by_flow = defaultdict(list)
    for r in results:
        by_flow[int(r["flow_id"])].append(r)

    for i, m in enumerate(msgset):
        rows = by_flow.get(i, [])
        if not rows:
            print(f"  {m.msg_id:>5}  {0:>5}  {'—':>10}  "
                  f"{'—':>10}  {'—':>10}  {0:>5}")
            continue
        print(f"  {m.msg_id:>5}  {len(rows):>5}  "
              f"{sum(r['total_e2e_ms'] for r in rows)/len(rows):>10.4f}  "
              f"{max(r['total_e2e_ms'] for r in rows):>10.4f}  "
              f"{sum(r['dest_gw_wait_ms'] for r in rows)/len(rows):>10.4f}  "
              f"{sum(1 for r in rows if not r['met_deadline']):>5}")


def print_dst_trace(results, n_show=20):
    print("\nDestination CAN Trace========================================")
    print(f"  {'msg':>5}  {'inst':>5}  {'rel_dst':>9}  {'tx_start':>9}  "
          f"{'tx_end':>9}  {'wait':>8}  {'e2e':>9}  {'ok':>4}")
    print("  " + "-" * 72)
    for r in results[:n_show]:
        ok = "✓" if r["met_deadline"] else "✗"
        print(f"  {r['msg_name']:>5}  {r['inst_id']:>5}  "
              f"{r['queue_insert_ms']:>9.4f}  "
              f"{r['dest_tx_start_ms']:>9.4f}  "
              f"{r['dest_tx_end_ms']:>9.4f}  "
              f"{r['dest_gw_wait_ms']:>8.4f}  "
              f"{r['total_e2e_ms']:>9.4f}  {ok:>4}")