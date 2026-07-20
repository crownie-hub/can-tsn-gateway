# src/gateways/decap_sim.py
# Decapsulation simulation - converts gateway batches into destination CAN events for the destination scheduler.
# All times in milliseconds (ms).

from dataclasses import dataclass


@dataclass
class DecapRelease:
    flow_id:             int
    inst_id:             int
    can_id:              int
    msg_name:            str
    src_release:         float
    src_arrive_gw:       float
    gateway_fwd_time:    float
    tsn_arrive_dst:      float
    decap_release_time:  float
    payload_size:        int
    period:              float
    deadline:            float
    batch_id:            int
    position:            int


def fifo_decap_releases(batches, msgset, tsn_delay_ms):
    """
    Convert gateway batches into destination CAN release events.

    Scheduler (FIFO/FP/MAT) decides ordering.
    """
    releases = []

    for batch in batches:
        tsn_arrive = float(batch.fwd_time) + float(tsn_delay_ms)

        # CAN tx loop serializes via tx_end.
        for pos, inst in enumerate(batch.instances):
            fid = int(inst.flow_id)
            m   = msgset[fid]
            releases.append(DecapRelease(
                flow_id=fid,
                inst_id=int(inst.inst_id),
                can_id=int(inst.can_id),
                msg_name=m.msg_id,
                src_release=float(inst.release),
                src_arrive_gw=float(inst.arrive_gw),
                gateway_fwd_time=float(batch.fwd_time),
                tsn_arrive_dst=tsn_arrive,
                decap_release_time=tsn_arrive,
                payload_size=int(inst.payload_size),
                period=float(inst.period),
                deadline=float(inst.deadline),
                batch_id=int(batch.batch_id),
                position=pos,
            ))

    return sorted(releases,
                  key=lambda r: (r.decap_release_time, r.can_id, r.inst_id))


def print_decap_releases(releases, n_show=20):
    print("\nFIFO Decapsulation Releases===================================")
    print(f"  {'msg':>5}  {'inst':>5}  {'batch':>5}  {'pos':>4}  "
          f"{'src_rel':>9}  {'gw_fwd':>9}  {'tsn_dst':>9}  {'dst_rel':>9}")
    print("  " + "-" * 72)
    for r in releases[:n_show]:
        print(f"  {r.msg_name:>5}  {r.inst_id:>5}  {r.batch_id:>5}  "
              f"{r.position:>4}  {r.src_release:>9.4f}  "
              f"{r.gateway_fwd_time:>9.4f}  {r.tsn_arrive_dst:>9.4f}  "
              f"{r.decap_release_time:>9.4f}")