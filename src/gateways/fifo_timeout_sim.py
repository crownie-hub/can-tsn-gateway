# src/gateways/fifo_timeout_sim.py
# FIFO-Timeout simulation using actual CAN gateway arrivals.
# All times in milliseconds (ms).
#
# Gateway fwds every T_tsn ms regardless of buffer fill.


from dataclasses import dataclass


@dataclass
class BatchResult:
    batch_id:  int
    trigger:   str
    fwd_time:  float
    instances: list


def simulate_fifo_timeout(instances, T_tsn, batch_size,
                           start_offset=0.0, flush_partial=True):
    """
    FIFO-Timeout simulation.

    Parameters
    ----------
    instances    : list[CANInstance] sorted by arrive_gw
    T_tsn        : float — TSN frame period / GCL cycle (ms)
    batch_size   : int   — max messages per frame
    start_offset : float — time of first TSN fwd (ms)
    flush_partial: bool  — emit final partial batch

    Returns
    -------
    list[BatchResult]
    """
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))
    if not instances:
        return []

    T_tsn  = float(T_tsn)
    t_fwd = float(start_offset) + T_tsn   # first fwd time
    batches   = []
    buffer    = []
    batch_id  = 0
    idx       = 0
    n         = len(instances)

    while idx < n or buffer:
        # collect all arrivals up to this fwd time
        while idx < n and float(instances[idx].arrive_gw) <= t_fwd:
            buffer.append(instances[idx])
            idx += 1

        # emit one frame per fwd (up to batch_size messages)
        if buffer:
            batch  = buffer[:batch_size]
            buffer = buffer[batch_size:]
            batches.append(BatchResult(
                batch_id=batch_id, trigger="timeout",
                fwd_time=t_fwd, instances=batch))
            batch_id += 1

        t_fwd += T_tsn

        # stop advancing time if no more arrivals and buffer empty
        if idx >= n and not buffer:
            break

    # flush remaining if any (shouldn't happen with correct T_tsn loop
    # but handles edge cases)
    if buffer and flush_partial:
        batches.append(BatchResult(
            batch_id=batch_id, trigger="end_of_sim",
            fwd_time=t_fwd - T_tsn,
            instances=buffer))

    return batches


def compute_batch_enc_delays(batches):
    rows = []
    for b in batches:
        for inst in b.instances:
            rows.append({
                "flow_id":   int(inst.flow_id),
                "inst_id":   int(inst.inst_id),
                "arrive_gw": float(inst.arrive_gw),
                "fwd_time":  float(b.fwd_time),
                "enc_delay": float(b.fwd_time) - float(inst.arrive_gw),
                "trigger":   b.trigger,
                "batch_id":  b.batch_id,
            })
    return rows


def print_timeout_batch_trace(batches, msgset=None, n_show=10):
    print("\nFIFO-Timeout Batch Trace=====================================")
    print("  fwds every T_tsn ms — collects all arrivals since last fwd\n")
    for b in batches[:n_show]:
        if not b.instances:
            continue
        max_enc = max(float(b.fwd_time) - float(i.arrive_gw)
                      for i in b.instances)
        print(f"  Batch {b.batch_id+1}  |F_j|={len(b.instances)}  "
              f"fwd={b.fwd_time:.4f}  enc={max_enc:.4f} ms  [{b.trigger}]")
        print(f"    {'msg':>5}  {'arrive_gw':>10}  {'enc_wait':>10}")
        print("    " + "-" * 32)
        for inst in b.instances:
            fid  = int(inst.flow_id)
            name = msgset[fid].msg_id if msgset else str(fid)
            a_i  = float(inst.arrive_gw)
            print(f"    {name:>5}  {a_i:>10.4f}  "
                  f"{float(b.fwd_time)-a_i:>10.4f} ms")
        print()
