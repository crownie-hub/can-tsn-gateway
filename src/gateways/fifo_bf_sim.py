# src/gateways/fifo_bf_sim.py
#
# FIFO buffer-full simulation using actual CAN gateway arrivals.
#
# The gateway transmits a frame once |F_j| = n.
# Remaining messages at the end of the simulation are flushed at the last arrival time.

from dataclasses import dataclass


@dataclass
class BatchResult:
    batch_id: int
    trigger: str
    fwd_time: float
    instances: list


def simulate_fifo_bf(instances, batch_size, flush_partial=True):
  
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))

    batches = []
    batch = []
    batch_id = 0

    for inst in instances:
        batch.append(inst)

        if len(batch) == batch_size:
            fwd_time = float(inst.arrive_gw)

            batches.append(
                BatchResult(
                    batch_id=batch_id,
                    trigger="buffer_full",
                    fwd_time=fwd_time,
                    instances=batch.copy(),
                )
            )

            batch_id += 1
            batch = []

    if batch and flush_partial:
        fwd_time = max(float(inst.arrive_gw) for inst in batch)

        batches.append(
            BatchResult(
                batch_id=batch_id,
                trigger="end_of_sim",
                fwd_time=fwd_time,
                instances=batch.copy(),
            )
        )

    return batches


def compute_batch_enc_delays(batches):
    """
    Return one row per message instance with its encapsulation delay.
    """

    rows = []

    for b in batches:
        for inst in b.instances:
            rows.append(
                {
                    "flow_id": int(inst.flow_id),
                    "inst_id": int(inst.inst_id),
                    "arrive_gw": float(inst.arrive_gw),
                    "fwd_time": float(b.fwd_time),
                    "enc_delay": float(b.fwd_time) - float(inst.arrive_gw),
                    "trigger": b.trigger,
                    "batch_id": b.batch_id,
                }
            )

    return rows


def print_bf_batch_trace(batches, msgset=None, n_show=10):
    """
    Print FIFO-BF batch trace.
    """

    print("\nFIFO-BF Batch Trace==========================================")
    print("  Trigger: buffer-full, or end-of-sim for the final partial batch")

    for b in batches[:n_show]:
        max_enc = max(
            float(b.fwd_time) - float(inst.arrive_gw)
            for inst in b.instances
        )

        print(
            f"\n  Batch {b.batch_id + 1}"
            f"  |F_j|={len(b.instances)}"
            f"  fwds={b.fwd_time:.4f}"
            f"  enc={max_enc:.4f} ms"
            f"  [{b.trigger}]"
        )

        print(
            f"    {'msg':>5}  "
            f"{'arrive_gw':>10}  "
            f"{'enc_wait':>10}"
        )
        print("    " + "-" * 32)

        for inst in b.instances:
            flow_id = int(inst.flow_id)

            if msgset is not None:
                name = msgset[flow_id].msg_id
            else:
                name = str(flow_id)

            a_i = float(inst.arrive_gw)
            enc = float(b.fwd_time) - a_i

            print(
                f"    {name:>5}  "
                f"{a_i:>10.4f}  "
                f"{enc:>10.4f} ms"
            )