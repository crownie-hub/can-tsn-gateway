# src/gateways/fifo_fp_sim.py
#
# Fixed-Priority Buffer-Full simulation.
#
# Same as FIFO-BF but messages are sorted by CAN priority
# The batch sends when n frames accumulate.
#
# All times in milliseconds (ms).

from .fifo_zs_sim import BatchResult


def simulate_fifo_fp(instances, batch_size):
    """
    Fixed-Priority Buffer-Full simulation.

    At each buffer-full event, the n buffered messages are
    sorted by CAN priority (ascending can_id) before the
    batch is forwarded.  This determines their position in
    the TSN frame and therefore their decapsulation order
    at the destination.

    Parameters
    ----------
    instances  : list[CANInstance]  sorted by arrive_gw
    batch_size : int

    Returns
    -------
    list[BatchResult]
    """
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))

    batches  = []
    F_j      = []
    batch_id = 0

    for inst in instances:
        F_j.append(inst)

        if len(F_j) >= batch_size:
            now = float(inst.arrive_gw)

            # sort by CAN priority before forwarding
            F_j_sorted = sorted(F_j, key=lambda i: int(i.can_id))

            batches.append(BatchResult(
                batch_id  = batch_id,
                trigger   = "buffer_full",
                fwd_time  = now,
                L_j       = now,
                instances = F_j_sorted,
            ))
            batch_id += 1
            F_j = []

    # flush
    if F_j:
        now      = float(F_j[-1].arrive_gw)
        F_j_sort = sorted(F_j, key=lambda i: int(i.can_id))
        batches.append(BatchResult(
            batch_id  = batch_id,
            trigger   = "buffer_full",
            fwd_time  = now,
            L_j       = now,
            instances = F_j_sort,
        ))

    return batches
