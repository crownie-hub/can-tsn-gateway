# src/gateways/fifo_fp_timeout_sim.py
#
# Fixed-Priority Timeout simulation.
#
# Timer triggers every T_GW ms. At each release, the n
# highest-priority buffered messages are selected (FP ordering).
#
# All times in milliseconds (ms).

import math
from .fifo_zs_sim import BatchResult


def simulate_fifo_fp_timeout(instances, gateway_period, batch_size):
    """
    Fixed-Priority Timeout simulation.

    Parameters
    ----------
    instances      : list[CANInstance]
    gateway_period : float  — T_GW in ms
    batch_size     : int    — max messages per TSN frame

    Returns
    -------
    list[BatchResult]
    """
    if not instances:
        return []

    instances = sorted(instances,
                       key=lambda x: (float(x.arrive_gw),
                                      int(x.flow_id),
                                      int(x.inst_id)))

    # first release instant after first arrival
    first_arrive = float(instances[0].arrive_gw)
    t_release    = math.ceil(first_arrive / gateway_period) * gateway_period
    if t_release <= first_arrive:
        t_release += gateway_period

    batches  = []
    buffer   = []
    batch_id = 0
    idx      = 0
    n        = len(instances)

    while idx < n or buffer:
        # enqueue all arrivals up to this release
        while idx < n and float(instances[idx].arrive_gw) <= t_release:
            buffer.append(instances[idx])
            idx += 1

        if buffer:
            # FP: sort by can_id ascending (highest priority first)
            buffer.sort(key=lambda i: (int(i.can_id),
                                       float(i.arrive_gw),
                                       int(i.flow_id),
                                       int(i.inst_id)))

            # select top batch_size messages
            batch   = buffer[:batch_size]
            buffer  = buffer[batch_size:]

            # sort selected batch by priority for TSN frame ordering
            batch_sorted = sorted(batch, key=lambda i: int(i.can_id))

            batches.append(BatchResult(
                batch_id  = batch_id,
                trigger   = "timeout",
                fwd_time  = t_release,
                L_j       = t_release,
                instances = batch_sorted,
            ))
            batch_id += 1

        t_release += gateway_period

    return batches
