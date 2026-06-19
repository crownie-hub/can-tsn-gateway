# src/gateways/fifo_fp_zs_sim.py
#
# Fixed-Priority Zero-Slack simulation.
#

from .fifo_zs_sim import BatchResult


def simulate_fifo_fp_zs(instances, S, batch_size):
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))

    batches  = []
    F_j      = []
    L_j      = float("inf")
    batch_id = 0
    idx      = 0

    while idx < len(instances):
        inst    = instances[idx]
        now     = float(inst.arrive_gw)
        fid     = int(inst.flow_id)

        # zero-slack trigger
        if F_j and now >= L_j:
            F_j_sorted = sorted(F_j, key=lambda i: int(i.can_id))
            batches.append(BatchResult(
                batch_id  = batch_id,
                trigger   = "zero_slack",
                fwd_time  = L_j,
                L_j       = L_j,
                instances = F_j_sorted,
            ))
            batch_id += 1
            F_j = []
            L_j = float("inf")
            continue

        # admission
        S_i   = float(S[fid])
        L_new = min(L_j, now + S_i)

        if (now >= L_new or len(F_j) >= batch_size) and F_j:
            F_j_sorted = sorted(F_j, key=lambda i: int(i.can_id))
            batches.append(BatchResult(
                batch_id  = batch_id,
                trigger   = "zero_slack",
                fwd_time  = L_j,
                L_j       = L_j,
                instances = F_j_sorted,
            ))
            batch_id += 1
            F_j = []
            L_j = float("inf")
            continue

        F_j.append(inst)
        L_j = L_new

        if len(F_j) >= batch_size:
            F_j_sorted = sorted(F_j, key=lambda i: int(i.can_id))
            batches.append(BatchResult(
                batch_id  = batch_id,
                trigger   = "buffer_full",
                fwd_time  = now,
                L_j       = L_j,
                instances = F_j_sorted,
            ))
            batch_id += 1
            F_j = []
            L_j = float("inf")

        idx += 1

    if F_j:
        F_j_sorted = sorted(F_j, key=lambda i: int(i.can_id))
        batches.append(BatchResult(
            batch_id  = batch_id,
            trigger   = "zero_slack",
            fwd_time  = L_j,
            L_j       = L_j,
            instances = F_j_sorted,
        ))

    return batches
