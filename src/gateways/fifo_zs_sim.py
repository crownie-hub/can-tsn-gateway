# src/gateways/fifo_zs_sim.py
# FIFO-ZS simulation using actual CAN arrival times.
# Transmit when:
#   1. zero-slack: current time >= L_j = min_x(a_x + S_x)
#   2. buffer-full: |F_j| == batch_size

from dataclasses import dataclass


@dataclass
class BatchResult:
    batch_id:  int
    trigger:   str
    fwd_time: float
    L_j:       float
    instances: list


def simulate_fifo_zs(instances, S, batch_size):
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))
    batches   = []
    F_j       = []
    L_j       = float("inf")
    batch_id  = 0
    idx       = 0

    while idx < len(instances):
        inst = instances[idx]
        now  = float(inst.arrive_gw)
        S_i  = float(S[inst.flow_id])

        # zero-slack fires before next arrival
        if F_j and now >= L_j:
            batches.append(BatchResult(batch_id, "zero_slack", L_j, L_j, F_j.copy()))
            batch_id += 1
            F_j = []
            L_j = float("inf")
            continue

        L_new = min(L_j, now + S_i)

        # admission fails — new message drives L_new < now
        if (now >= L_new or len(F_j) >= batch_size) and F_j:
            batches.append(BatchResult(batch_id, "zero_slack", L_j, L_j, F_j.copy()))
            batch_id += 1
            F_j = []
            L_j = float("inf")
            continue

        F_j.append(inst)
        L_j = L_new

        if len(F_j) >= batch_size:
            batches.append(BatchResult(batch_id, "buffer_full", now, L_j, F_j.copy()))
            batch_id += 1
            F_j = []
            L_j = float("inf")

        idx += 1

    if F_j:
        batches.append(BatchResult(batch_id, "zero_slack", L_j, L_j, F_j.copy()))

    return batches


def compute_batch_enc_delays(batches):
    """Per-instance enc_delay records."""
    rows = []
    for b in batches:
        for inst in b.instances:
            rows.append({
                "flow_id":   int(inst.flow_id),
                "inst_id":   int(inst.inst_id),
                "arrive_gw": float(inst.arrive_gw),
                "fwd_time": float(b.fwd_time),
                "enc_delay": float(b.fwd_time) - float(inst.arrive_gw),
                "trigger":   b.trigger,
                "batch_id":  b.batch_id,
            })
    return rows


def print_zs_batch_trace(batches, S, msgset=None, n_show=10):
    print("\nFIFO-ZS Batch Trace==========================================")
    print("  L_j = min_i(a_i + S_i)  |  trigger: zero-slack or buffer-full\n")

    for b in batches[:n_show]:
        trig    = "zero-slack" if b.trigger == "zero_slack" else "buffer-full"
        max_enc = max(b.fwd_time - float(i.arrive_gw) for i in b.instances)
        print(f"  Batch {b.batch_id+1}  |F_j|={len(b.instances)}  "
              f"fwd={b.fwd_time:.4f}  L_j={b.L_j:.4f}  "
              f"enc={max_enc:.4f} ms  [{trig}]")
        print(f"    {'msg':>5}  {'arrive_gw':>10}  {'S_i':>8}  "
              f"{'a+S':>8}  {'enc_wait':>10}")
        print("    " + "-" * 50)
        for inst in b.instances:
            fid  = int(inst.flow_id)
            name = msgset[fid].msg_id if msgset else str(fid)
            s_i  = float(S[fid])
            a_i  = float(inst.arrive_gw)
            print(f"    {name:>5}  {a_i:>10.4f}  {s_i:>8.4f}  "
                  f"{a_i+s_i:>8.4f}  {b.fwd_time-a_i:>10.4f} ms")
        print()