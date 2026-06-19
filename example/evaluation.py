# src/gateways/evaluation.py
# Evaluation utilities — enc delay, e2e, prediction error.


def compute_max_enc_delay(batches):
    """
    Worst-case enc_delay per flow_id across all batches.

    enc_delay_i = fwd_time - arrive_gw  (per instance)
    """
    per_flow = {}
    for b in batches:
        for inst in b.instances:
            enc = b.fwd_time - float(inst.arrive_gw)
            fid = inst.flow_id
            per_flow[fid] = max(per_flow.get(fid, 0.0), enc)
    return per_flow


def compute_e2e(msgset, R_src_actual, C, R_dst, enc_delays,
                tsn_wcrt, delta_dec):
    """
    Simulated worst-case e2e per message.

        e2e = R_src_actual + enc + tsn_wcrt + delta_dec + R_dst - C

    """
    rows = []
    for i, m in enumerate(msgset):
        enc = enc_delays.get(i, 0.0)
        e2e = (float(R_src_actual[i]) + enc + float(tsn_wcrt)
               + float(delta_dec) + float(R_dst[i]) - float(C[i]))
        rows.append({
            "msg_id":   m.msg_id,
            "R_src":    R_src_actual[i],
            "enc":      enc,
            "tsn":      tsn_wcrt,
            "dec":      delta_dec,
            "R_dst":    R_dst[i],
            "C":        C[i],
            "e2e":      e2e,
            "deadline": m.deadline,
            "feasible": e2e <= m.deadline + 1e-9,
        })
    return rows


def compute_prediction_errors(batches_ap, predicted_arrivals):
    """
    Measure prediction error |a_tilde - a_actual| per instance.

    """
    errors = []
    for b in batches_ap:
        for inst in b.instances:
            key = (inst.flow_id, inst.inst_id)
            if key in predicted_arrivals:
                pred   = predicted_arrivals[key]
                actual = float(inst.arrive_gw)
                errors.append({
                    "flow_id":  inst.flow_id,
                    "inst_id":  inst.inst_id,
                    "predicted": pred,
                    "actual":    actual,
                    "error":     abs(pred - actual),
                })
    return errors


def verify_theorem_bound(enc_delays_zs, enc_delays_ap, R_src, C):
    """
    Verify:
        0 <= delta_zs - delta_zs_ap <= 2 * max_i(R_i - C_i)

    Returns dict with improvement per flow and bound check.
    """
    gain  = 2.0 * max(r - c for r, c in zip(R_src, C))
    rows  = []
    max_improvement = 0.0

    for fid in enc_delays_zs:
        d_zs = enc_delays_zs.get(fid, 0.0)
        d_ap = enc_delays_ap.get(fid, 0.0)
        improvement = d_zs - d_ap
        max_improvement = max(max_improvement, improvement)
        rows.append({
            "flow_id":     fid,
            "delta_zs":    d_zs,
            "delta_ap":    d_ap,
            "improvement": improvement,
            "within_bound": improvement <= gain + 1e-9,
        })

    return {
        "gain":            gain,
        "max_improvement": max_improvement,
        "bound_holds":     max_improvement <= gain + 1e-9,
        "rows":            rows,
    }


def compute_mat_positions(msgset, tsn_wcrt, R_dst):
    """
    Compute analytical MAT position for each message.

    MAT(m_i) = D_i - tsn_wcrt - R_dst_i
    """
    mat_vals = [
        float(m.deadline) - float(tsn_wcrt) - float(R_dst[i])
        for i, m in enumerate(msgset)
    ]
    # rank: position of each message in MAT-sorted order
    order     = sorted(range(len(msgset)), key=lambda i: mat_vals[i])
    positions = [0] * len(msgset)
    for rank, idx in enumerate(order):
        positions[idx] = rank
    return positions


def compute_e2e_mat(msgset, R_src, C, R_dst, batches,
                    tsn_wcrt, c_can_max):
    """
    E2E for FIFO-BF + MAT-DST using per-batch MAT positions.

    For each batch, MAT sorts messages by urgency:
        MAT(m_i) = D_i - tsn_wcrt - R_dst_i

    Message i gets worst-case position across all batches it appears in.
    delta_dec_i = worst_position_i * C_max

    No double-counting — position from MAT analysis, not simulation.
    """
    mat_vals = [
        float(m.deadline) - float(tsn_wcrt) - float(R_dst[i])
        for i, m in enumerate(msgset)
    ]

    # worst MAT position per flow across all batches
    worst_pos = {}

    for b in batches:
        # sort instances in this batch by MAT
        sorted_insts = sorted(b.instances,
                              key=lambda inst: mat_vals[int(inst.flow_id)])
        for rank, inst in enumerate(sorted_insts):
            fid = int(inst.flow_id)
            worst_pos[fid] = max(worst_pos.get(fid, 0), rank)

    rows = []
    for i, m in enumerate(msgset):
        pos       = worst_pos.get(i, 0)
        delta_dec = pos * float(c_can_max)
        # enc delay: worst case across batches message i appears in
        enc = max(
            (b.fwd_time - float(inst.arrive_gw))
            for b in batches
            for inst in b.instances
            if int(inst.flow_id) == i
        ) if any(int(inst.flow_id) == i
                 for b in batches for inst in b.instances) else 0.0

        e2e = (float(R_src[i]) + enc + float(tsn_wcrt)
               + delta_dec + float(R_dst[i]) - float(C[i]))
        rows.append({
            "msg_id":    m.msg_id,
            "R_src":     R_src[i],
            "enc":       enc,
            "tsn":       tsn_wcrt,
            "delta_dec": delta_dec,
            "position":  pos,
            "R_dst":     R_dst[i],
            "C":         C[i],
            "e2e":       e2e,
            "deadline":  m.deadline,
            "feasible":  e2e <= m.deadline + 1e-9,
        })
    return rows