# src/gateways/evaluation.py
# Evaluation — enc delay, e2e, prediction error.


def compute_max_enc_delay(batches):
    """
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
