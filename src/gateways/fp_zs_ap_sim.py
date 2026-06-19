# src/gateways/fifo_fp_zs_ap_sim.py
#
# Fixed-Priority Zero-Slack AP simulation.
# Admission and AP prediction trigger identical to FIFO-ZS-AP.

from .fifo_zs_sim import BatchResult


def simulate_fifo_fp_zs_ap(
    instances,
    S,
    R_src,
    C,
    batch_size,
    return_prediction_log=False,
):
    instances = sorted(instances, key=lambda x: float(x.arrive_gw))

    batches        = []
    prediction_log = []
    last_arrival   = {}
    min_period     = {}
    predicted_next = {}   # (flow_id, inst_id) -> predicted arrival

    F_j      = []
    L_j      = float("inf")
    batch_id = 0
    idx      = 0

    def _fire(trigger, fwd_time, L_j_val, F_j_local):
        # sort by priority before forwarding
        sorted_batch = sorted(F_j_local, key=lambda i: int(i.can_id))
        return BatchResult(batch_id, trigger, fwd_time, L_j_val, sorted_batch)

    while idx < len(instances):
        inst    = instances[idx]
        now     = float(inst.arrive_gw)
        fid     = int(inst.flow_id)
        inst_id = int(inst.inst_id)

        # zero-slack trigger
        if F_j and now >= L_j:
            batches.append(_fire("zero_slack", L_j, L_j, F_j))
            batch_id += 1; F_j = []; L_j = float("inf")
            continue

        # prediction error measurement
        pred_key = (fid, inst_id)
        if pred_key in predicted_next:
            pred  = float(predicted_next[pred_key])
            err   = abs(pred - now)
            bound = 2.0 * (float(R_src[fid]) - float(C[fid]))
            prediction_log.append({
                "flow_id": fid, "inst_id": inst_id,
                "actual_arrival": now, "predicted_arrival": pred,
                "error": err, "bound": bound, "ok": err <= bound + 1e-9,
            })
            del predicted_next[pred_key]

        # FIFO-ZS admission
        S_i   = float(S[fid])
        L_new = min(L_j, now + S_i)

        if (now >= L_new or len(F_j) >= batch_size) and F_j:
            batches.append(_fire("zero_slack", L_j, L_j, F_j))
            batch_id += 1; F_j = []; L_j = float("inf")
            continue

        F_j.append(inst); L_j = L_new

        # update predictor
        if fid in last_arrival:
            T_q = now - float(last_arrival[fid])
            min_period[fid] = min(min_period.get(fid, T_q), T_q)
            actual_src_delay = float(inst.can_delay)
            predicted_next[(fid, inst_id + 1)] = (
                now + min_period[fid] + actual_src_delay - float(C[fid]))
        last_arrival[fid] = now

        # buffer-full trigger
        if len(F_j) >= batch_size:
            batches.append(_fire("buffer_full", now, L_j, F_j))
            batch_id += 1; F_j = []; L_j = float("inf")
            idx += 1; continue

        # AP prediction trigger
        finite = [p for p in predicted_next.values() if p != float("inf")]
        if F_j and finite and min(finite) >= L_j:
            batches.append(_fire("prediction", now, L_j, F_j))
            batch_id += 1; F_j = []; L_j = float("inf")

        idx += 1

    if F_j:
        batches.append(_fire("zero_slack", L_j, L_j, F_j))

    if return_prediction_log:
        return batches, prediction_log
    return batches
