# src/gateways/fifo_zs_ap_sim.py
#
# FIFO-ZS-AP simulation.
#
# Practical simulation version:
#
#   - one-step prediction
#   - event-driven re-evaluation
#   - no recursive forecasting
#   - no cumulative prediction
#
# Prediction compensation uses:
#
#   actual CAN delay - C_i
#
# instead of analytical WCRT.
#
# This version is intended for:
#   FIFO-ZS vs FIFO-ZS-AP E2E evaluation
#
# NOT theorem validation.
#
# All times in milliseconds (ms).

from .fifo_zs_sim import BatchResult


def simulate_fifo_zs_ap(
    instances,
    S,
    R_src,
    C,
    batch_size,
    return_prediction_log=False,
):

    instances = sorted(
        instances,
        key=lambda x: float(x.arrive_gw)
    )

    batches = []

    prediction_log = []

    # =====================================================
    # prediction state
    # =====================================================

    last_arrival = {}

    min_period = {}

    # (flow_id, predicted_inst_id)
    predicted_next = {}

    # =====================================================
    # batch state
    # =====================================================

    F_j = []

    L_j = float("inf")

    batch_id = 0

    idx = 0

    while idx < len(instances):

        inst = instances[idx]

        now = float(inst.arrive_gw)

        fid = int(inst.flow_id)

        inst_id = int(inst.inst_id)

        # =================================================
        # zero-slack trigger
        # =================================================

        if F_j and now >= L_j:

            batches.append(

                BatchResult(
                    batch_id,
                    "zero_slack",
                    L_j,
                    L_j,
                    F_j.copy(),
                )
            )

            batch_id += 1

            F_j = []

            L_j = float("inf")

            continue

        # =================================================
        # prediction error measurement
        # =================================================

        pred_key = (
            fid,
            inst_id,
        )

        if pred_key in predicted_next:

            pred = float(
                predicted_next[pred_key]
            )

            err = abs(
                pred - now
            )

            # theorem bound still uses analytical WCRT

            bound = (
                2.0 *
                (
                    float(R_src[fid]) -
                    float(C[fid])
                )
            )

            prediction_log.append({

                "flow_id":
                    fid,

                "inst_id":
                    inst_id,

                "actual_arrival":
                    now,

                "predicted_arrival":
                    pred,

                "error":
                    err,

                "bound":
                    bound,

                "ok":
                    err <= bound + 1e-9,
            })

            # remove stale prediction

            del predicted_next[pred_key]

        # =================================================
        # FIFO-ZS admission
        # =================================================

        S_i = float(
            S[fid]
        )

        L_new = min(
            L_j,
            now + S_i,
        )

        if (
            (
                now >= L_new
                or
                len(F_j) >= batch_size
            )
            and F_j
        ):

            batches.append(

                BatchResult(
                    batch_id,
                    "zero_slack",
                    L_j,
                    L_j,
                    F_j.copy(),
                )
            )

            batch_id += 1

            F_j = []

            L_j = float("inf")

            continue

        # =================================================
        # admit message
        # =================================================

        F_j.append(inst)

        L_j = L_new

        # =================================================
        # update predictor
        #
        # Practical simulation version:
        #
        #   a_tilde_i^(q+1)
        #       =
        #   a_i^q
        #   + T_tilde_i
        #   + actual_src_delay
        #   - C_i
        # =================================================

        if fid in last_arrival:

            T_q = (
                now -
                float(last_arrival[fid])
            )

            min_period[fid] = min(
                min_period.get(fid, T_q),
                T_q,
            )

            next_inst_id = inst_id + 1

            actual_src_delay = float(
                inst.can_delay
            )

            predicted_next[
                (
                    fid,
                    next_inst_id,
                )
            ] = (
                now
                +
                min_period[fid]
                +
                (
                    actual_src_delay
                    -
                    float(C[fid])
                )
            )

        last_arrival[fid] = now

        # =================================================
        # buffer-full trigger
        # =================================================

        if len(F_j) >= batch_size:

            batches.append(

                BatchResult(
                    batch_id,
                    "buffer_full",
                    now,
                    L_j,
                    F_j.copy(),
                )
            )

            batch_id += 1

            F_j = []

            L_j = float("inf")

            idx += 1

            continue

        # =================================================
        # AP prediction trigger
        #
        # p = arg min_i a_tilde_i
        #
        # if a_tilde_p >= L_j:
        #     transmit immediately
        #
        # else:
        #     wait for next arrival
        #
        # Re-evaluated at every REAL arrival.
        # =================================================

        finite_predictions = [

            p

            for p in predicted_next.values()

            if p != float("inf")
        ]

        if F_j and finite_predictions:

            earliest_prediction = min(
                finite_predictions
            )

            # no predicted arrival before L_j

            if earliest_prediction >= L_j:

                batches.append(

                    BatchResult(
                        batch_id,
                        "prediction",
                        now,
                        L_j,
                        F_j.copy(),
                    )
                )

                batch_id += 1

                F_j = []

                L_j = float("inf")

        idx += 1

    # =====================================================
    # flush final batch
    # =====================================================

    if F_j:

        batches.append(

            BatchResult(
                batch_id,
                "zero_slack",
                L_j,
                L_j,
                F_j.copy(),
            )
        )

    if return_prediction_log:

        return batches, prediction_log

    return batches


def print_prediction_error_log(
    prediction_log,
    msgset=None,
    n_show=30,
):

    print(
        "\nFIFO-ZS-AP Prediction Error Validation"
        "======================="
    )

    print(
        "  |a_tilde_i^q - a_i^q| "
        "<= 2*(R_i^src - C_i)"
    )

    if not prediction_log:

        print(
            "  No predictions recorded yet."
        )

        return

    print(
        f"\n  {'msg':>5}  "
        f"{'inst':>5}  "
        f"{'pred':>10}  "
        f"{'actual':>10}  "
        f"{'error':>10}  "
        f"{'bound':>10}  "
        f"{'ok':>4}"
    )

    print("  " + "-" * 65)

    for row in prediction_log[:n_show]:

        name = (
            msgset[row["flow_id"]].msg_id
            if msgset else
            str(row["flow_id"])
        )

        ok = "✓" if row["ok"] else "✗"

        print(
            f"  {name:>5}  "
            f"{row['inst_id']:>5}  "
            f"{row['predicted_arrival']:>10.4f}  "
            f"{row['actual_arrival']:>10.4f}  "
            f"{row['error']:>10.4f}  "
            f"{row['bound']:>10.4f}  "
            f"{ok:>4}"
        )

    max_err = max(
        r["error"]
        for r in prediction_log
    )

    max_bound = max(
        r["bound"]
        for r in prediction_log
    )

    all_ok = all(
        r["ok"]
        for r in prediction_log
    )

    print(
        f"\n  predictions : {len(prediction_log)}"
    )

    print(
        f"  max error   : {max_err:.4f} ms"
    )

    print(
        f"  max bound   : {max_bound:.4f} ms"
    )

    print(
        f"  bound holds : "
        f"{'✓' if all_ok else '✗'}"
    )