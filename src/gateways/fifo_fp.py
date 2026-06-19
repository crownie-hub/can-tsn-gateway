# src/gateways/fifo_fp.py
#
# Fixed-Priority analytical gateway classes.
#


import math


def _compute_A(tau_ms, msgset, periods, C_src):
    """
    Total CAN frame arrivals in window tau_ms (all messages).
    A(tau) = sum_i floor(tau / T_i)
    """
    return sum(math.floor(tau_ms / T) for T in periods)


def _compute_A_H(tau_ms, hp_periods):
    """
    A_H(tau) = sum_{h in HP} floor(tau / T_h)
    """
    return sum(math.floor(tau_ms / T) for T in hp_periods)


def _min_tau_for_threshold(threshold, periods, C_src):
    """
    Find minimum tau such that A(tau) >= threshold.
    Used binary search over arrival times.
    """
    if threshold <= 0:
        return 0.0

    # upper bound: all messages at their period
    tau = max(periods) * threshold
    for _ in range(200):
        a = _compute_A(tau, None, periods, C_src)
        if a >= threshold:
            # refine downward
            tau_lo = 0.0
            tau_hi = tau
            for _ in range(50):
                mid = (tau_lo + tau_hi) / 2.0
                if _compute_A(mid, None, periods, C_src) >= threshold:
                    tau_hi = mid
                else:
                    tau_lo = mid
            return tau_hi
        tau *= 2.0
    return tau


class FPBatchGateway:
   

    def __init__(self, msgset, R_src, C_src, batch_size, tsn_wcrt, c_can_max):
        self.msgset     = msgset
        self.R_src      = list(R_src)
        self.C_src      = list(C_src)
        self.n          = int(batch_size)
        self.tsn_wcrt   = float(tsn_wcrt)
        self.c_can_max  = float(c_can_max)

        self.periods    = [float(m.period) for m in msgset]
        self.delta_dec  = (self.n - 1) * self.c_can_max

        # compute per-message enc delay
        self._enc = [self._fp_bf_enc(i) for i in range(len(msgset))]
        self.delta_enc = max(self._enc)

    def _fp_bf_enc(self, idx):
        """FP-BF enc delay for message at index idx."""
        # higher-priority messages: lower priority number
        my_prio   = self.msgset[idx].priority
        hp_idx    = [j for j, m in enumerate(self.msgset)
                     if m.priority < my_prio]
        hp_periods = [self.periods[j] for j in hp_idx]

        # |M_H| = HP messages already in buffer when m_i arrives
        # conservatively = number of HP messages = len(hp_idx)
        M_H = len(hp_idx)

        # fixed-point iteration
        N_H = math.floor(M_H / self.n)

        for _ in range(100):
            threshold = (N_H + 1) * self.n
            tau       = _min_tau_for_threshold(
                threshold, self.periods, self.C_src)

            A_H       = _compute_A_H(tau, hp_periods)
            N_H_new   = math.floor((M_H + A_H) / self.n)

            if N_H_new == N_H:
                return tau
            N_H = N_H_new

        return tau

    def enc_delay(self, idx):
        return self._enc[idx]

    def wcrt(self, idx):
        return (self.R_src[idx] + self._enc[idx]
                + self.tsn_wcrt + self.delta_dec
                + self.R_src[idx] - self.C_src[idx])

    def summary(self):
        return {
            "delta_enc": self.delta_enc,
            "delta_dec": self.delta_dec,
            "per_msg_enc": self._enc,
        }


class FPZeroSlackGateway:


    def __init__(self, msgset, R_src, C_src, S, batch_size,
                 tsn_wcrt, c_can_max):
        from .fifo_zs import FIFOZeroSlackGateway
        _gw = FIFOZeroSlackGateway(
            msgset, R_src, C_src, S, batch_size, tsn_wcrt, c_can_max)

        self.delta_enc       = _gw.delta_enc
        self.delta_dec       = _gw.delta_dec
        self.tau_bf          = _gw.tau_bf
        self.tau_zs          = _gw.tau_zs
        self.delta_enc_lower = getattr(_gw, 'delta_enc_lower', 0.0)
        self.delta_enc_upper = getattr(_gw, 'delta_enc_upper', _gw.delta_enc)
        self.tsn_wcrt        = float(tsn_wcrt)
        self.R_src           = list(R_src)
        self.C_src           = list(C_src)
        self.c_can_max       = float(c_can_max)
        self.n               = int(batch_size)


def fp_delta_dec(msgset, batches, c_can_max):
    """
    Per-message worst-case decapsulation delay under FP ordering.

    In each batch messages are sorted by can_id ascending
    (highest CAN priority first, position 0).

        delta_dec_i = worst_position_i * C_max
    """
    worst_pos = {}
    for b in batches:
        sorted_insts = sorted(b.instances, key=lambda i: int(i.can_id))
        for rank, inst in enumerate(sorted_insts):
            fid = int(inst.flow_id)
            worst_pos[fid] = max(worst_pos.get(fid, 0), rank)
    return [worst_pos.get(i, 0) * float(c_can_max)
            for i in range(len(msgset))]
