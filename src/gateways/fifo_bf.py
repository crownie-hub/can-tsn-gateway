# src/gateway/fifo_bf.py
# FIFO Buffer-Full analytical encapsulation delay.
#
# A(tau) = 1 + sum_{m_x} max(0, floor((tau - (R_x - C_x)) / T_x))
# delta_enc = min{ tau >= 0 | A(tau) >= n }

import math


def arrival_function(tau, msgset, R, C):
    total = 0
    for m, r, c in zip(msgset, R, C):
        total += max(0, math.floor((tau - (r - c)) / m.period))
    return 1 + total


def _min_tau(msgset, R, C, target):
    if arrival_function(0.0, msgset, R, C) >= target:
        return 0.0

    T_max   = max(m.period for m in msgset)
    max_tau = (target - 1) * T_max + max(r - c for r, c in zip(R, C))

    taus = {0.0}
    for m, r, c in zip(msgset, R, C):
        k = 1
        while k * m.period + (r - c) <= max_tau + m.period:
            taus.add(k * m.period + (r - c))
            k += 1

    for tau in sorted(taus):
        if arrival_function(tau, msgset, R, C) >= target:
            return tau

    raise RuntimeError(f"_min_tau: no solution for target={target}")


def fifo_bf_enc_delay(msgset, R, C, n):
    if n == 1:
        return 0.0
    return _min_tau(msgset, R, C, n)


class FIFOBatchGateway:
    def __init__(self, msgset, R, C, n, tsn_wcrt, c_can_max):
        self.msgset    = list(msgset)
        self.R         = list(R)
        self.C         = list(C)
        self.n         = n
        self.delta_enc = fifo_bf_enc_delay(msgset, R, C, n)
        self.tsn_wcrt  = float(tsn_wcrt)
        self.delta_dec = (n - 1) * float(c_can_max)

    @property
    def gateway_wcrt(self):
        return self.delta_enc + self.tsn_wcrt + self.delta_dec

    def e2e(self, r_src, r_dst=0.0):
        return float(r_src) + self.gateway_wcrt + float(r_dst)

    def feasible(self, msg, r_src, r_dst=0.0):
        return self.e2e(r_src, r_dst) <= float(msg.deadline)

    def results(self, R_dst=None):
        if R_dst is None:
            R_dst = [0.0] * len(self.msgset)
        return [
            {
                "msg_id":    m.msg_id,
                "period":    m.period,
                "deadline":  m.deadline,
                "R_src":     r,
                "delta_enc": self.delta_enc,
                "tsn_wcrt":  self.tsn_wcrt,
                "delta_dec": self.delta_dec,
                "R_dst":     rd,
                "e2e":       self.e2e(r, rd),
                "feasible":  self.feasible(m, r, rd),
            }
            for m, r, rd in zip(self.msgset, self.R, R_dst)
        ]