# src/gateway/fifo_zs.py
# FIFO Zero-Slack analytical encapsulation delay.
#
# S_i       = D_i - (R_i^src + R_j^TSN + delta_dec + R_i^dst - C_i)
# tau_BF    = min{ tau >= 0 | A(tau) >= n }
# tau_ZS    = max(0, min_i S_i)
# delta_enc = min(tau_BF, tau_ZS)

from .fifo_bf import fifo_bf_enc_delay


def compute_slacks(msgset, R_src, C, tsn_wcrt, delta_dec, R_dst):
    """
    S_i = D_i - (R_i^src + R_j^TSN + delta_dec + R_i^dst - C_i)

    S_i is the time budget available for encapsulation.
    delta_enc is NOT included — that is what S_i is budgeting for.
    -C_i excludes the minimum CAN delay since the actual source delay
    is unknown at the gateway.
    """
    return [
        m.deadline - (r_src + tsn_wcrt + delta_dec + r_dst - c)
        for m, r_src, c, r_dst in zip(msgset, R_src, C, R_dst)
    ]


def fifo_zs_enc_delay(msgset, R, C, S, n):
    tau_bf = fifo_bf_enc_delay(msgset, R, C, n)
    tau_zs = max(0.0, min(S))
    return min(tau_bf, tau_zs)


class FIFOZeroSlackGateway:
    def __init__(self, msgset, R, C, S, n, tsn_wcrt, c_can_max):
        self.msgset    = list(msgset)
        self.R         = list(R)
        self.C         = list(C)
        self.S         = list(S)
        self.n         = n
        self.tau_bf    = fifo_bf_enc_delay(msgset, R, C, n)
        self.tau_zs    = max(0.0, min(S))
        self.delta_enc = min(self.tau_bf, self.tau_zs)
        self.tsn_wcrt  = float(tsn_wcrt)
        self.delta_dec = (n - 1) * float(c_can_max)

    @property
    def gateway_wcrt(self):
        return self.delta_enc + self.tsn_wcrt + self.delta_dec

    @property
    def triggered_by(self):
        return "zero_slack" if self.tau_zs <= self.tau_bf else "buffer_full"

    def e2e(self, r_src, c, r_dst=0.0):
        # e2e - C_i: consistent with slack definition S_i = d_i - (R_i^e2e - C_i)
        return float(r_src) + self.gateway_wcrt + float(r_dst) - float(c)

    def feasible(self, msg, r_src, c, r_dst=0.0):
        return self.e2e(r_src, c, r_dst) <= float(msg.deadline)

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
                "slack":     s,
                "C":         c,
                "e2e":       self.e2e(r, c, rd),
                "feasible":  self.feasible(m, r, c, rd),
            }
            for m, r, c, s, rd in zip(self.msgset, self.R, self.C, self.S, R_dst)
        ]