# src/gateways/fifo_timeout.py
# FIFO-Timeout analytical encapsulation delay.
# All times in milliseconds (ms).
#
# Reference: Berisa et al.
#
# mu_FIFO(q) = floor( sum_{i in fwd(q)} floor(T_TSN / T_i) / beta )
#
# D_FIFO(q)  = (mu_FIFO(q) + 1) * T_TSN
#
# The gateway fires every T_TSN ms regardless of buffer fill.
# mu_FIFO counts how many TSN frames must transmit before the
# worst-case last queued CAN frame gets encapsulated.

import math


def compute_T_tsn(msgset, beta):
    """
    Compute the TSN frame period from the message set and batch size.

    Equation (1):
        T_TSN(q) = floor( beta / sum_{i in fwd(q)} 1/T_i )

    Parameters
    ----------
    msgset : list[CANMessage]
    beta   : int  — batch size (number of CAN frames per TSN frame)

    Returns
    -------
    float  — TSN frame period in ms
    """
    rate_sum = sum(1.0 / float(m.period) for m in msgset)
    return math.floor(float(beta) / rate_sum)


def mu_fifo(msgset, T_tsn, beta):
    """
    mu_FIFO(q) = floor( sum_i floor(T_TSN / T_i) / beta )

    Parameters
    ----------
    msgset : list[CANMessage]  — forwarded messages
    T_tsn  : float             — TSN frame period (ms) = GCL cycle
    beta   : int               — batch size

    Returns
    -------
    int
    """
    total = sum(math.floor(float(T_tsn) / float(m.period)) for m in msgset)
    return math.floor(total / int(beta))


def fifo_timeout_enc_delay(msgset, T_tsn, beta):
    """
    D_FIFO(q) = (mu_FIFO(q) + 1) * T_TSN

    Worst-case forwarding delay for any CAN frame under FIFO-timeout.

    Parameters
    ----------
    msgset : list[CANMessage]
    T_tsn  : float  — TSN period / GCL cycle (ms)
    beta   : int    — batch size

    Returns
    -------
    float  — worst-case encapsulation delay (ms)
    """
    return (mu_fifo(msgset, T_tsn, beta) + 1) * float(T_tsn)


class FIFOTimeoutGateway:
    """
    Analytical FIFO-Timeout gateway.

    Fires every T_tsn ms (GCL cycle) regardless of buffer fill.

    Parameters
    ----------
    msgset    : forwarded CAN messages
    R         : worst-case source CAN response times (ms)
    C         : CAN transmission times (ms)
    T_tsn     : TSN frame period / GCL cycle (ms)
    beta      : batch size
    tsn_wcrt  : TSN WCRT (ms)
    c_can_max : max destination CAN tx time (ms)
    """

    def __init__(self, msgset, R, C, T_tsn, beta, tsn_wcrt, c_can_max):
        self.msgset   = list(msgset)
        self.R        = list(R)
        self.C        = list(C)
        self.T_tsn    = float(T_tsn)
        self.beta     = int(beta)
        self.mu       = mu_fifo(msgset, T_tsn, beta)
        self.delta_enc = fifo_timeout_enc_delay(msgset, T_tsn, beta)
        self.tsn_wcrt  = float(tsn_wcrt)
        self.delta_dec = (beta - 1) * float(c_can_max)

    @property
    def gateway_wcrt(self):
        return self.delta_enc + self.tsn_wcrt + self.delta_dec

    def e2e(self, r_src, c, r_dst=0.0):
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
                "C":         c,
                "R_dst":     rd,
                "mu":        self.mu,
                "T_tsn":     self.T_tsn,
                "delta_enc": self.delta_enc,
                "tsn_wcrt":  self.tsn_wcrt,
                "delta_dec": self.delta_dec,
                "e2e":       self.e2e(r, c, rd),
                "feasible":  self.feasible(m, r, c, rd),
            }
            for m, r, c, rd in zip(self.msgset, self.R, self.C, R_dst)
        ]