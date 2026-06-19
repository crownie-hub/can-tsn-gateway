# src/gateways/fifo_zs_ap.py
#
# FIFO Zero-Slack with Arrival Prediction (FIFO-ZS-AP)
#
# Analytical bound derived directly from theorem:
#
# 0 <= delta_zs - delta_zs_ap
#    <= 2 * max_i(R_i^src - C_i)


from .fifo_zs import fifo_zs_enc_delay


def prediction_error_bound(R, C):
    """
    Maximum analytical prediction improvement bound.

    gain =
        2 * max_i(R_i^src - C_i)
    """
    return 2.0 * max(
        r - c
        for r, c in zip(R, C)
    )


class FIFOZeroSlackAPGateway:
    """
    Analytical FIFO-ZS-AP gateway model.

    The theorem only provides an interval bound:

        delta_lower <= delta_zs_ap <= delta_upper

    where:

        delta_upper = FIFO-ZS delay
        delta_lower = FIFO-ZS delay - gain
    """

    def __init__(
        self,
        msgset,
        R,
        C,
        S,
        n,
        tsn_wcrt,
        c_can_max,
    ):

        self.msgset = list(msgset)

        self.R = list(R)
        self.C = list(C)
        self.S = list(S)

        self.n = int(n)

        #
        # FIFO-ZS baseline
        #
        self.delta_zs = fifo_zs_enc_delay(
            msgset,
            R,
            C,
            S,
            n,
        )

        #
        # Prediction improvement bound
        #
        self.prediction_gain = prediction_error_bound(
            R,
            C,
        )

        #
        # Theorem interval
        #
        self.delta_enc_upper = self.delta_zs

        self.delta_enc_lower = max(
            0.0,
            self.delta_zs - self.prediction_gain
        )

        #
        # TSN delay
        #
        self.tsn_wcrt = float(tsn_wcrt)

        #
        # Decapsulation delay
        #
        self.delta_dec = (
            (n - 1) * float(c_can_max)
        )

    @property
    def gateway_wcrt_upper(self):
        """
        Worst-case gateway delay upper bound.
        """
        return (
            self.delta_enc_upper
            + self.tsn_wcrt
            + self.delta_dec
        )

    @property
    def gateway_wcrt_lower(self):
        """
        Best analytical gateway delay
        allowed by theorem.
        """
        return (
            self.delta_enc_lower
            + self.tsn_wcrt
            + self.delta_dec
        )

    def e2e_upper(
        self,
        r_src,
        c,
        r_dst=0.0,
    ):
        """
        Conservative E2E bound.

        Uses FIFO-ZS delay directly.
        """

        return (
            float(r_src)
            + self.gateway_wcrt_upper
            + float(r_dst)
            - float(c)
        )

    def e2e_lower(
        self,
        r_src,
        c,
        r_dst=0.0,
    ):
        """
        Best analytical E2E bound
        implied by theorem.
        """

        return (
            float(r_src)
            + self.gateway_wcrt_lower
            + float(r_dst)
            - float(c)
        )

    def feasible_upper(
        self,
        msg,
        r_src,
        c,
        r_dst=0.0,
    ):
        """
        Conservative schedulability.
        """

        return (
            self.e2e_upper(
                r_src,
                c,
                r_dst,
            )
            <= float(msg.deadline)
        )

    def feasible_lower(
        self,
        msg,
        r_src,
        c,
        r_dst=0.0,
    ):
        """
        Best-case schedulability
        allowed by theorem.
        """

        return (
            self.e2e_lower(
                r_src,
                c,
                r_dst,
            )
            <= float(msg.deadline)
        )

    def results(self, R_dst=None):

        if R_dst is None:
            R_dst = [0.0] * len(self.msgset)

        rows = []

        for m, r, c, s, rd in zip(
            self.msgset,
            self.R,
            self.C,
            self.S,
            R_dst,
        ):

            rows.append({

                "msg_id": m.msg_id,
                "period": m.period,
                "deadline": m.deadline,

                "R_src": r,
                "C": c,
                "R_dst": rd,

                "slack": s,

                #
                # FIFO-ZS baseline
                #
                "delta_zs": self.delta_zs,

                #
                # AP theorem terms
                #
                "prediction_gain":
                    self.prediction_gain,

                "delta_enc_lower":
                    self.delta_enc_lower,

                "delta_enc_upper":
                    self.delta_enc_upper,

                #
                # TSN
                #
                "tsn_wcrt":
                    self.tsn_wcrt,

                #
                # Decapsulation
                #
                "delta_dec":
                    self.delta_dec,

                #
                # Gateway interval
                #
                "gateway_wcrt_lower":
                    self.gateway_wcrt_lower,

                "gateway_wcrt_upper":
                    self.gateway_wcrt_upper,

                #
                # E2E interval
                #
                "e2e_lower":
                    self.e2e_lower(
                        r,
                        c,
                        rd,
                    ),

                "e2e_upper":
                    self.e2e_upper(
                        r,
                        c,
                        rd,
                    ),

                #
                # Feasibility
                #
                "feasible_lower":
                    self.feasible_lower(
                        m,
                        r,
                        c,
                        rd,
                    ),

                "feasible_upper":
                    self.feasible_upper(
                        m,
                        r,
                        c,
                        rd,
                    ),
            })

        return rows