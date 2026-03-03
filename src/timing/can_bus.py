# rta.py
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from .can_classic import classic_tx_time_ms
from .can_fd import fd_tx_time_ms

"""
    Compute RTA based off CANASTA Paper.
 """

@dataclass
class CANBusConfig:
    # CAN or CAN-FD
    bus_type: str = "CAN"
    #bus_type: str = "CAN-FD"
    tbit_ms: float = 0.004
    dtbit_ms: float = 0.0005

    # Fixed-point convergence
    eps: float = 1e-12
    max_iter: int = 10_000
    jitter_ms: float = 0.0


def compute_tx_times_ms(payload_bytes, cfg: CANBusConfig) -> List[float]:
    """
    Compute transmission times C_i (ms) from payload.
    """
    C: List[float] = []
    if cfg.bus_type.upper() == "CAN":
        for pb in payload_bytes:
            C.append(classic_tx_time_ms(int(pb), cfg.tbit_ms))
    elif cfg.bus_type.upper() == "CAN-FD":
        for pb in payload_bytes:
            C.append(fd_tx_time_ms(int(pb), cfg.tbit_ms, cfg.dtbit_ms))
    else:
        raise ValueError(f"Unknown bus_type={cfg.bus_type!r}. Use 'CAN' or 'CAN-FD'.")
    return C


def _blocking_time_ms(prio, C, k: int) -> float:
    """
    blocking:
    B_k = max { C_i | prio_i is lower than prio_k }
    """
    bk = 0.0
    pk = prio[k]
    for i in range(len(prio)):
        if prio[i] > pk:
            bk = max(bk, C[i])
    return bk


def _hp_interference_ms(prio, T, C, k, w, jitter_ms= 0.0,) -> float:
    """
    Higher-priority interference over busy length w (ms):
    sum_{i in hp(k)} ceil((w + jitter)/T_i) * C_i
    """
    s = 0.0
    pk = prio[k]
    for i in range(len(prio)):
        if prio[i] < pk:
            Ti = float(T[i])
            s += math.ceil((w + jitter_ms) / Ti) * float(C[i])
    return s


def response_time_rta_ms(prio, periods_ms, C_ms,k, cfg: CANBusConfig) -> float:
    """
    Response time for message.
    """
    Bk = _blocking_time_ms(prio, C_ms, k)

    w_prev = None
    w = Bk + float(C_ms[k])  # initial 

    for _ in range(cfg.max_iter):
        if w_prev is not None and abs(w - w_prev) <= cfg.eps:
            return w

        w_prev = w
        w = Bk + float(C_ms[k]) + _hp_interference_ms(
            prio, periods_ms, C_ms, k, w_prev, cfg.jitter_ms
        )

        if not math.isfinite(w):
            raise RuntimeError(
                "CAN RTA diverged to infinity."
                "This means the set is unschedulable "
            )

    raise RuntimeError("CAN RTA did not converge within max_iter")


def compute_response_times_ms(
    priorities: Sequence[int],
    periods_ms: Sequence[float],
    payload_bytes: Sequence[int],
    cfg: CANBusConfig,
) -> List[float]:
    """
    Compute R_i (ms) for all messages.
    """
    if not (len(priorities) == len(periods_ms) == len(payload_bytes)):
        raise ValueError("priorities, periods_ms, payload_bytes must have the same length")

    C_ms = compute_tx_times_ms(payload_bytes, cfg)

    R_ms: List[float] = []
    for k in range(len(priorities)):
        R_ms.append(response_time_rta_ms(priorities, periods_ms, C_ms, k, cfg))
    return R_ms


def apply_response_times_to_releases(releases,R_ms,) -> List[Tuple[float, int]]:
    """
    Map (release_time_ms, message_id) to gie (finish_time_ms, message_id)
    using worst-case response times.
    """
    out: List[Tuple[float, int]] = []
    for r, sid in releases:
        out.append((float(r) + float(R_ms[sid]), int(sid)))
    return out