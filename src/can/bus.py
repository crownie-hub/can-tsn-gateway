# src/can/bus.py
# CAN bus timing and response-time analysis.
# All times in milliseconds (ms).

import math
from dataclasses import dataclass

from .classic import classic_tx_time
from .fd      import fd_tx_time


@dataclass
class CANBusConfig:
    bus_type: str   = "CAN"
    tbit:     float = 0.004
    dtbit:    float = 0.0005
    jitter:   float = 0.0
    eps:      float = 1e-12
    max_iter: int   = 10_000


def compute_tx_times(payload_bytes, cfg):
    if cfg.bus_type.upper() == "CAN":
        return [classic_tx_time(p, cfg.tbit) for p in payload_bytes]
    if cfg.bus_type.upper() == "CAN-FD":
        return [fd_tx_time(p, cfg.tbit, cfg.dtbit) for p in payload_bytes]
    raise ValueError(f"Unknown bus_type {cfg.bus_type!r}")


def _blocking(priorities, C, k):
    pk = priorities[k]
    return max((C[i] for i in range(len(priorities)) if priorities[i] > pk), default=0.0)


def _hp_interference(priorities, periods, C, k, w, jitter=0.0):
    pk = priorities[k]
    return sum(
        math.ceil((w + jitter) / float(periods[i])) * float(C[i])
        for i in range(len(priorities))
        if priorities[i] < pk
    )


def response_time(priorities, periods, C, k, cfg):
    B = _blocking(priorities, C, k)
    w = B + float(C[k])
    for _ in range(cfg.max_iter):
        w_new = B + float(C[k]) + _hp_interference(
            priorities, periods, C, k, w, cfg.jitter)
        if not math.isfinite(w_new):
            raise RuntimeError(f"CAN RTA diverged for k={k}")
        if abs(w_new - w) <= cfg.eps:
            return w_new
        w = w_new
    raise RuntimeError(f"CAN RTA did not converge for k={k}")


def compute_response_times(priorities, periods, payload_bytes, cfg):
    C = compute_tx_times(payload_bytes, cfg)
    return [response_time(priorities, periods, C, k, cfg)
            for k in range(len(priorities))]
