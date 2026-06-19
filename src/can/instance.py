# src/can/instance.py
# All times in milliseconds (ms).

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence


@dataclass(frozen=True)
class CANInstance:
    # identity
    flow_id:      int
    inst_id:      int
    can_id:       int
    # timing
    release:      float
    can_delay:    float
    arrive_gw:    float
    # message params
    period:       float
    deadline:     float
    payload_size: int
    # optional RTA breakdown
    can_blocking: Optional[float] = None
    can_interf:   Optional[float] = None


def build_instances(msgset, R, horizon, phases=None):
    horizon = float(horizon)
    if phases is None:
        phases = [0.0] * len(msgset)

    instances = []
    for i, msg in enumerate(msgset):
        Ri = float(R[i])
        ph = float(phases[i])
        j  = 0
        while True:
            release = ph + j * float(msg.period)
            if release > horizon:
                break
            instances.append(CANInstance(
                flow_id=i, inst_id=j,
                can_id=int(msg.msg_id, 16),
                release=release,
                can_delay=Ri,
                arrive_gw=release + Ri,
                period=float(msg.period),
                deadline=float(msg.deadline),
                payload_size=int(msg.payload_size),
            ))
            j += 1

    instances.sort(key=lambda x: x.arrive_gw)
    return instances


def build_instances_full_batches(msgset, R, base_horizon, batch_size,
                                  phases=None, step=1.0, max_extend=10_000.0):
    horizon   = float(base_horizon)
    instances = build_instances(msgset, R, horizon, phases=phases)
    base_count = len(instances)
    remainder  = base_count % batch_size

    if remainder == 0:
        return instances, horizon, base_count, 0

    target = base_count + (batch_size - remainder)
    limit  = horizon + float(max_extend)

    while len(instances) < target:
        horizon += float(step)
        if horizon > limit:
            raise RuntimeError(
                f"Could not complete full batch within max_extend={max_extend} ms."
            )
        instances = build_instances(msgset, R, horizon, phases=phases)

    return instances, horizon, base_count, remainder
