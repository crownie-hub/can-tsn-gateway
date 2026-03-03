# src/pipeline/can_instances.py
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence


@dataclass(frozen=True)
class CANInstance:
    """
    One instance of a periodic CAN message as seen by the CAN→TSN gateway.
    """
    # id flow = message set ID
    flow_id: int
    inst_id: int
    can_id: int

    # timing
    release_ms: float
    can_delay_ms: float
    arrive_gw_ms: float

    #charac
    period: float
    deadline: float
    payload_size: int


    can_blocking_ms: Optional[float] = None
    can_interf_ms: Optional[float] = None


def build_instances_to_gateway(
    msgset,
    R_ms: Sequence[float],
    horizon_ms: float,
    phases_ms: Optional[Sequence[float]] = None,
) -> List[CANInstance]:
    """
    Bound for now:
        arrive_gw_ms = release_ms + R_i
    Generates instances whose release_ms <= horizon_ms/length of time. 
    Returns in gateway arrival order (arrive_gw_ms).
    """
    horizon_ms = float(horizon_ms)

    if phases_ms is None:
        phases_ms = [0.0] * len(msgset)

    if len(R_ms) != len(msgset):
        raise ValueError("R_ms must have the same length as msgset")

    if len(phases_ms) != len(msgset):
        raise ValueError("phases_ms must have the same length as msgset")

    instances: List[CANInstance] = []

    for i, msg in enumerate(msgset):
        P = float(msg.period)
        D = float(msg.deadline)
        payload = int(msg.payload_size)
        can_id = int(msg.msg_id, 16)

        Ri = float(R_ms[i])
        phase = float(phases_ms[i])

        j = 0
        while True:
            r = phase + j * P
            if r > horizon_ms:
                break

            arrive = r + Ri

            instances.append(
                CANInstance(
                    flow_id=i,
                    inst_id=j,
                    can_id=can_id,
                    release_ms=r,
                    can_delay_ms=Ri,
                    arrive_gw_ms=arrive,
                    period=P,
                    deadline=D,
                    payload_size=payload,
                )
            )
            j += 1

    instances.sort(key=lambda x: x.arrive_gw_ms)
    return instances


def build_instances_full_batches(
    msgset,
    R_ms: Sequence[float],
    base_horizon_ms: float,
    batch_size: int,
    phases_ms: Optional[Sequence[float]] = None,
    step_ms: float = 1.0,
    max_extend_ms: float = 10_000.0,
):
    """
    Extends length until total instances is a multiple of batch_size
    (i.e., last batch is FULL; no partial batches).
    Returns: (instances, extended_horizon_ms, base_count, remainder)
    """
    if batch_size <= 0:
        raise ValueError("batch_size must be > 0")

    base_horizon_ms = float(base_horizon_ms)
    horizon = base_horizon_ms

    instances = build_instances_to_gateway(msgset, R_ms, horizon, phases_ms=phases_ms)
    base_count = len(instances)
    remainder = base_count % batch_size

    if remainder == 0:
        return instances, horizon, base_count, 0

    target = base_count + (batch_size - remainder)
    limit = base_horizon_ms + float(max_extend_ms)

    while len(instances) < target:
        horizon += float(step_ms)
        if horizon > limit:
            raise RuntimeError(
                f"Could not complete a full final batch within max_extend_ms={max_extend_ms}. "
                f"Have {len(instances)}, need {target}."
            )
        instances = build_instances_to_gateway(msgset, R_ms, horizon, phases_ms=phases_ms)

    return instances, horizon, base_count, remainder