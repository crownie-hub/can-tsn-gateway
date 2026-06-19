# src/can/sim_bridge.py
# Converts CAN simulator output into gateway-ready CANInstance objects.
# All times in milliseconds (ms).

from .instance  import CANInstance
from .simulator import FrameInstance, CANBusSimulator


def build_instances_from_sim(completed, use_arrival="finish"):
    # use_arrival: "finish" | "start" | "release"
    instances = []
    for f in completed:
        if use_arrival == "finish":
            if f.finish_tx is None: continue
            arrive_gw = float(f.finish_tx)
        elif use_arrival == "start":
            if f.start_tx is None: continue
            arrive_gw = float(f.start_tx)
        else:
            arrive_gw = float(f.release)

        finish    = float(f.finish_tx) if f.finish_tx is not None else arrive_gw
        can_delay = finish - float(f.release)

        instances.append(CANInstance(
            flow_id=int(f.msg_index),
            inst_id=int(f.seq),
            can_id=int(f.msg_id),
            release=float(f.release),
            can_delay=can_delay,
            arrive_gw=arrive_gw,
            period=float(f.period),
            deadline=float(f.deadline) if f.deadline is not None else float(f.period),
            payload_size=int(f.payload_bytes),
        ))

    instances.sort(key=lambda x: x.arrive_gw)
    return instances


def build_sim_instances_full_batches(
    simulator_factory, *, batch_size, base_horizon,
    step=1.0, max_extend=10_000.0, use_arrival="finish",
):
    horizon    = float(base_horizon)
    limit      = horizon + float(max_extend)
    instances  = build_instances_from_sim(
        simulator_factory(horizon).run(), use_arrival=use_arrival)
    base_count = len(instances)
    remainder  = base_count % batch_size

    if remainder == 0:
        return instances, horizon, base_count, 0

    target = base_count + (batch_size - remainder)
    while len(instances) < target:
        horizon += float(step)
        if horizon > limit:
            raise RuntimeError(
                f"Could not complete full batch within max_extend={max_extend} ms.")
        instances = build_instances_from_sim(
            simulator_factory(horizon).run(), use_arrival=use_arrival)

    return instances, horizon, base_count, remainder
