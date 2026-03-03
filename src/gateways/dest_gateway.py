#src/dest_gateway.py


def decap_delay_ms(batch_size, fixed_overhead_us: float = 30.0, per_msg_overhead_us: float = 5.0, queue_overhead_us: float = 2.0,) -> float:
    """
    Worst-case TSN->CAN decapsulation delay (ms).
    Model:
      δ_decap = fixed_overhead_us + batch_size * (per_msg_overhead_us + queue_overhead_us)
      The  wcrt of CAN is then added to this to get total delay from TSN frame arrival to CAN message release.
    """
    if batch_size <= 0:
        raise ValueError("batch_size must be >= 1")

    total_us = fixed_overhead_us + batch_size * (per_msg_overhead_us + queue_overhead_us)
    return total_us / 1000.0