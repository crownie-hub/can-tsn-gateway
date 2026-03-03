# can_classic.py
def bitrate_to_tbit_ms(bitrate_bps: int) -> float:
    """
    Convert bit time to milliseconds (ms).
    """
    return 1000.0 / float(bitrate_bps)


def classic_tx_time_ms(payload_bytes: int, tbit_ms: float) -> float:
    """
    Classic CAN frame transmission time (ms).
    """
    bits = 55 + 10 * int(payload_bytes)
    return bits * float(tbit_ms)