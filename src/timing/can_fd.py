# can_fd.py
def can_fd_data_bytes(d: int) -> int:
    """
    payload length to CAN-FD DLC payload bytes.
    """
    d = int(d)
    return (
        d if d <= 8 else
        12 if d <= 12 else
        16 if d <= 16 else
        20 if d <= 20 else
        24 if d <= 24 else
        32 if d <= 32 else
        48 if d <= 48 else
        64
    )


def fd_tx_time_ms(payload_bytes: int, tbit_ms: float, dtbit_ms: float) -> float:
  
    dlc = can_fd_data_bytes(payload_bytes)

    tbit_ms = float(tbit_ms)
    dtbit_ms = float(dtbit_ms)

    if dlc <= 16:
        return 33 * tbit_ms + (35 + 10 * dlc) * dtbit_ms
    else:
        return 33 * tbit_ms + (40 + 10 * dlc) * dtbit_ms