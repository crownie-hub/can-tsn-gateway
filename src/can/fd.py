# src/can/fd.py

def _dlc(d):
    d = int(d)
    if d <= 8:  return d
    if d <= 12: return 12
    if d <= 16: return 16
    if d <= 20: return 20
    if d <= 24: return 24
    if d <= 32: return 32
    if d <= 48: return 48
    return 64

def fd_tx_time(payload_bytes, tbit, dtbit):
    dlc = _dlc(payload_bytes)
    if dlc <= 16:
        return 33 * float(tbit) + (35 + 10 * dlc) * float(dtbit)
    return 33 * float(tbit) + (40 + 10 * dlc) * float(dtbit)
