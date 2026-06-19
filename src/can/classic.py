# src/can/classic.py

def bitrate_to_tbit(bitrate_bps):
    return 1000.0 / float(bitrate_bps)

def classic_tx_time(payload_bytes, tbit):
    return (55 + 10 * int(payload_bytes)) * float(tbit)
