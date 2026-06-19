# src/gateway/decap.py
# Decapsulation delay at the TSN-to-CAN egress gateway.
#
# delta_dec = (n - 1) * C_CAN_max


def decap_delay(n, c_can_max):
    return (n - 1) * float(c_can_max)