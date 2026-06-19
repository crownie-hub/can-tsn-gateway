# src/tsn/wcrt.py
# TSN TAS worst-case response time.
# All internal computation in microseconds (us); public API returns us.
# Use wcrt_ms() for milliseconds.
#
# Closed-form WCRT (Thiele et al. 2015):
#   I^max = sum_{k!=j} ceil(T_c / T_k) * C_k^max
#   R_j   = T_c + I^max + C_j^max + hop_delay

import math
from dataclasses import dataclass
from typing import Sequence

from .gcl  import GCL
from .flow import Flow


@dataclass(frozen=True)
class TSNConfig:
    link_speed_mbps:       int   = 1000
    num_switches:          int   = 1
    switch_processing_us:  float = 3.0
    propagation_delay_us:  float = 1.0
    ethernet_header_bytes: int   = 14
    vlan_tag_bytes:        int   = 4
    tsn_header_bytes:      int   = 8
    crc_bytes:             int   = 4
    interframe_gap_bytes:  int   = 12
    preamble_sfd_bytes:    int   = 8

    def header_bytes(self):
        return (self.ethernet_header_bytes + self.vlan_tag_bytes
                + self.tsn_header_bytes + self.crc_bytes)

    def on_wire_bytes(self, payload_bytes):
        base   = payload_bytes + self.header_bytes()
        padded = max(64, base)
        return padded + self.interframe_gap_bytes + self.preamble_sfd_bytes

    def tx_time_us(self, payload_bytes):
        return self.on_wire_bytes(payload_bytes) / (self.link_speed_mbps / 8.0)

    def hop_delay_us(self):
        return self.num_switches * (self.switch_processing_us
                                    + self.propagation_delay_us)


def interference_us(flow_j, competing_flows, T_c_us, config):
    total = 0.0
    for fk in competing_flows:
        if fk.flow_id == flow_j.flow_id:
            continue
        T_k_us = float(fk.period) * 1000.0
        total += math.ceil(T_c_us / T_k_us) * config.tx_time_us(fk.payload_bytes)
    return total


def wcrt_us(flow_j, competing_flows, gcl, config):
    w      = gcl.window(flow_j.priority)
    T_c_us = float(w.cycle_us)
    C_j    = config.tx_time_us(flow_j.payload_bytes)
    I_max  = interference_us(flow_j, competing_flows, T_c_us, config)
    return T_c_us + I_max + C_j + config.hop_delay_us()


def wcrt_ms(flow_j, competing_flows, gcl, config):
    return wcrt_us(flow_j, competing_flows, gcl, config) / 1000.0


def actual_tx_delay_us(payload_bytes, priority, current_time_us, gcl, config):
    # simulation path only — not a WCRT bound
    w         = gcl.window(priority)
    C_us      = config.tx_time_us(payload_bytes)
    next_open = gcl.next_window_open_us(priority, current_time_us)
    gate_wait = max(0.0, next_open - current_time_us)
    t_hat     = float(w.window_us)
    spill     = 0.0
    if t_hat > 0 and C_us > t_hat:
        spill = (math.ceil(C_us / t_hat) - 1) * float(w.cycle_us)
    return gate_wait + C_us + spill + config.hop_delay_us()
