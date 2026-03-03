# src/tsn_wcrt.py
from __future__ import annotations

from dataclasses import dataclass
from src.tsn.gcl import GCL


@dataclass(frozen=True)
class TSNConfig:
    # Physical parameters
    link_speed_mbps: int
    num_switches: int
    switch_processing_us: int
    propagation_delay_us: int

    # encapsulation overhead
    ethernet_header_bytes: int = 14
    vlan_tag_bytes: int = 4
    tsn_header_bytes: int = 8
    crc_bytes: int = 4
    interframe_gap_bytes: int = 12
    preamble_sfd_bytes: int = 0  

    def total_overhead_bytes(self) -> int:
        return (
            self.ethernet_header_bytes
            + self.vlan_tag_bytes
            + self.tsn_header_bytes
            + self.crc_bytes
            + self.interframe_gap_bytes
            + self.preamble_sfd_bytes
        )

    def bytes_per_us(self) -> float:
        # Mbps/8 bytes per microsecond
        return float(self.link_speed_mbps) / 8.0


class TSNWCRTCalculator:
    """
    TAS-style that uses a GCL map and per-frame priority.
    """

    def __init__(self, config: TSNConfig, gcl: GCL):
        self.config = config
        self.gcl = gcl

    def on_wire_bytes(self, payload_bytes: int) -> int:
        if payload_bytes < 0:
            raise ValueError("payload_bytes must be >= 0")
        return int(payload_bytes) + self.config.total_overhead_bytes()

    def tx_time_us(self, payload_bytes: int) -> float:
        total_bytes = self.on_wire_bytes(payload_bytes)
        return total_bytes / self.config.bytes_per_us()

    def calculate_wcrt(self, payload_bytes: int, priority: int) -> float:
        """
        Returns WCRT in microseconds for a frame of payload_bytes sent in queue priority.
        """
        w = self.gcl.window(priority)
        cycle_us = float(w.cycle_us)
        window_us = float(w.window_us)

        max_gap_us = cycle_us - window_us  # worst-case wait to next open window
        if max_gap_us < 0:
            max_gap_us = 0.0

        tx_us = self.tx_time_us(payload_bytes)

        #  spillover if tx doesn't fit in one window
        extra_cycles_delay = 0.0
        if window_us > 0 and tx_us > window_us:
            windows_needed = int((tx_us + window_us - 1.0) // window_us)  # ceil-ish
            extra_cycles_delay = float(windows_needed - 1) * cycle_us

        hop_delay = float(self.config.num_switches) * (
            float(self.config.switch_processing_us) + float(self.config.propagation_delay_us)
        )

        return float(max_gap_us) + tx_us + extra_cycles_delay + hop_delay