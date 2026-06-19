# src/tsn/gcl.py
# Gate Control List for IEEE 802.1Qbv TAS.
# All times in microseconds (us).

import math
from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class GCLWindow:
    cycle_us:  int
    window_us: int
    offset_us: int = 0


class GCL:
    def __init__(self, windows: Dict[int, GCLWindow]):
        self._windows = {int(p): w for p, w in windows.items()}

    def window(self, priority):
        p = int(priority)
        if p not in self._windows:
            raise KeyError(f"No GCL window for priority {p}")
        return self._windows[p]

    def next_window_open_us(self, priority, t_us):
        w      = self.window(priority)
        cycle  = float(w.cycle_us)
        offset = float(w.offset_us)
        window = float(w.window_us)
        phase  = (t_us - offset) % cycle
        if phase < 0:
            phase += cycle
        if phase <= window:
            return t_us
        return t_us + (cycle - phase)

    @staticmethod
    def sample_uniform(cycle_us=1000, window_us=200):
        return GCL({p: GCLWindow(cycle_us, window_us) for p in range(8)})

    @staticmethod
    def sample_staggered(cycle_us=1000, window_us=200):
        windows = {}
        for p in range(8):
            offset = (7 - p) * window_us
            if offset + window_us > cycle_us:
                offset = 0
            windows[p] = GCLWindow(cycle_us, window_us, offset)
        return GCL(windows)
