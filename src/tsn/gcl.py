# src/gcl.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class GCLWindow:
    """Defines a GCL window for a given priority.
    We keep it simple, one open window per cycle for that queue.
    """
    cycle_us: int
    window_us: int


class GCL:
    """
    Map TSN priority (0..7) -> (cycle_us, window_us).
    """

    def __init__(self, windows: Dict[int, GCLWindow]):
        # validate 
        for p, w in windows.items():
            if not (0 <= int(p) <= 7):
                raise ValueError(f"Invalid TSN priority {p}; must be 0..7")
            if w.cycle_us <= 0:
                raise ValueError(f"cycle_us must be > 0 for priority {p}")
            if w.window_us <= 0:
                raise ValueError(f"window_us must be > 0 for priority {p}")
            if w.window_us > w.cycle_us:
                raise ValueError(f"window_us cannot exceed cycle_us for priority {p}")

        self._windows = {int(p): w for p, w in windows.items()}

    def window(self, priority: int) -> GCLWindow:
        p = int(priority)
        if p not in self._windows:
            raise KeyError(f"No GCL window defined for priority {p}")
        return self._windows[p]

    @staticmethod
    def sample_uniform(cycle_us: int = 1000, window_us: int = 200) -> "GCL":
        """Same cycle/window for all priorities 0..7."""
        return GCL({p: GCLWindow(cycle_us=cycle_us, window_us=window_us) for p in range(8)})