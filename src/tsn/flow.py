# src/tsn/flow.py


from dataclasses import dataclass
from typing import Optional


@dataclass
class Flow:
    flow_id:       int
    period:        float
    payload_bytes: int
    priority:      int
    deadline:      Optional[float] = None

    def __post_init__(self):
        if self.deadline is None:
            self.deadline = float(self.period)
