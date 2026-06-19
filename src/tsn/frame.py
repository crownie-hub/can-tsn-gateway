# src/tsn/frame.py
# All times in milliseconds (ms).

from dataclasses import dataclass, field
from typing import Any, List


@dataclass
class TSNFrame:
    frame_id:             str
    instances:            List[Any] = field(default_factory=list)
    creation_time:        float     = 0.0
    tx_start:             float     = 0.0
    tx_end:               float     = 0.0
    priority:             int       = 0
    batch_header_bytes:   int       = 0
    per_can_header_bytes: int       = 0

    def add_instance(self, inst):
        self.instances.append(inst)

    def batch_size(self):
        return len(self.instances)

    def payload_bytes(self):
        # D_j = batch_header + sum(D_i + per_can_header) for m_i in F_j
        return (self.batch_header_bytes
                + sum(int(inst.payload_size) + self.per_can_header_bytes
                      for inst in self.instances))

    def __repr__(self):
        return (f"TSNFrame({self.frame_id}, n={self.batch_size()}, "
                f"t=[{self.tx_start:.3f}, {self.tx_end:.3f}] ms)")