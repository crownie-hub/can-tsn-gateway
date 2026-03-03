# src/tsn_frame.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List


@dataclass
class TSNFrame:
    frame_id: str
    instances: List[Any] = field(default_factory=list)  
    # timestamps (ms) filled by gateway/pipeline
    creation_time_ms: float = 0.0
    tx_start_time_ms: float = 0.0
    tx_end_time_ms: float = 0.0

    priority: int = 0  # same for now
    batch_header_bytes: int = 0
    per_can_header_bytes: int = 0

    def add_instance(self, inst: Any) -> None:
        self.instances.append(inst)

    def batch_size(self) -> int:
        return len(self.instances)

    def payload_bytes(self) -> int:
        # TSN payload = batch header + sum(CAN payload + per-CAN header)
        return (
            int(self.batch_header_bytes)
            + sum(int(i.payload_size) + int(self.per_can_header_bytes) for i in self.instances)
        )

    def message_ids(self) -> List[str]:
        return [f"m{i.flow_id}_{i.inst_id}" for i in self.instances]

    def __repr__(self) -> str:
        return (
            f"TSNFrame(id={self.frame_id}, n={self.batch_size()}, "
            f"created={self.creation_time_ms:.3f}ms, "
            f"tx=[{self.tx_start_time_ms:.3f},{self.tx_end_time_ms:.3f}]ms)"
        )