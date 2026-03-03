from dataclasses import dataclass
from typing import Optional

from .can_message import CANMessage
from .pipeline.can_instances import CANInstance


@dataclass
class PeriodicSource:
    """
    Defines one periodic CAN flow (message type).
    """
    source_id: int
    can_id: str                 # fixed CAN ID in hex, e.g. "A0"
    period_ms: float
    payload_size: int = 8
    deadline_ms: Optional[float] = None
    priority: Optional[int] = None  # if None, derived from can_id

    def to_message_def(self) -> CANMessage:
        """
        Flow-level definition (for RTA).
        """
        if self.deadline_ms is None:
            self.deadline_ms = self.period_ms
        if self.priority is None:
            self.priority = int(self.can_id, 16)


        return CANMessage(
            msg_id=self.can_id,          # flow ID is the CAN ID
            source_id=self.source_id,
            period=self.period_ms,
            arrival_time=0.0,            
            priority=self.priority,
            payload_size=self.payload_size,
            deadline=self.deadline_ms,
        )

    def generate_instance(self, flow_id: int, inst_id: int, release_ms: float, can_delay_ms: float) -> CANInstance:
        """
        arrive_gw_ms = release_ms + can_delay_ms
        """
        D = self.deadline_ms if self.deadline_ms is not None else self.period_ms
        return CANInstance(
            flow_id=flow_id,
            inst_id=inst_id,
            can_id=int(self.can_id, 16),
            release_ms=release_ms,
            can_delay_ms=can_delay_ms,
            arrive_gw_ms=release_ms + can_delay_ms,
            period_ms=self.period_ms,
            deadline_ms=D,
            payload_bytes=self.payload_size,
        )