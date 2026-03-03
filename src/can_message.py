# src/can_tsn_gateway/can_message.py
from dataclasses import dataclass
from typing import Optional


@dataclass
class CANMessage:

    msg_id: str                 # CAN arbitration ID as hex string, e.g. "A0"
    source_id: int              
    period: float               #
    payload_size: int = 8       
    deadline: Optional[float] = None
    priority: Optional[int] = None

    def __post_init__(self):
        
        if self.deadline is None:
            self.deadline = float(self.period)

        if self.priority is None:
            self.priority = int(self.msg_id, 16)

def message_set():
    # msg_id MUST be the CAN ID in hex
  
    return [
        CANMessage(msg_id="A0", source_id=0, period=10.0,    payload_size=8, deadline=10.0),
        CANMessage(msg_id="A3", source_id=0, period=15.0,    payload_size=8, deadline=20.0),
        CANMessage(msg_id="B0", source_id=0, period=25.0,   payload_size=8, deadline=10.0),
        CANMessage(msg_id="D0", source_id=0, period=1000.0, payload_size=8, deadline=1000.0),
        CANMessage(msg_id="A2", source_id=1, period=30.0,    payload_size=8, deadline=10.0),
        CANMessage(msg_id="C1", source_id=1, period=100.0,  payload_size=8, deadline=100.0),
    ]