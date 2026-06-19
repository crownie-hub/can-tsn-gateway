# src/can/message.py
# All times in milliseconds (ms).

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class CANMessage:
    msg_id:       str
    source_id:    int
    period:       float
    payload_size: int            = 8
    deadline:     Optional[float] = None
    priority:     Optional[int]   = None

    def __post_init__(self):
        if self.deadline is None:
            self.deadline = float(self.period)
        if self.priority is None:
            self.priority = int(self.msg_id, 16)


def message_set() -> List[CANMessage]:
    # source 0: Vehicle Control, 1: Brakes, 2: Battery,
    # source 3: Driver, 4: IMC, 5: Transmission
    
    return [
        CANMessage("A0", 0, 5.0,    1, 5.0),
        CANMessage("B0", 0, 10.0,   6, 10.0),
        CANMessage("D0", 0, 1000.0, 1, 1000.0),
        CANMessage("A1", 1, 5.0,    2, 5.0),
        CANMessage("C1", 1, 100.0,  1, 100.0),
        CANMessage("B2", 2, 10.0,   1, 10.0),
        CANMessage("C2", 2, 100.0,  4, 100.0),
        CANMessage("D2", 2, 1000.0, 3, 1000.0),
        CANMessage("A3", 3, 5.0,    1, 5.0),
        CANMessage("B3", 3, 10.0,   2, 10.0),
        CANMessage("A4", 4, 5.0,    2, 5.0),
        CANMessage("B4", 4, 10.0,   2, 10.0),
        CANMessage("A5", 5, 5.0,    1, 5.0),
        CANMessage("C5", 5, 100.0,  1, 100.0),
        CANMessage("D5", 5, 1000.0, 1, 1000.0),
    ]
    
 