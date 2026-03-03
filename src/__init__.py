"""
CAN-TSN Gateway Implementation

This package implements Fixed-Priority and FIFO batch forwarding for CAN-TSN gateways.
"""

__version__ = '1.0.0'


from .can_message import CANMessage
from .sources import PeriodicSource
from .tsn_frame import TSNFrame
from .utils import (
    calculate_hyperperiod,
    calculate_arrival_function,
    calculate_required_arrivals,
    calculate_batch_number,
    find_batch_completion_time
)

__all__ = [
    'CANMessage',
    'PeriodicSource',
    'TSNFrame',
    'calculate_hyperperiod',
    'generate_message_arrivals',
    'calculate_arrival_function',
    'calculate_required_arrivals',
    'calculate_batch_number',
    'find_batch_completion_time',
]
