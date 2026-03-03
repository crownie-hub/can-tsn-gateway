

import examples.setup  # noqa: F401
from src.sources import PeriodicSource
from src.gateways.fixed_priority_gateway import FixedPriorityGateway
from src.utils import calculate_hyperperiod, generate_message_arrivals
from src.timing.can_bus import CANBusConfig, compute_response_times_ms

