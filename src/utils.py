"""
Utility Functions for CAN-TSN Gateway Analysis

Contains helper functions for worst-case delay analysis and arrival function calculations.
"""

import math
from typing import List, Tuple
from .sources import PeriodicSource


def calculate_arrival_function(sources: List[PeriodicSource], time: float, 
                               start_time: float = 0.0) -> int:
    """
    Calculate the arrival function A(t) - total number of message arrivals 
    from all sources up to time t.
    
    For periodic sources with synchronous release (all start at start_time),
    the number of arrivals from source i by time t is:
        floor((t - start_time) / period_i) + 1  if t >= start_time, else 0
    
    Args:
        sources: List of periodic sources
        time: Time point to evaluate (ms)
        start_time: Initial release time for all sources (ms)
        
    Returns:
        Total number of message arrivals by time t
    """
    if time < start_time:
        return 0
    
    total_arrivals = 0
    for source in sources:
        # Number of complete periods elapsed
        periods_elapsed = (time - start_time) / source.period
        # Add 1 for the initial release at start_time
        arrivals = math.floor(periods_elapsed) + 1
        total_arrivals += arrivals
    
    return total_arrivals


def calculate_required_arrivals(position_in_queue: int, batch_size: int) -> int:
    """
    Calculate the total number of arrivals (NR) required for a message 
    at position to be transmitted under FIFO batch forwarding.
    
    From the paper:
        fb = floor(pos/ n)  (batch number)
        NR = n * ceil(pos/ n) = n * fb  if pos is multiple of n, else n * (fb + 1)
    
    Simplified: NR = n * ceil(pos / n)
    
    Args:
        position_in_queue: Position η of the message (1-indexed)
        batch_size: Batch size n
        
    Returns:
        Total arrivals required
    """
    return batch_size * math.ceil(position_in_queue / batch_size)


def calculate_batch_number(position_in_queue: int, batch_size: int) -> int:
    """
    Calculate which batch a message belongs to.
    
    Args:
        position_in_queue: Position η of the message (1-indexed)
        batch_size: Batch size n
        
    Returns:
        Batch number (0-indexed)
    """
    return math.floor((position_in_queue - 1) / batch_size)


def find_batch_completion_time(sources: List[PeriodicSource], 
                               required_arrivals: int,
                               start_time: float = 0.0,
                               max_time: float = 1000.0,
                               time_step: float = 0.1) -> float:
    """
    Find the earliest time t when the arrival function reaches required_arrivals.
    
    This implements the search for w_i(q) - the waiting time when batch completion
    condition is satisfied.
    
    Args:
        sources: List of periodic sources
        required_arrivals: Number of arrivals needed (NR)
        start_time: Initial release time (ms)
        max_time: Maximum time to search (ms)
        time_step: Time resolution for search (ms)
        
    Returns:
        Time when batch is complete (ms), or max_time if not found
    """
    # Start searching from start_time
    t = start_time
    
    while t <= max_time:
        arrivals = calculate_arrival_function(sources, t, start_time)
        if arrivals >= required_arrivals:
            return t
        t += time_step
    
    return max_time  # Not found within max_time


def calculate_hyperperiod(sources: List[PeriodicSource]) -> float:
    """
    Calculate the hyperperiod (LCM of all periods).
    
    Args:
        sources: List of periodic sources
        
    Returns:
        Hyperperiod in ms
    """
    from math import gcd
    from functools import reduce
    
    def lcm(a, b):
        return abs(a * b) // gcd(int(a), int(b))
    
    periods = [int(source.period) for source in sources]
    return reduce(lcm, periods)


def generate_message_arrivals(sources: List[PeriodicSource], 
                              simulation_time: float,
                              start_time: float = 0.0) -> List[Tuple[float, int]]:
    """
    Generate a sorted list of all message arrival times.
    
    Args:
        sources: List of periodic sources
        simulation_time: Total simulation time (ms)
        start_time: Initial release time (ms)
        
    Returns:
        List of (arrival_time, source_id) tuples, sorted by arrival time
    """
    arrivals = []
    
    for source in sources:
        t = start_time
        while t <= simulation_time:
            arrivals.append((t, source.source_id))
            t += source.period
    
    # Sort by arrival time
    arrivals.sort(key=lambda x: x[0])
    return arrivals
