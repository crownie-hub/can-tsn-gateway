# CAN-TSN Gateway Implementation

Implementation of FIFO and Fixed-Priority batch forwarding for CAN-TSN gateways.

## Author
Zaynab - PhD Student, Computer Science  
University of Colorado Colorado Springs (UCCS)

## Project Structure

```
can_tsn_gateway/
├── src/                    # Core implementation
│   ├── can_message.py      # CAN message and periodic source
│   ├── tsn_frame.py        # TSN frame encapsulation
│   ├── utils.py            # Utility functions
│   └── gateways/
│       ├── fifo_gateway.py           # FIFO batch forwarding
│       └── fixed_priority_gateway.py # Fixed-Priority batch forwarding
├── examples/               # Usage examples
├── tests/                  # Unit tests
├── docs/                   # Documentation
└── README.md              # This file
```

## Quick Start

### FIFO Gateway
```python
from src.gateways import FIFOBatchGateway
from src.can_message import PeriodicSource

gateway = FIFOBatchGateway(batch_size=4, encapsulation_overhead=1.0)
source = PeriodicSource(source_id=1, period=5.0, payload_size=8)
gateway.add_source(source)
```

### Fixed-Priority Gateway
```python
from src.gateways import FixedPriorityGateway

gateway = FixedPriorityGateway(batch_size=4, encapsulation_overhead=1.0)
source = PeriodicSource(1, period=5.0, priority=1, payload_size=8)  # HIGHEST
gateway.add_source(source)
```

## Running Examples

```bash
cd can_tsn_gateway
python examples/use_fp_gateway_directly.py
```

## Research Context

This implementation is part of PhD research on security vulnerabilities in LLM-based Multi-Agent Systems, focusing on attack methodologies for dynamic multi-agent systems and time-sensitive networking constraints.

## License

Research code for PhD dissertation - UCCS
