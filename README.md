# CAN-TSN Gateway Implementation

Implementation of FIFO and Fixed-Priority batch forwarding for CAN-TSN gateways.

## Author
Essl - Omolade Ikumapayi
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


