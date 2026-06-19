# examples/fp_bf_example.py
# Fixed-Priority Buffer-Full gateway example.
from common import build_setup, print_setup, print_results
from src.gateways.fp_bf import FPBatchGateway

BATCH_SIZE = 5
HORIZON    = 100.0

print("=" * 60)
print("FP-BF Gateway Example")
print("=" * 60)

s  = build_setup(batch_size=BATCH_SIZE, horizon=HORIZON)
print_setup(s)

gw     = FPBatchGateway(
    batch_size=BATCH_SIZE,
    tsn_calc=s["tsn_calc"],
    default_priority=0,
)
result = gw.run_on_instances(s["instances"], length=HORIZON)
print_results(result, "FP-BF")

# Assertions
assert result["num_batches"] > 0
assert all(r["total_delay"] > 0 for r in result["per_message"])
print("\nAll assertions passed ✓")
