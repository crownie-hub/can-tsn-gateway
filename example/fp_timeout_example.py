# examples/fp_timeout_example.py
# Fixed-Priority Timeout gateway example.
from common import build_setup, print_setup, print_results
from src.gateways.fp_timeout import FPTimeoutGateway

BATCH_SIZE      = 5
HORIZON         = 100.0
GATEWAY_PERIOD  = 10.0   # ms

print("=" * 60)
print("FP-Timeout Gateway Example")
print(f"  gateway_period = {GATEWAY_PERIOD} ms")
print("=" * 60)

s  = build_setup(batch_size=BATCH_SIZE, horizon=HORIZON)
print_setup(s)

gw     = FPTimeoutGateway(
    batch_size=BATCH_SIZE,
    gateway_period=GATEWAY_PERIOD,
    tsn_calc=s["tsn_calc"],
    default_priority=0,
    start_offset=0.0,
)
result = gw.run_on_instances(s["instances"], length=HORIZON)
print_results(result, "FP-Timeout")

# Partial batches allowed; FP selects highest priority first each cycle
sizes = [r["batch_size_actual"] for r in result["per_message"]]
print(f"\n  batch sizes seen : {sorted(set(sizes))}")
assert result["num_batches"] > 0
assert all(r["total_delay"] > 0 for r in result["per_message"])
print("All assertions passed ✓")
