# example/fifo_bf_example.py
import sys, os
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from common import (build_setup, print_can_rta, print_tsn,
                    print_gateway_components, print_enc_delay_trace,
                    print_batch_trace, print_e2e_table)
from src.gateways.fifo_bf import FIFOBatchGateway

BATCH_SIZE = 2

s  = build_setup(batch_size=BATCH_SIZE)
gw = FIFOBatchGateway(s["msgset"], s["R"], s["C"],
                      BATCH_SIZE, s["tsn_wcrt"], s["c_can_max"])

print_can_rta(s)
print_tsn(s)
print_gateway_components("FIFO-BF", gw.delta_enc,
                          gw.tsn_wcrt, gw.delta_dec, BATCH_SIZE)
print_enc_delay_trace(s, gw.delta_enc, label="FIFO-BF")
print_batch_trace(s, gw.delta_enc, n_batches=3)
print_e2e_table(gw.results(s["R_dst"]))