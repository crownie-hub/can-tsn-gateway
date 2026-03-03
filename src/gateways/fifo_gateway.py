# src/gateways/fifo_gateway.py
from __future__ import annotations

from typing import Any, Dict, List, Optional
from src.tsn_frame import TSNFrame


class FIFOBatchGateway:
    def __init__(
        self,
        batch_size: int,
        tsn_calc,  # TSNWCRTCalculator
        *,
        default_priority: int = 0,   # 0..7
        batch_header_bytes: int = 0,
        per_can_header_bytes: int = 0,
    ):
        if batch_size <= 0:
            raise ValueError("batch_size must be > 0")
        if not (0 <= int(default_priority) <= 7):
            raise ValueError("default_priority must be in 0..7")

        self.batch_size = int(batch_size)
        self.tsn_calc = tsn_calc

        self.default_priority = int(default_priority)
        self.batch_header_bytes = int(batch_header_bytes)
        self.per_can_header_bytes = int(per_can_header_bytes)

        self.frames: List[TSNFrame] = []

    @staticmethod
    def _msg_id(inst: Any) -> str:
        return f"m{inst.flow_id}_{inst.inst_id}"

    def assign_priority(self, batch: List[Any]) -> int:
        """
        For now, return same priority for all frames.
        Replace later with mapping based on flow_id, deadline, etc.
        """
        return self.default_priority

    def run_on_instances(
        self,
        instances: List[Any],
        *,
        length: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not instances:
            return {"num_batches": 0, "per_message": [], "frames": []}

        inst = sorted(instances, key=lambda x: x.arrive_gw_ms)

        self.frames = []
        per_message: List[Dict[str, Any]] = []
        batch_no = 0

        total_full = (len(inst) // self.batch_size) * self.batch_size

        for start in range(0, total_full, self.batch_size):
            batch = inst[start : start + self.batch_size]
            batch_no += 1

            t_form_ms = float(batch[-1].arrive_gw_ms)
            prio = self.assign_priority(batch)

            frame = TSNFrame(
                frame_id=f"frame_{batch_no}",
                creation_time_ms=t_form_ms,
                priority=prio,
                batch_header_bytes=self.batch_header_bytes,
                per_can_header_bytes=self.per_can_header_bytes,
            )
            for m in batch:
                frame.add_instance(m)

            payload_bytes = frame.payload_bytes()

            # priority used by TSN WCRT via GCL
            wcrt_us = float(self.tsn_calc.calculate_wcrt(payload_bytes, priority=frame.priority))
            t_depart_ms = t_form_ms + (wcrt_us / 1000.0)

            frame.tx_start_time_ms = t_form_ms
            frame.tx_end_time_ms = t_depart_ms
            self.frames.append(frame)

            for pos, m in enumerate(batch, start=1):
                arrive = float(m.arrive_gw_ms)
                total_delay_ms = t_depart_ms - arrive

                row = {
                    "message_id": self._msg_id(m),
                    "flow_id": m.flow_id,
                    "inst_id": m.inst_id,
                    "can_id": getattr(m, "can_id", None),

                    "arrive_gw_ms": arrive,
                    "batch_number": batch_no,
                    "position": pos,

                    "frame_id": frame.frame_id,
                    "frame_priority": frame.priority,
                    "frame_payload_bytes": payload_bytes,
                    "tsn_wcrt_us": wcrt_us,
                    "depart_tsn_ms": t_depart_ms,

                    "total_delay_ms": total_delay_ms,
                }

                if length is None or arrive <= float(length):
                    per_message.append(row)

        return {"num_batches": batch_no, "per_message": per_message, "frames": self.frames}