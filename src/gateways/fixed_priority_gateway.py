# src/gateways/fixed_priority_gateway.py
from __future__ import annotations

from typing import Any, Dict, List, Optional
from src.tsn_frame import TSNFrame


class FixedPriorityBatchGateway:
    def __init__(
        self,
        batch_size: int,
        tsn_calc,
        *,
        default_priority: int = 0,       # TSN priority for frames (0..7)
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
        self._buffer: List[Any] = []

    @staticmethod
    def _msg_id(inst: Any) -> str:
        return f"m{inst.flow_id}_{inst.inst_id}"

    @staticmethod
    def _fp_key(inst: Any):
       

        can_prio = getattr(inst, "priority", None)
        if can_prio is None:
            can_prio = int(getattr(inst, "can_id", "0"), 16) if getattr(inst, "can_id", None) else int(inst.flow_id)
        return (int(can_prio), float(inst.arrive_gw_ms), int(inst.flow_id), int(inst.inst_id))

    def assign_tsn_priority(self, batch: List[Any]) -> int:
    
        return self.default_priority

    def run_on_instances(
        self,
        instances: List[Any],
        *,
        length: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not instances:
            return {"num_batches": 0, "per_message": [], "frames": []}

        inst_sorted = sorted(instances, key=lambda x: float(x.arrive_gw_ms))

        self.frames = []
        self._buffer = []
        per_message: List[Dict[str, Any]] = []
        batch_no = 0

        i = 0
        while i < len(inst_sorted):
            t = float(inst_sorted[i].arrive_gw_ms)

            # enqueue all arrivals at the same timestamp t
            while i < len(inst_sorted) and float(inst_sorted[i].arrive_gw_ms) == t:
                self._buffer.append(inst_sorted[i])
                i += 1


            while len(self._buffer) >= self.batch_size:
                batch_no += 1

                # FP select: pick best batch_size by priority
                self._buffer.sort(key=self._fp_key)
                batch = self._buffer[: self.batch_size]
                self._buffer = self._buffer[self.batch_size :]

                t_form_ms = t
                prio = self.assign_tsn_priority(batch)

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