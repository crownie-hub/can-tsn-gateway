# src/gateway/fp_timeout.py
# Fixed-Priority Timeout CAN-to-TSN gateway (FP-Timeout).
# All times in milliseconds (ms).
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional
from src.tsn.frame import TSNFrame


class FPTimeoutGateway:

    def __init__(
        self,
        batch_size:           int,
        gateway_period:       float,
        tsn_calc,
        *,
        default_priority:     int   = 0,
        batch_header_bytes:   int   = 0,
        per_can_header_bytes: int   = 0,
        start_offset:         float = 0.0,
    ):
        if batch_size <= 0:
            raise ValueError("batch_size must be > 0")
        if gateway_period <= 0:
            raise ValueError("gateway_period must be > 0")
        if not (0 <= int(default_priority) <= 7):
            raise ValueError("default_priority must be in 0..7")
        if start_offset < 0:
            raise ValueError("start_offset must be >= 0")

        self.batch_size           = int(batch_size)
        self.gateway_period       = float(gateway_period)
        self.tsn_calc             = tsn_calc
        self.default_priority     = int(default_priority)
        self.batch_header_bytes   = int(batch_header_bytes)
        self.per_can_header_bytes = int(per_can_header_bytes)
        self.start_offset         = float(start_offset)

        self.frames:  List[TSNFrame] = []
        self._buffer: List[Any]      = []

    # -- Helpers ----------------------------------------------------------

    @staticmethod
    def _msg_id(inst: Any) -> str:
        return f"m{inst.flow_id}_{inst.inst_id}"

    @staticmethod
    def _priority_value(inst: Any) -> int:
        prio = getattr(inst, "priority", None)
        if prio is None:
            can_id = getattr(inst, "can_id", None)
            prio   = int(can_id) if can_id is not None else int(inst.flow_id)
        return int(prio)

    @classmethod
    def _fp_key(cls, inst: Any):
        """
        1. CAN priority (smaller = higher priority).
        """
        return (
            cls._priority_value(inst),
            float(inst.arrive_gw),
            int(inst.flow_id),
            int(inst.inst_id),
        )

    def _next_release(self, t: float) -> float:
        """Smallest gateway release instant >= t."""
        if t <= self.start_offset:
            return self.start_offset + self.gateway_period
        k = math.ceil((t - self.start_offset) / self.gateway_period)
        return self.start_offset + k * self.gateway_period

    def _emit_batch(
        self,
        batch:       List[Any],
        batch_no:    int,
        t_form:      float,
        per_message: List[Dict[str, Any]],
        length:      Optional[float],
    ) -> None:
        frame = TSNFrame(
            frame_id=f"frame_{batch_no}",
            creation_time=t_form,
            priority=self.default_priority,
            batch_header_bytes=self.batch_header_bytes,
            per_can_header_bytes=self.per_can_header_bytes,
        )
        for m in batch:
            frame.add_instance(m)

        payload_bytes = frame.payload_bytes()
        wcrt_us       = float(
            self.tsn_calc.calculate_wcrt(payload_bytes, priority=frame.priority)
        )
        t_depart = t_form + wcrt_us / 1000.0

        frame.tx_start = t_form
        frame.tx_end   = t_depart
        self.frames.append(frame)

        for pos, m in enumerate(batch, start=1):
            arrive      = float(m.arrive_gw)
            enc_delay   = t_form - arrive
            total_delay = t_depart - arrive

            row: Dict[str, Any] = {
                "message_id":     self._msg_id(m),
                "flow_id":        m.flow_id,
                "inst_id":        m.inst_id,
                "can_id":         getattr(m, "can_id", None),
                "can_priority":   self._priority_value(m),
                "arrive_gw":      arrive,
                "batch_number":   batch_no,
                "position":       pos,
                "batch_size_actual": len(batch),
                "frame_id":       frame.frame_id,
                "frame_priority": frame.priority,
                "payload_bytes":  payload_bytes,
                "t_form":         t_form,
                "enc_delay":      enc_delay,
                "tsn_wcrt_us":    wcrt_us,
                "depart_tsn":     t_depart,
                "total_delay":    total_delay,
            }

            if length is None or arrive <= float(length):
                per_message.append(row)

    # -- Main simulation --------------------------------------------------

    def run_on_instances(
        self,
        instances: List[Any],
        *,
        length: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not instances:
            return {"num_batches": 0, "per_message": [], "frames": []}

        inst_sorted = sorted(
            instances,
            key=lambda x: (float(x.arrive_gw), int(x.flow_id), int(x.inst_id)),
        )

        self.frames  = []
        self._buffer = []
        per_message: List[Dict[str, Any]] = []
        batch_no = 0
        i        = 0
        n        = len(inst_sorted)

        t_release = self._next_release(float(inst_sorted[0].arrive_gw))

        while i < n or self._buffer:
            # Enqueue all arrivals up to this gateway release
            while i < n and float(inst_sorted[i].arrive_gw) <= t_release:
                self._buffer.append(inst_sorted[i])
                i += 1

            # Emit one frame per cycle if buffer is non-empty
            if self._buffer:
                self._buffer.sort(key=self._fp_key)
                batch        = self._buffer[: self.batch_size]
                self._buffer = self._buffer[self.batch_size :]
                batch_no    += 1
                self._emit_batch(batch, batch_no, t_release, per_message, length)

            t_release += self.gateway_period

        return {
            "num_batches": batch_no,
            "per_message": per_message,
            "frames":      self.frames,
        }
