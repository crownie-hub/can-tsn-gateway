# src/gateways/predictive_daa_gateway.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Literal

from src.tsn_frame import TSNFrame


DstModel = Literal["none", "tx"]


def _msg_id(inst: Any) -> str:
    return f"m{inst.flow_id}_{inst.inst_id}"


def _can_prio(inst: Any) -> int:
    # smaller CAN-ID => higher priority
    return int(inst.can_id)


@dataclass
class Admitted:
    inst: Any
    a_ms: float
    can_prio: int
    base_latest_start_ms: float  # a_i + delta_i^{eps,max}


class PredictiveDAAGateway:
    """
    Predictive DAA gateway (no eviction)
      dst_model="none": destination handled only by CAN WCRT R_ms
      dst_model="tx"  : apply incremental destination-side tightening using CAN tx times C_ms
    """
    def __init__(
        self,
        tsn_calc,  # TSNWCRTCalculator
        *,
        decap_delay_ms: float,
        R_ms: List[float],             # destination CAN WCRT per flow
        C_ms: Optional[List[float]] = None,  # CAN tx times per flow (needed if dst_model="tx")
        dst_model: DstModel = "none",
        default_tsn_priority: int = 0,
        batch_header_bytes: int = 0,
        per_can_header_bytes: int = 0,
    ):
        self.tsn_calc = tsn_calc
        self.decap_delay_ms = float(decap_delay_ms)
        self.R_ms = [float(x) for x in R_ms]

        self.dst_model: DstModel = dst_model
        self.C_ms = [float(x) for x in C_ms] if C_ms is not None else None
        if self.dst_model == "tx" and not self.C_ms:
            raise ValueError("dst_model='tx' requires C_ms (CAN transmission times per flow).")

        self.default_tsn_priority = int(default_tsn_priority)
        self.batch_header_bytes = int(batch_header_bytes)
        self.per_can_header_bytes = int(per_can_header_bytes)

        self.frames: List[TSNFrame] = []

    def _tsn_wcrt_ms(self, payload_bytes: int, prio: int) -> float:
        wcrt_us = float(self.tsn_calc.calculate_wcrt(payload_bytes, priority=prio))
        return wcrt_us / 1000.0

    def _delta_eps_max_ms(self, inst: Any, *, payload_bytes: int, prio: int) -> float:
        """
        delta_i^{eps,max} = deadline - (tsn + decap + destCAN_WCRT) # decap
        """
        d_ms = float(inst.deadline)
        flow = int(inst.flow_id)
        tsn_ms = self._tsn_wcrt_ms(payload_bytes, prio)
        dest_ms = float(self.R_ms[flow])
        return d_ms - (tsn_ms + self.decap_delay_ms + dest_ms)

    def _effective_close_time(
        self,
        admitted: List[Admitted],
        *,
        cum_delta_c_ms: float,
        dst_inc_ms: Dict[str, float],
    ) -> float:
        best = float("inf")
        for a in admitted:
            mid = _msg_id(a.inst)
            val = a.base_latest_start_ms - cum_delta_c_ms - float(dst_inc_ms.get(mid, 0.0))
            if val < best:
                best = val
        return best

    def _dst_inc_after_admit(
        self,
        admitted: List[Admitted],
        cand: Admitted,
        dst_inc_ms: Dict[str, float],
    ) -> Dict[str, float]:
        """
        Eq- tightening: admitting a higher-priority candidate can add up to one
        CAN tx time of candidate to each lower-priority admitted message.
        """
        if self.dst_model != "tx":
            return dst_inc_ms  # no dst tightening

        assert self.C_ms is not None
        cand_tx_ms = float(self.C_ms[int(cand.inst.flow_id)])

        new = dict(dst_inc_ms)
        for a in admitted:
            if a.can_prio > cand.can_prio:
                mid = _msg_id(a.inst)
                new[mid] = float(new.get(mid, 0.0)) + cand_tx_ms
        return new

    def _next_higher_prio_arrival_within(
        self,
        inst_sorted: List[Any],
        start_idx: int,
        *,
        horizon_ms: float,
        cand_can_prio: int,
    ) -> Optional[float]:
        for k in range(start_idx, len(inst_sorted)):
            nxt = inst_sorted[k]
            a = float(nxt.arrive_gw_ms)
            if a > horizon_ms:
                return None
            if _can_prio(nxt) < cand_can_prio:
                return a
        return None

    def run_on_instances(self, instances: List[Any], *, length: Optional[float] = None) -> Dict[str, Any]:
        if not instances:
            return {"num_frames": 0, "per_message": [], "frames": []}

        inst_sorted = sorted(instances, key=lambda x: float(x.arrive_gw_ms))

        self.frames = []
        per_message: List[Dict[str, Any]] = []

        idx = 0
        frame_no = 0

        while idx < len(inst_sorted):
            seed = inst_sorted[idx]
            idx += 1

            t_ms = float(seed.arrive_gw_ms)

            frame_no += 1
            frame = TSNFrame(
                frame_id=f"frame_{frame_no}",
                creation_time_ms=t_ms,
                priority=self.default_tsn_priority,
                batch_header_bytes=self.batch_header_bytes,
                per_can_header_bytes=self.per_can_header_bytes,
            )

            admitted: List[Admitted] = []
            dst_inc_ms: Dict[str, float] = {}
            cum_delta_c_ms = 0.0

            # admit seed
            frame.add_instance(seed)
            payload = frame.payload_bytes()
            eps = self._delta_eps_max_ms(seed, payload_bytes=payload, prio=frame.priority)
            admitted.append(
                Admitted(
                    inst=seed,
                    a_ms=t_ms,
                    can_prio=_can_prio(seed),
                    base_latest_start_ms=t_ms + eps,
                )
            )

            hat_L = self._effective_close_time(admitted, cum_delta_c_ms=cum_delta_c_ms, dst_inc_ms=dst_inc_ms)

            # predictive loop
            while idx < len(inst_sorted):
                cand_inst = inst_sorted[idx]
                a_cand = float(cand_inst.arrive_gw_ms)

                # must arrive within the remaining slack
                if a_cand > hat_L:
                    break

                # wait until it arrives
                t_ms = a_cand

                cand = Admitted(
                    inst=cand_inst,
                    a_ms=a_cand,
                    can_prio=_can_prio(cand_inst),
                    base_latest_start_ms=0.0,
                )

                # tentative add
                frame.add_instance(cand_inst)
                new_payload = frame.payload_bytes()

                # TSN payload-growth tightening (or use TSN WCRT delta )
                old_tsn = self._tsn_wcrt_ms(payload, frame.priority)
                new_tsn = self._tsn_wcrt_ms(new_payload, frame.priority)
                delta_c_ms = max(0.0, new_tsn - old_tsn)
                new_cum = cum_delta_c_ms + delta_c_ms

                # destination-side tightening (optional)
                new_dst = self._dst_inc_after_admit(admitted, cand, dst_inc_ms)

                # candidate latest start under new payload
                cand_eps = self._delta_eps_max_ms(cand_inst, payload_bytes=new_payload, prio=frame.priority)
                cand.base_latest_start_ms = a_cand + cand_eps

                admitted_plus = admitted + [cand]
                hat_L_if = self._effective_close_time(admitted_plus, cum_delta_c_ms=new_cum, dst_inc_ms=new_dst)

                # priority lookahead guard (Eq. 43 idea)
                a_h = self._next_higher_prio_arrival_within(
                    inst_sorted, idx + 1, horizon_ms=hat_L, cand_can_prio=cand.can_prio
                )
                if a_h is not None and a_h > hat_L_if:
                    frame.instances.pop()
                    break

                # predictive admissibility
                if a_cand > hat_L_if:
                    frame.instances.pop()
                    break

                # commit
                admitted = admitted_plus
                dst_inc_ms = new_dst
                cum_delta_c_ms = new_cum
                payload = new_payload
                hat_L = hat_L_if
                idx += 1

            # transmit
            t_form = float(t_ms)
            payload = frame.payload_bytes()
            tsn_ms = self._tsn_wcrt_ms(payload, frame.priority)
            t_depart = t_form + tsn_ms

            frame.tx_start_time_ms = t_form
            frame.tx_end_time_ms = t_depart
            self.frames.append(frame)

            for pos, m in enumerate(frame.instances, start=1):
                arrive = float(m.arrive_gw_ms)
                gw_tsn = t_depart - arrive

                row = {
                    "message_id": _msg_id(m),
                    "flow_id": int(m.flow_id),
                    "inst_id": int(m.inst_id),
                    "can_id": int(m.can_id),

                    "arrive_gw_ms": arrive,
                    "frame_id": frame.frame_id,
                    "frame_priority": int(frame.priority),
                    "frame_payload_bytes": int(payload),

                    "tx_start_ms": t_form,
                    "tsn_wcrt_ms": tsn_ms,
                    "depart_tsn_ms": t_depart,

                    "position": pos,
                    "total_delay_ms": gw_tsn,  # gateway wait + TSN
                }

                if length is None or arrive <= float(length):
                    per_message.append(row)

        return {"num_frames": frame_no, "per_message": per_message, "frames": self.frames}