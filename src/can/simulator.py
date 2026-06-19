# src/can/simulator.py
# CAN / CAN-FD bus simulator.
# All times in milliseconds (ms).
#


import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional

from .bus import CANBusConfig, compute_tx_times


@dataclass
class MessageConfig:
    msg_id:        int
    period:        float
    payload_bytes: int
    deadline:      Optional[float] = None
    start_offset:  float           = 0.0
    name:          Optional[str]   = None

    def __post_init__(self):
        if self.deadline is None:
            self.deadline = float(self.period)


@dataclass
class NodeConfig:
    node_id:   int
    node_name: str
    messages:  List[MessageConfig]


@dataclass
class FrameInstance:
    node_id:       int
    node_name:     str
    msg_index:     int
    msg_id:        int
    msg_name:      str
    payload_bytes: int
    period:        float
    deadline:      Optional[float]
    seq:           int
    release:       float
    tx_time:       float
    start_tx:      Optional[float] = None
    finish_tx:     Optional[float] = None

    @property
    def waiting_time(self):
        return None if self.start_tx is None else self.start_tx - self.release

    @property
    def response_time(self):
        return None if self.finish_tx is None else self.finish_tx - self.release

    @property
    def missed_deadline(self):
        if self.deadline is None or self.finish_tx is None:
            return None
        return self.response_time > self.deadline


class CANBusSimulator:
    """
    Event-driven CAN / CAN-FD bus simulator.
    """

    def __init__(self, nodes, cfg, sim_duration):
        self.nodes        = nodes
        self.cfg          = cfg
        self.sim_duration = float(sim_duration)

        self._flat    = []
        self._C       = []
        self._periods = []
        self._prios   = []

        for node in nodes:
            for msg in node.messages:
                self._flat.append((node, msg))
                self._C.append(0.0)   # filled below
                self._periods.append(float(msg.period))
                self._prios.append(int(msg.msg_id))

        payloads  = [int(node_msg[1].payload_bytes) for node_msg in self._flat]
        self._C   = compute_tx_times(payloads, cfg)

        self._events:   List = []
        self._ready:    List = []
        self.completed: List[FrameInstance] = []
        self.busy       = False
        self.now        = 0.0
        self._seq: Dict[int, int] = {i: 0 for i in range(len(self._flat))}

        for idx, (_, msg) in enumerate(self._flat):
            t0 = float(msg.start_offset)
            if t0 <= self.sim_duration:
                heapq.heappush(self._events, (t0, 0, idx, idx))

    def _handle_release(self, idx):
        node, msg = self._flat[idx]
        seq = self._seq[idx]
        self._seq[idx] += 1

        frame = FrameInstance(
            node_id=node.node_id,
            node_name=node.node_name,
            msg_index=idx,
            msg_id=int(msg.msg_id),
            msg_name=msg.name or f"0x{msg.msg_id:X}",
            payload_bytes=int(msg.payload_bytes),
            period=float(msg.period),
            deadline=float(msg.deadline),
            seq=seq,
            release=self.now,
            tx_time=float(self._C[idx]),
        )
        heapq.heappush(self._ready, (frame.msg_id, frame.release, frame.seq, frame))

        next_t = self.now + float(msg.period)
        if next_t <= self.sim_duration:
            heapq.heappush(self._events, (next_t, 0, idx, idx))

    def _try_start_tx(self):
        if self.busy or not self._ready:
            return
        _, _, _, frame = heapq.heappop(self._ready)
        frame.start_tx  = self.now
        frame.finish_tx = self.now + frame.tx_time
        self.busy       = True
        heapq.heappush(self._events,
                       (frame.finish_tx, 1, frame.msg_index, frame))

    def _handle_tx_done(self, frame):
        self.completed.append(frame)
        self.busy = False

    def run(self):
        while self._events:
            t, etype, _, payload = heapq.heappop(self._events)
            if t > self.sim_duration:
                break
            self.now = float(t)
            if etype == 0:
                self._handle_release(int(payload))
                self._try_start_tx()
            else:
                self._handle_tx_done(payload)
                self._try_start_tx()
        return self.completed

    def summary(self):
        print(f"\nCAN Simulation  [{self.cfg.bus_type}  {self.sim_duration:.1f} ms]")
        print(f"  {len(self.completed)} frames completed")

        by_msg: Dict[int, list] = {}
        for f in self.completed:
            by_msg.setdefault(f.msg_index, []).append(f)

        print(f"  {'idx':>4}  {'node':<10}  {'msg':<8}  "
              f"{'n':>5}  {'C':>8}  {'maxW':>8}  {'maxR':>8}  {'miss':>5}")
        print("  " + "-" * 65)
        for idx, frames in sorted(by_msg.items()):
            node, msg = self._flat[idx]
            waits  = [f.waiting_time  for f in frames if f.waiting_time  is not None]
            rts    = [f.response_time for f in frames if f.response_time is not None]
            misses = sum(1 for f in frames if f.missed_deadline)
            print(f"  {idx:>4}  {node.node_name:<10}  "
                  f"{(msg.name or hex(msg.msg_id)):<8}  "
                  f"{len(frames):>5}  {self._C[idx]:>8.4f}  "
                  f"{max(waits) if waits else 0.0:>8.4f}  "
                  f"{max(rts)   if rts   else 0.0:>8.4f}  "
                  f"{misses:>5}")
