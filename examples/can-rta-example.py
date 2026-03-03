import math
from dataclasses import dataclass
#tesing

def _blocking_time_ms(prio, C, k):

    bk = 0.0
    for i in range(len(prio)):
        if prio[i] > prio[k]:  # lower priority (numerically larger)
            bk = max(bk, C[i])
    return bk


def _hp_interference_ms(prio, T, C, k, w, jitter_ms=0.0):
    s = 0.0
    for i in range(len(prio)):
        if prio[i] < prio[k]:  # higher priority
            s += math.ceil((w + jitter_ms) / T[i]) * C[i]
    return s


@dataclass
class RTAConfig:
    eps: float = 1e-12
    max_iter: int = 10_000
    jitter_ms: float = 0.0  

def response_time_rta_ms(prio, periods_ms, C_ms, k, cfg: RTAConfig):
    Bk = _blocking_time_ms(prio, C_ms, k)

    w_prev = None
    w = Bk + C_ms[k]  # initial guess

    for _ in range(cfg.max_iter):
        if w_prev is not None and abs(w - w_prev) <= cfg.eps:
            return w

        w_prev = w
        w = Bk + C_ms[k] + _hp_interference_ms(prio, periods_ms, C_ms, k, w_prev, cfg.jitter_ms)

        # Helpful guard if something goes numerically wrong
        if not math.isfinite(w):
            raise RuntimeError("RTA diverged (w became non-finite). Check  utilization.")

    raise RuntimeError("CAN RTA did not converge within max_iter")



@dataclass
class Msg:
    name: str
    sender_hex: str   
    D_ms: float
    P_ms: float
    C_ms: float       # CDi (ms)


msgs = [
    Msg("VC_A0",     "A0",    5,    5,    0.52),
    Msg("VC_B0",     "B0",   10,   10,    0.92),
    Msg("VC_D0",     "D0", 1000, 1000,    0.52),

    Msg("Brakes_A1", "A1",    5,    5,    0.60),
    Msg("Brakes_C1", "C1",  100,  100,    0.52),

    Msg("Battery_B2","B2",   10,   10,    0.52),
    Msg("Battery_C2","C2",  100,  100,    0.76),
    Msg("Battery_D2","D2", 1000, 1000,    0.68),

    Msg("Driver_A3", "A3",    5,    5,    0.52),
    Msg("Driver_B3", "B3",   10,   10,    0.60),

    Msg("IMC_A4",    "A4",    5,    5,    0.60),
    Msg("IMC_B4",    "B4",   10,   10,    0.60),

    Msg("Trans_A5",  "A5",    5,    5,    0.52),
    Msg("Trans_C5",  "C5",  100,  100,    0.52),
    Msg("Trans_D5",  "D5", 1000, 1000,    0.52),
]

msgs = sorted(msgs, key=lambda m: int(m.sender_hex, 16))

priorities = [int(m.sender_hex, 16) for m in msgs]  # smaller => higher priority
periods_ms  = [m.P_ms for m in msgs]
C_ms        = [m.C_ms for m in msgs]
deadlines   = [m.D_ms for m in msgs]

cfg = RTAConfig(jitter_ms=0.0)

R_ms = []
for k in range(len(msgs)):
    R_ms.append(response_time_rta_ms(priorities, periods_ms, C_ms, k, cfg))

print("idx  name         ID   prio(ID)  P(ms)   C(ms)   D(ms)    R(ms)   OK?")
for k, m in enumerate(msgs):
    ok = R_ms[k] <= m.D_ms
    print(f"{k:>3}  {m.name:<11} {m.sender_hex:>2}   {priorities[k]:>7}  "
          f"{m.P_ms:>6.1f}  {m.C_ms:>6.2f}  {m.D_ms:>6.1f}  {R_ms[k]:>7.2f}   {ok}")


releases = [
    (0.0,  0), 
    (0.0,  3),
    (5.0,  9),
]

finishes = [(r + R_ms[sid], sid) for r, sid in releases]

print("\nFinish times (finish_ms, stream_idx, stream_name):")
for f, sid in finishes:
    print((f, sid, msgs[sid].name))