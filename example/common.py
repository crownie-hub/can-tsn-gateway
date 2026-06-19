# example/common.py
import sys, os, math
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from src.can.message import message_set
from src.can.bus     import CANBusConfig, compute_tx_times, compute_response_times
from src.tsn.gcl     import GCL
from src.tsn.flow    import Flow
from src.tsn.wcrt    import TSNConfig, wcrt_ms
from src.gateways.decap import decap_delay


def build_setup(batch_size=5):
    can_cfg = CANBusConfig(bus_type="CAN", tbit=0.002)
    tsn_cfg = TSNConfig(link_speed_mbps=1000, num_switches=1,
                        switch_processing_us=3.0, propagation_delay_us=1.0)
    gcl     = GCL.sample_uniform(cycle_us=1000, window_us=200)

    msgset   = message_set()
    payloads = [m.payload_size for m in msgset]
    C = compute_tx_times(payloads, can_cfg)
    R = compute_response_times(
            [m.priority for m in msgset],
            [m.period   for m in msgset],
            payloads, can_cfg)

    wc_payload = 17 + sum(sorted(payloads, reverse=True)[:batch_size])
    tsn_wcrt   = wcrt_ms(Flow(0, min(m.period for m in msgset),
                              wc_payload, 0), [], gcl, tsn_cfg)
    c_can_max  = max(C)
    delta_dec  = decap_delay(batch_size, c_can_max)

    # R_dst: same bus config and message set as source for now
    R_dst = list(R)

    return dict(msgset=msgset, C=C, R=R, R_dst=R_dst,
                can_cfg=can_cfg, tsn_cfg=tsn_cfg, gcl=gcl,
                tsn_wcrt=tsn_wcrt, c_can_max=c_can_max,
                delta_dec=delta_dec, batch_size=batch_size)


# ── Printers ──────────────────────────────────────────────────────────────────

def print_can_rta(s):
    print(f"\n{'CAN RTA':=<62}")
    print(f"  {'msg':>5}  {'T':>7}  {'C':>7}  {'R':>7}  {'D':>7}  "
          f"{'jitter':>7}  {'ok':>4}")
    print(f"  {'-'*55}")
    for m, c, r in zip(s["msgset"], s["C"], s["R"]):
        ok = "ok" if r <= m.deadline else "no"
        print(f"  {m.msg_id:>5}  {m.period:>7.1f}  {c:>7.4f}  {r:>7.4f}  "
              f"{m.deadline:>7.1f}  {r-c:>7.4f}  {ok:>4}")
    print(f"  C_max={max(s['C']):.4f}  R_max={max(s['R']):.4f}  "
          f"jitter_max={max(r-c for r,c in zip(s['R'],s['C'])):.4f} ms")


def print_tsn(s):
    gcl_w = s["gcl"].window(0)
    print(f"\n{'TSN config':=<62}")
    print(f"  link     : {s['tsn_cfg'].link_speed_mbps} Mbps  "
          f"switches: {s['tsn_cfg'].num_switches}")
    print(f"  GCL      : cycle={gcl_w.cycle_us}us  "
          f"window={gcl_w.window_us}us  "
          f"closed={gcl_w.cycle_us - gcl_w.window_us}us")
    wc = 17 + sum(sorted([m.payload_size for m in s['msgset']],
                          reverse=True)[:s['batch_size']])
    print(f"  payload  : {wc} bytes  (17 hdr + {s['batch_size']} msgs)")
    print(f"  TSN WCRT : {s['tsn_wcrt']:.4f} ms")
    print(f"    = T_c {gcl_w.cycle_us/1000:.3f}ms"
          f" + C_j {s['tsn_cfg'].tx_time_us(wc)/1000:.4f}ms"
          f" + hop {s['tsn_cfg'].hop_delay_us()/1000:.4f}ms")


def print_gateway_components(label, delta_enc, tsn_wcrt, delta_dec, n):
    print(f"\n{'Gateway — ' + label:=<62}")
    print(f"  n={n}")
    print(f"  delta_enc  = {delta_enc:.4f} ms")
    print(f"  tsn_wcrt   = {tsn_wcrt:.4f} ms")
    c_max = delta_dec / (n - 1) if n > 1 else 0
    print(f"  delta_dec  = {delta_dec:.4f} ms  [(n-1)*C_max = {n-1}*{c_max:.4f}]")
    print(f"  total      = {delta_enc + tsn_wcrt + delta_dec:.4f} ms")


def print_enc_delay_trace(s, delta_enc, label="FIFO-BF"):
    """
    Show how delta_enc was computed — arrival function at key breakpoints.
    """
    from src.gateways.fifo_bf import arrival_function
    n   = s["batch_size"]
    R   = s["R"]
    C   = s["C"]
    msg = s["msgset"]

    print(f"\n{'Encapsulation delay trace — ' + label:=<62}")
    print(f"  Searching min tau s.t. A(tau) >= n={n}")
    print(f"  A(tau) = 1 + sum_x max(0, floor((tau - jitter_x) / T_x))")
    print(f"  jitter_x = R_x - C_x  (CAN response-time jitter)\n")

    # show A at tau=0 and at delta_enc
    A0 = arrival_function(0.0, msg, R, C)
    Ad = arrival_function(delta_enc, msg, R, C)
    print(f"  A(0)          = {A0}  (just m_i in buffer)")
    print(f"  A({delta_enc:.4f})  = {Ad}  >= n={n}  ← delta_enc")
    print(f"\n  Contribution of each message at tau={delta_enc:.4f} ms:")
    print(f"  {'msg':>5}  {'T':>7}  {'jitter':>7}  {'(tau-j)/T':>10}  "
          f"{'floor':>6}  {'contributes':>12}")
    print(f"  {'-'*55}")
    total = 0
    for m, r, c in zip(msg, R, C):
        jitter  = r - c
        raw     = (delta_enc - jitter) / m.period
        floored = max(0, math.floor(raw))
        total  += floored
        print(f"  {m.msg_id:>5}  {m.period:>7.1f}  {jitter:>7.4f}  "
              f"{raw:>10.4f}  {floored:>6}  {floored:>12}")
    print(f"  {'':>5}  {'':>7}  {'':>7}  {'':>10}  {'SUM':>6}  {total:>12}")
    print(f"  A(tau) = 1 + {total} = {1+total}")


def print_batch_trace(s, delta_enc, n_batches=3):
    """
    Show which messages fall into each batch based on worst-case arrive_gw.
    arrive_gw = release + R_i 
    Batches are groups of n sorted by arrive_gw.
    """
    from src.can.instance import build_instances
    n         = s["batch_size"]
    instances = build_instances(s["msgset"], s["R"],
                                horizon=max(m.period for m in s["msgset"]))

    print(f"\n{'Batch trace (worst-case arrivals)':=<62}")
    print(f"  arrive_gw = release + R_i  (worst-case CAN delay)")
    print(f"  Batch fires when {n} messages accumulate\n")

    batch_no = 1
    for start in range(0, min(len(instances), n * n_batches), n):
        batch = instances[start:start + n]
        if not batch:
            break
        t_form = max(inst.arrive_gw for inst in batch)
        print(f"  Batch {batch_no}  fires at t={t_form:.4f} ms:")
        for inst in batch:
            m      = s["msgset"][inst.flow_id]
            enc    = t_form - inst.arrive_gw
            print(f"    {m.msg_id:>5}  arrive={inst.arrive_gw:.4f}  "
                  f"release={inst.release:.4f}  R={inst.can_delay:.4f}  "
                  f"enc_wait={enc:.4f} ms")
        batch_no += 1



def print_zs_batch_trace(s, S, n_batches=3):
    """
    Simulate FIFO-ZS batch formation:
    - which messages were admitted into each batch
    - what L_j was (latest safe TX time)
    """
    import math
    from src.can.instance import build_instances

    n         = s["batch_size"]
    msgset    = s["msgset"]
    instances = build_instances(msgset, s["R"],
                                horizon=max(m.period for m in msgset))

    # build slack lookup by can_id
    slack_map = {int(m.msg_id, 16): si for m, si in zip(msgset, S)}

    print(f"\n{'FIFO-ZS Batch trace':=<62}")
    print(f"  Frame fires at L_j = min_{{m_x in F_j}}(a_x + S_x)")
    print(f"  Trigger: zero-slack (L_j reached) or buffer-full (|F_j|=={n})\n")

    F_j     = []
    L_j     = math.inf
    batch_no = 0
    printed  = 0

    def show_batch(F_j, L_j, trigger, t_fire):
        nonlocal printed
        if printed >= n_batches:
            return
        printed += 1
        trigger_str = f"{trigger}  (L_j={L_j:.4f})" if t_fire != L_j else trigger
        print(f"  Batch {printed}  |F_j|={len(F_j)}  "
              f"fires={t_fire:.4f} ms  [{trigger_str}]")
        print(f"    {'msg':>5}  {'arrive_gw':>10}  {'S_i':>8}  "
              f"{'a+S':>8}  {'enc_wait':>10}")
        print(f"    {'-'*50}")
        for inst, a in F_j:
            m       = msgset[inst.flow_id]
            si      = slack_map[inst.can_id]
            enc     = t_fire - a
            print(f"    {m.msg_id:>5}  {a:>10.4f}  {si:>8.4f}  "
                  f"{a+si:>8.4f}  {enc:>10.4f} ms")
        print()

    for inst in instances:
        a_q    = float(inst.arrive_gw)
        can_id = int(inst.can_id)
        S_i    = slack_map[can_id]

        L_new = min(L_j, a_q + S_i)

        # admit into current frame
        if a_q < L_new and len(F_j) < n:
            F_j.append((inst, a_q))
            L_j = L_new
        # admission fails
        else:
            # transmit current frame at its scheduled time
            if F_j:
                trigger = "buffer-full" if len(F_j) >= n else "zero-slack"
                show_batch(F_j, L_j, trigger, L_j)
            # initialize new frame with arriving message
            F_j = [(inst, a_q)]
            L_j = a_q + S_i
            # immediate transmission if slack already exhausted
            if L_j <= a_q:
                show_batch(F_j, L_j, "immediate", a_q)
                F_j = []
                L_j = math.inf
                continue

        # buffer-full trigger
        if len(F_j) == n:
            show_batch(F_j, L_j, "buffer-full", L_j)
            F_j = []
            L_j = math.inf

        if printed >= n_batches:
            break

        if printed >= n_batches:
            break

    if F_j and printed < n_batches:
        show_batch(F_j, L_j, "end-of-sim", L_j)


def print_zs_ap_trace(s, S, frames, n_show=10):

    import math

    msgset = s["msgset"]

    slack_map = {
        int(m.msg_id, 16): si
        for m, si in zip(msgset, S)
    }

    print(f"\n{'FIFO-ZS-AP Batch Trace':=<65}")
    print(f"  FIFO-ZS batching with optional early transmission by prediction")
    print(f"  AP = transmitted earlier than L_j due to prediction")
    print(f"  ZS = transmitted at L_j")
    print(f"  BF = transmitted because buffer became full\n")

    printed = 0

    for f in frames:

        if printed >= n_show:
            break

        printed += 1

        t_fire = f["t_form"]
        L_j = f["L_j"]
        trigger = f["trigger"]

        msgs = [msgset[inst.flow_id] for inst, _ in f["messages"]]
        arrivals = [a for _, a in f["messages"]]

        if trigger == "prediction":

            tag = "←AP"

            note = (
                f"predicted_next={f['predicted_next']:.4f} >= L_j"
                if f.get("predicted_next") is not None
                else "prediction-triggered"
            )

        elif trigger == "zero-slack":

            tag = " ZS"
            note = "FIFO-ZS latest safe transmit time reached"

        elif trigger == "buffer-full":

            tag = " BF"
            note = "FIFO-ZS buffer-full trigger"

        elif trigger == "immediate":

            tag = "IMM"
            note = "S_i <= 0 at arrival"

        else:

            tag = "  -"
            note = trigger

        print(
            f"  Batch {printed}  [{tag}]"
            f"  size={f['size']}"
            f"  fires={t_fire:.4f}"
            f"  L_j={L_j:.4f}"
            f"  enc={f['enc_delay']:.4f} ms"
        )

        print(f"  note: {note}")

        print(
            f"    {'msg':>5}"
            f"  {'arrive_gw':>10}"
            f"  {'S_i':>8}"
            f"  {'enc_wait':>10}"
        )

        print(f"  {'-'*42}")

        for m, a in zip(msgs, arrivals):

            si = slack_map[int(m.msg_id, 16)]
            enc = t_fire - a

            print(
                f"    {m.msg_id:>5}"
                f"  {a:>10.4f}"
                f"  {si:>8.4f}"
                f"  {enc:>10.4f} ms"
            )

        print()

def print_zs_ap_e2e(s, S, frames, tsn_wcrt, delta_dec, R_dst=None):
    """
    Per-message worst-case e2e from ZS-AP simulation.
    Uses the worst observed enc_delay per message across all frames.
    e2e_i = R_src_i + max_enc_i + tsn_wcrt + delta_dec + R_dst_i
    """
    msgset  = s["msgset"]
    R_src   = s["R"]
    R_dst   = R_dst or [0.0] * len(msgset)

    # worst enc_delay per flow_id
    worst_enc = {}
    for f in frames:
        for inst, a in f["messages"]:
            fid = inst.flow_id
            enc = f["t_form"] - a
            worst_enc[fid] = max(worst_enc.get(fid, 0.0), enc)

    print(f"\n{'FIFO-ZS-AP E2E WCRT (simulated worst-case)':=<65}")
    print(f"  e2e = R_src + max_enc_observed + tsn_wcrt + delta_dec + R_dst - C")
    print(f"  max_enc_observed: worst enc_delay seen per message across all instances\n")
    print(f"  {'msg':>5}  {'R_src':>7}  {'max_enc':>8}  {'tsn':>7}  "
          f"{'d_dec':>7}  {'R_dst':>7}  {'e2e':>8}  {'D':>7}  {'ok':>4}")
    print(f"  {'-'*70}")

    n_ok = 0
    C_src = s.get('C', [0.0]*len(msgset))
    for i, (m, r, c, rd) in enumerate(zip(msgset, R_src, C_src, R_dst)):
        enc      = worst_enc.get(i, 0.0)
        e2e      = r + enc + tsn_wcrt + delta_dec + rd - c
        EPS = 1e-9
        feasible = e2e <= (m.deadline + EPS)
        if feasible: n_ok += 1
        ok = "ok" if feasible else "no"
        print(f"  {m.msg_id:>5}  {r:>7.4f}  {enc:>8.4f}  {tsn_wcrt:>7.4f}  "
              f"{delta_dec:>7.4f}  {rd:>7.4f}  {e2e:>8.4f}  "
              f"{m.deadline:>7.1f}  {ok:>4}")

    all_e2e = [R_src[i] + worst_enc.get(i,0) + tsn_wcrt + delta_dec
               + R_dst[i] - C_src[i]
               for i in range(len(msgset))]
    print(f"\n  {n_ok}/{len(msgset)} feasible   "
          f"worst={max(all_e2e):.4f}  best={min(all_e2e):.4f} ms")

def print_slack_trace(s, S):
    """
    Show how each slack S_i was computed and which message triggered tau_ZS.
    S_i = D_i - (R_i^src + tsn_wcrt + delta_dec + R_i^dst - C_i)
    delta_enc is NOT included — S_i budgets for it.
    """
    print(f"\n{'Slack computation trace':=<62}")
    print(f"  S_i = D_i - (R_src + tsn_wcrt + delta_dec + R_dst - C_i)")
    print(f"  tsn_wcrt={s['tsn_wcrt']:.4f}  delta_dec={s['delta_dec']:.4f}\n")

    has_rdst = "R_dst" in s and any(r != 0 for r in s["R_dst"])
    hdr = f"  {'msg':>5}  {'D':>7}  {'R_src':>7}  {'C':>7}  {'R_dst':>7}  {'total':>8}  {'S_i':>8}"
    print(hdr)
    print(f"  {'-'*60}")

    min_s   = min(S)
    min_msg = s["msgset"][[i for i, si in enumerate(S) if si == min_s][0]].msg_id
    R_dst   = s.get("R_dst", [0.0]*len(s["msgset"]))

    for m, r, c, rd, si in zip(s["msgset"], s["R"], s["C"], R_dst, S):
        total = r + s["tsn_wcrt"] + s["delta_dec"] + rd - c
        sign  = "tau_ZS" if m.msg_id == min_msg else ""
        print(f"  {m.msg_id:>5}  {m.deadline:>7.1f}  {r:>7.4f}  {c:>7.4f}  "
              f"{rd:>7.4f}  {total:>8.4f}  {si:>8.4f}{sign}")

    tau_zs = max(0.0, min_s)
    print(f"\n  min S_i = {min_s:.4f} ms  (msg {min_msg})")
    print(f"  tau_ZS  = max(0, {min_s:.4f}) = {tau_zs:.4f} ms")


def print_e2e_table(rows, show_breakdown=True):
    print(f"\n{'E2E WCRT per message':=<62}")
    has_slack     = "slack" in rows[0]
    has_breakdown = show_breakdown and "delta_enc" in rows[0]
    has_rdst      = "R_dst" in rows[0] and any(r["R_dst"] != 0.0 for r in rows)

    if has_breakdown:
        formula = "R_src + delta_enc + tsn_wcrt + delta_dec - C"
        if has_rdst:
            formula += " + R_dst"
        print(f"  e2e = {formula}")
        hdr = (f"  {'msg':>5}  {'R_src':>7}  {'d_enc':>7}  {'tsn':>7}  {'d_dec':>7}")
        if has_rdst:
            hdr += f"  {'R_dst':>7}"
        hdr += f"  {'e2e':>8}  {'D':>7}"
    else:
        hdr = f"  {'msg':>5}  {'T':>7}  {'R_src':>7}  {'e2e':>8}  {'D':>7}"
    if has_slack:
        hdr += f"  {'slack':>8}"
    hdr += f"  {'ok':>4}"
    print(hdr)
    print(f"  {'-'*65}")

    for r in rows:
        ok = "ok" if r["feasible"] else "no"
        if has_breakdown:
            line = (f"  {r['msg_id']:>5}  {r['R_src']:>7.4f}  "
                    f"{r['delta_enc']:>7.4f}  {r['tsn_wcrt']:>7.4f}  "
                    f"{r['delta_dec']:>7.4f}")
            if has_rdst:
                line += f"  {r['R_dst']:>7.4f}"
            line += f"  {r['e2e']:>8.4f}  {r['deadline']:>7.1f}"
        else:
            line = (f"  {r['msg_id']:>5}  {r['period']:>7.1f}  "
                    f"{r['R_src']:>7.4f}  {r['e2e']:>8.4f}  {r['deadline']:>7.1f}")
        if has_slack:
            line += f"  {r['slack']:>8.4f}"
        line += f"  {ok}"
        print(line)

    n_ok = sum(1 for r in rows if r["feasible"])
    print(f"\n  {n_ok}/{len(rows)} feasible   "
          f"worst={max(r['e2e'] for r in rows):.4f}  "
          f"best={min(r['e2e'] for r in rows):.4f} ms")