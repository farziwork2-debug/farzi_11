#!/usr/bin/env python3
"""
myact_udp.py - laptop side of the simulation/laptop -> motors bridge for myact_ctl.

Board side (3 motors, the laptop chooses mode and targets live):
    sudo ./myact_ctl --mode udp --expect 3 --no-dc --max-step 20 --limit-deg 30 --tq-limit 50,50,50

Library use:

    from myact_udp import MyActBridge, HOLD, POS, VEL, TORQUE
    bridge = MyActBridge("192.168.50.2", n_joints=3)
    bridge.send(q=[0, 0, 0])                                   # position mode (default), angles in rad
    bridge.send(mode=[VEL, HOLD, POS], q=q, v=[0.5, 0, 0])     # per motor: velocity, hold, position
    bridge.send(mode=TORQUE, tau_pct=[3, 0, 0])                # torque mode, % of rated torque
    bridge.send(mode=TORQUE, tau_nm=tau_nm, rated_nm=60.0)     # torque mode, N*m (X10-200 rated = 60 N*m)
    st = bridge.poll()                                         # latest feedback or None

Units: q in rad (relative to the pose at enable), v in rad/s, tau in % of rated torque.
Packets must keep flowing (>= 20/s): the board stops all motors if none arrives for --udp-timeout-ms.

Packet format (little endian), identical on both sides:
    request : u32 magic 'MYA2', u32 seq, u32 n, u8 mode[16], f64 q[16], f64 v[16], f64 tau[16]   (412 bytes)
    reply   : u32 magic, u32 seq, u32 n, f64 q[16], f64 qd[16], f64 torque_pct[16],
              u16 statusword[16], u8 phase[16], u8 mode[16]                                       (460 bytes)
"""
import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Union

MAGIC = 0x3241594D          # bytes 'M' 'Y' 'A' '2'
MAX_AX = 16
REQ = struct.Struct("<III16B16d16d16d")
REP = struct.Struct("<III16d16d16d16H16B16B")
PHASES = ["WAIT_OP", "MONITOR", "PREPARE", "ENABLING", "RUN", "STOPPING", "DONE", "FAULT"]

HOLD, POS, VEL, TORQUE = 0, 1, 2, 3
MODE_NAMES = {0: "HOLD", 1: "POS", 2: "VEL", 3: "TORQUE", 255: "SWITCHING"}
MODE_BY_NAME = {"hold": HOLD, "pos": POS, "position": POS, "vel": VEL, "velocity": VEL,
                "torque": TORQUE, "tq": TORQUE}


@dataclass
class MotorState:
    seq: int
    q: List[float]            # joint angle, rad, same convention as the commands (NaN until the board aligned)
    qd: List[float]           # rad/s
    torque_pct: List[float]   # % of rated torque
    statusword: List[int]
    phase: List[str]          # WAIT_OP / PREPARE / ENABLING / RUN / STOPPING / DONE / FAULT
    mode: List[str]           # active mode per motor; SWITCHING while the drive changes mode


class MyActBridge:
    def __init__(self, host: str, n_joints: int, port: int = 5005):
        if not 1 <= n_joints <= MAX_AX:
            raise ValueError("n_joints must be 1..16")
        self.n = n_joints
        self.addr = (host, port)
        self.seq = 0
        self.tx = 0              # packets sent
        self.rx = 0              # valid replies received
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def _vec(self, x, default, name):
        if x is None:
            x = default
        if isinstance(x, (int, float)):
            x = [x] * self.n
        x = list(x)
        if len(x) != self.n:
            raise ValueError(f"{name}: expected {self.n} values, got {len(x)}")
        return x

    def send(self, q=None, mode: Union[int, Sequence[int], None] = None, v=None, tau_pct=None,
             tau_nm=None, rated_nm=None) -> None:
        """q [rad], v [rad/s], torque as tau_pct [% of rated] or tau_nm [N*m] together with rated_nm
        [N*m rated torque of each motor, e.g. 60 for the X10-200], mode: HOLD/POS/VEL/TORQUE
        (one for all or one per motor)."""
        qv = [float(x) for x in self._vec(q, 0.0, "q")]
        vv = [float(x) for x in self._vec(v, 0.0, "v")]
        if tau_nm is not None:
            if tau_pct is not None:
                raise ValueError("give tau_pct or tau_nm, not both")
            if rated_nm is None:
                raise ValueError("tau_nm needs rated_nm (rated torque of the motor in N*m)")
            nm = [float(x) for x in self._vec(tau_nm, 0.0, "tau_nm")]
            rt = [float(x) for x in self._vec(rated_nm, 0.0, "rated_nm")]
            if any(r <= 0 for r in rt):
                raise ValueError("rated_nm must be > 0")
            tau_pct = [100.0 * t / r for t, r in zip(nm, rt)]
        tv = [float(x) for x in self._vec(tau_pct, 0.0, "tau_pct")]
        mv = [int(x) for x in self._vec(mode, POS, "mode")]
        if not all(math.isfinite(x) for x in qv + vv + tv):
            raise ValueError("non-finite value, not sending")
        if any(m not in (HOLD, POS, VEL, TORQUE) for m in mv):
            raise ValueError("mode must be HOLD, POS, VEL or TORQUE")
        pad = MAX_AX - self.n
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.sock.sendto(REQ.pack(MAGIC, self.seq, self.n, *(mv + [0] * pad),
                                  *(qv + [0.0] * pad), *(vv + [0.0] * pad), *(tv + [0.0] * pad)), self.addr)
        self.tx += 1

    def poll(self) -> Optional[MotorState]:
        """Drain the socket and return the newest reply (None if nothing arrived)."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            if len(data) != REP.size:
                continue
            f = REP.unpack(data)
            if f[0] != MAGIC:
                continue
            latest = f
            self.rx += 1
        if latest is None:
            return None
        n = self.n
        return MotorState(
            seq=latest[1],
            q=list(latest[3:3 + n]),
            qd=list(latest[19:19 + n]),
            torque_pct=list(latest[35:35 + n]),
            statusword=list(latest[51:51 + n]),
            phase=[PHASES[p] if p < len(PHASES) else str(p) for p in latest[67:67 + n]],
            mode=[MODE_NAMES.get(m, str(m)) for m in latest[83:83 + n]],
        )


def _lst(text, n, conv):
    vals = [conv(x) for x in text.split(",") if x.strip() != ""]
    if len(vals) == 1:
        vals = vals * n
    if len(vals) != n:
        raise SystemExit(f"need 1 value or {n} values (one per motor), got: {text}")
    return vals


def _demo():
    ap = argparse.ArgumentParser(description="drive the motors from the laptop, any mode per motor")
    ap.add_argument("--host", required=True, help="IP of the Cerebellum board")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--n", type=int, default=1, help="number of motors (must match the board)")
    ap.add_argument("--modes", default="pos", help="hold|pos|vel|torque, one for all or comma list per motor")
    ap.add_argument("--amp-deg", default="10", help="position mode: sine amplitude, deg (list per motor ok)")
    ap.add_argument("--freq", type=float, default=0.2, help="position mode: Hz")
    ap.add_argument("--vel", default="0.5", help="velocity mode: rad/s (list per motor ok)")
    ap.add_argument("--tau", default="2", help="torque mode: %% of rated torque (list per motor ok)")
    ap.add_argument("--tau-sine", type=float, default=0.0, metavar="HZ",
                    help="torque mode: alternate the torque as a sine of this frequency (keeps the shaft near its start, safest way to test an unloaded motor)")
    ap.add_argument("--duration", type=float, default=15.0, help="seconds")
    ap.add_argument("--rate", type=float, default=200.0, help="packets per second")
    ap.add_argument("--ramp", type=float, default=2.0, help="seconds to fade the command in (position, speed and torque)")
    ap.add_argument("--sequence", action="store_true",
                    help="demo: all motors hold -> position -> hold -> velocity -> hold -> torque -> hold")
    a = ap.parse_args()

    n = a.n
    modes = _lst(a.modes, n, lambda s: MODE_BY_NAME[s.strip().lower()])
    amp = [math.radians(x) for x in _lst(a.amp_deg, n, float)]
    vel = _lst(a.vel, n, float)
    tau = _lst(a.tau, n, float)
    # (name, mode, seconds) for --sequence
    seq = [("hold", HOLD, 2), ("position", POS, 6), ("hold", HOLD, 1.5), ("velocity", VEL, 4),
           ("hold", HOLD, 1.5), ("torque", TORQUE, 3), ("hold", HOLD, 2)]
    if a.sequence:
        a.duration = sum(s[2] for s in seq)

    br = MyActBridge(a.host, n, a.port)
    t0 = time.perf_counter()
    nxt = t0
    last_print = -1.0
    pos_t0 = None
    while True:
        now = time.perf_counter()
        t = now - t0
        if t > a.duration:
            break
        if a.sequence:
            acc, cur, label = 0.0, HOLD, "hold"
            for name, m, dur in seq:
                if t < acc + dur:
                    cur, label = m, name
                    break
                acc += dur
            cur_modes = [cur] * n
            ph_t = t - acc
        else:
            cur_modes, label, ph_t = modes, "static", t
        fade = min(1.0, ph_t / a.ramp) if a.ramp > 0 else 1.0
        q = [amp[i] * fade * math.sin(2 * math.pi * a.freq * ph_t) for i in range(n)]
        alt = math.sin(2 * math.pi * a.tau_sine * ph_t) if a.tau_sine > 0 else 1.0
        br.send(q=q, mode=cur_modes, v=[x * fade for x in vel], tau_pct=[x * fade * alt for x in tau])
        st = br.poll()
        if st and t - last_print > 0.5:
            last_print = t
            print(f"t={t:5.1f}s [{label}] " + " | ".join(
                f"[{i}] {st.phase[i]}/{st.mode[i]} q={math.degrees(st.q[i]):7.2f}deg "
                f"qd={st.qd[i]:6.2f}rad/s tq={st.torque_pct[i]:5.1f}%" for i in range(n)))
        nxt += 1.0 / a.rate
        time.sleep(max(0.0, nxt - time.perf_counter()))
    print("done sending - the board stops the motors when packets stop (watchdog)")


if __name__ == "__main__":
    _demo()
