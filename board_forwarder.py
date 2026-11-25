#!/usr/bin/env python3
"""
board_forwarder.py - runs on the Orange Pi 5. Replaces your own receiver script.

    laptop (your packets, port 5005)  ->  this script  ->  myact_ctl (port 5006, same machine)  ->  motors

Your packet format is kept (change it with --fmt / --hdr if it differs):  struct "<Id12d" = uint32 seq, float64 timestamp, 12 x float64 values
(values 0..2 = FL hip, thigh, calf, ...). Choose which of the 12 go to which motor with --idx.

What it adds on top of a plain receiver, so that nothing jerks:
  * motion is RELATIVE to the first packet and faded in with a smooth ramp (--ramp-s), scaled by --gain
  * every joint is clamped to +/- --max-dev-deg around its start value
  * every joint is speed-limited (--max-rate-dps for angles, --max-nm-s for torques)
  * if packets stop for --stale-ms the motors are told to HOLD (they stay enabled, no drop);
    when packets resume the ramp starts again from zero, from wherever the input is then
  * torque mode: --torque-scale multiplies the incoming torques (any factor), --gain is the 0..1 fade-in
    level, --max-nm clips. The status line shows the peak incoming and sent torque, so you can tune the scale
  * packets that are the wrong size, not finite, or absurdly large are ignored
  * --dry-run prints what would be sent without touching the motors

Start the board side first:   sudo ./myact_ctl --mode udp --udp-port 5006 --udp-from 127.0.0.1 ...
Then:                          python3 board_forwarder.py --send pos --gain 0.2 --max-dev-deg 10
"""
import argparse
import csv
import math
import socket
import struct
import sys
import time

from myact_udp import HOLD, POS, TORQUE, MyActBridge

DEFAULT_FMT = "<Id12d"               # your sender: seq, timestamp, 12 values


def smooth(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen-port", type=int, default=5005, help="port your laptop sends to")
    ap.add_argument("--bridge-host", default="127.0.0.1", help="where myact_ctl runs")
    ap.add_argument("--bridge-port", type=int, default=5006, help="myact_ctl --udp-port")
    ap.add_argument("--fmt", default=DEFAULT_FMT,
                    help="struct format of your packet (default \"<Id12d\" = uint32 seq, float64 time, 12 float64)")
    ap.add_argument("--hdr", type=int, default=2, help="number of header fields before the values in --fmt")
    ap.add_argument("--idx", default="0,1,2", help="which of the 12 values drive the motors, in motor bus order")
    ap.add_argument("--send", choices=["pos", "torque", "hold"], default="pos",
                    help="pos: the values are joint angles in rad; torque: they are joint torques in N*m; "
                         "hold: enable the motors and hold, ignore the values (link test)")
    ap.add_argument("--gain", type=float, default=0.2, help="fraction of the commanded motion that is applied (0..1)")
    ap.add_argument("--ramp-s", type=float, default=5.0, help="seconds to fade the motion in (also after a pause)")
    ap.add_argument("--max-dev-deg", type=float, default=10.0, help="angles: max distance from the start value (0 = no clamp)")
    ap.add_argument("--max-rate-dps", type=float, default=60.0, help="angles: max joint speed, deg/s (0 = no limit)")
    ap.add_argument("--torque-scale", type=float, default=1.0,
                    help="torque mode: multiply the incoming torques by this (e.g. 0.1 or 3); applied before gain and clip")
    ap.add_argument("--rated-nm", default="60", help="torque mode: rated torque of the motors, N*m (list ok)")
    ap.add_argument("--max-nm", default="2", help="torque mode: clip, N*m (list ok; 0 = no clip)")
    ap.add_argument("--max-nm-s", type=float, default=5.0, help="torque mode: max torque change, N*m per second (0 = no limit)")
    ap.add_argument("--stale-ms", type=float, default=150.0, help="no packet for this long -> HOLD")
    ap.add_argument("--rate", type=float, default=100.0, help="packets per second sent to myact_ctl")
    ap.add_argument("--print-hz", type=float, default=2.0)
    ap.add_argument("--log", default=None, metavar="FILE", help="write a CSV of input, command and motor feedback")
    ap.add_argument("--dry-run", action="store_true", help="do not talk to myact_ctl, only print")
    a = ap.parse_args()

    try:
        PKT = struct.Struct(a.fmt)
    except struct.error as e:
        sys.exit(f"bad --fmt: {e}")
    nvals = len(PKT.unpack(bytes(PKT.size))) - a.hdr
    idx = [int(x) for x in a.idx.split(",")]
    n = len(idx)
    if nvals < 1 or not all(0 <= i < nvals for i in idx):
        sys.exit(f"--idx values must be 0..{nvals - 1} for this packet format")
    if not 0.0 <= a.gain <= 1.0:
        sys.exit("--gain must be between 0 and 1")
    rated = [float(x) for x in a.rated_nm.split(",")]
    rated = rated * n if len(rated) == 1 else rated
    max_nm = [abs(float(x)) for x in a.max_nm.split(",")]
    max_nm = max_nm * n if len(max_nm) == 1 else max_nm
    if len(rated) != n or len(max_nm) != n:
        sys.exit("--rated-nm / --max-nm need 1 value or one per motor")
    dev = math.radians(a.max_dev_deg)
    rate_lim = math.radians(a.max_rate_dps)
    dt = 1.0 / a.rate

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", a.listen_port))
    rx.settimeout(0.002)
    bridge = None if a.dry_run else MyActBridge(a.bridge_host, n, a.bridge_port)
    logf = logw = None
    if a.log:
        logf = open(a.log, "w", newline="")
        logw = csv.writer(logf)
        logw.writerow(["t_s", "seq", "state", "gain"] + [f"in{i}" for i in range(n)] + [f"cmd{i}" for i in range(n)]
                      + [f"q_real{i}" for i in range(n)] + [f"tq_real_pct{i}" for i in range(n)])

    if a.send == "torque":
        print(f"torque mode: sent = clip(gain * {a.torque_scale} * input, +/-{max_nm} N*m); "
              f"slew <= {a.max_nm_s} N*m/s", flush=True)
    print(f"listening on UDP {a.listen_port}; forwarding {a.send} of values {idx} to "
          f"{'(dry run)' if a.dry_run else f'{a.bridge_host}:{a.bridge_port}'}; gain={a.gain} ramp={a.ramp_s}s "
          f"max_dev={a.max_dev_deg}deg rate<={a.max_rate_dps}deg/s", flush=True)

    t0 = time.perf_counter()
    latest = None                 # newest input values (selected), list of n
    seq = last_seq = 0
    t_rx = 0.0
    n_rx = n_bad = n_gap = 0
    state = "WAIT"                # WAIT (no packets yet), RUN, STALE
    base = None                   # input values at the start of the current run
    prev = None                   # last command
    t_arm = 0.0
    t_next = t0
    t_print = 0.0
    fb = None
    win_in = [0.0] * n            # peak |input| / |command| since the last status line
    win_cmd = [0.0] * n
    win_n = win_clip = 0

    try:
        while True:
            try:
                data, _ = rx.recvfrom(4096)
                if len(data) == PKT.size:
                    f = PKT.unpack(data)
                    vals = [f[a.hdr + i] for i in idx]
                    if all(math.isfinite(v) and abs(v) < 1000.0 for v in vals):
                        n_rx += 1
                        if last_seq and f[0] != last_seq + 1:
                            n_gap += 1
                        last_seq = seq = f[0]
                        latest, t_rx = vals, time.perf_counter()
                    else:
                        n_bad += 1
                else:
                    n_bad += 1
            except socket.timeout:
                pass

            now = time.perf_counter()
            if now < t_next:
                continue
            t_next += dt
            if t_next < now - 0.1:
                t_next = now

            if latest is None:
                continue                                        # nothing received yet: send nothing
            fresh = (now - t_rx) * 1000.0 <= a.stale_ms

            if a.send == "hold" or not fresh:
                if state == "RUN":
                    state = "STALE"
                    print("[fwd] input stale -> HOLD (motors stay enabled); will ramp in again when packets resume",
                          flush=True)
                if state == "WAIT" and a.send == "hold":
                    state = "HOLD"
                cmd, g = prev, 0.0
                if bridge:
                    bridge.send(mode=HOLD)
            else:
                if state != "RUN":                              # (re)arm: rebase on the current input, ramp from 0
                    state, base, t_arm = "RUN", list(latest), now
                    prev = list(base) if a.send == "pos" else [0.0] * n
                    print(f"[fwd] armed: input rebased, ramping in over {a.ramp_s:.1f}s at gain {a.gain}", flush=True)
                g = a.gain * (smooth((now - t_arm) / a.ramp_s) if a.ramp_s > 0 else 1.0)
                cmd = []
                for k in range(n):
                    if a.send == "pos":
                        want = base[k] + g * (latest[k] - base[k])
                        if dev > 0:
                            want = clip(want, base[k] - dev, base[k] + dev)
                        v = want if rate_lim <= 0 else prev[k] + clip(want - prev[k], -rate_lim * dt, rate_lim * dt)
                    else:
                        raw = g * latest[k] * a.torque_scale
                        want = raw if max_nm[k] <= 0 else clip(raw, -max_nm[k], max_nm[k])
                        if want != raw:
                            win_clip += 1
                        v = want if a.max_nm_s <= 0 else prev[k] + clip(want - prev[k], -a.max_nm_s * dt, a.max_nm_s * dt)
                    cmd.append(v)
                    win_in[k] = max(win_in[k], abs(latest[k]))
                    win_cmd[k] = max(win_cmd[k], abs(v))
                win_n += 1
                prev = cmd
                if bridge:
                    if a.send == "pos":
                        bridge.send(mode=POS, q=cmd)
                    else:
                        bridge.send(mode=TORQUE, tau_nm=cmd, rated_nm=rated)
            if bridge:
                st = bridge.poll()
                if st:
                    fb = st

            if logw and cmd is not None:
                logw.writerow([f"{now - t0:.4f}", seq, state, f"{g:.3f}"] + [f"{v:.5f}" for v in latest]
                              + [f"{v:.5f}" for v in cmd]
                              + ([f"{v:.5f}" for v in fb.q] + [f"{v:.2f}" for v in fb.torque_pct] if fb else [""] * 2 * n))
            if now - t_print > 1.0 / a.print_hz:
                t_print = now
                s = f"[fwd] {state:5s} seq={seq} rx={n_rx} gaps={n_gap} bad={n_bad} g={g:.2f} "
                s += "in=" + str([round(v, 3) for v in latest]) + " "
                if cmd is not None:
                    s += "cmd=" + str([round(v, 3) for v in cmd]) + " "
                if a.send == "torque" and win_n:
                    s += (f"peak_in={[round(v, 1) for v in win_in]} peak_cmd={[round(v, 2) for v in win_cmd]} "
                          f"clipped={100.0 * win_clip / (win_n * n):.0f}% ")
                    win_in, win_cmd, win_n, win_clip = [0.0] * n, [0.0] * n, 0, 0
                if fb:
                    s += "| motors " + " ".join(f"{p}/{m} q={q:.3f} tq={t:.1f}%" for p, m, q, t in
                                                 zip(fb.phase, fb.mode, fb.q, fb.torque_pct))
                elif bridge:
                    s += f"| no reply yet (tx={bridge.tx})"
                print(s, flush=True)
    except KeyboardInterrupt:
        print("\n[fwd] stopping: sending HOLD, then the board's watchdog will disable the motors", flush=True)
    finally:
        if bridge:
            for _ in range(10):
                bridge.send(mode=HOLD)
                time.sleep(0.01)
        if logf:
            logf.close()


if __name__ == "__main__":
    main()
