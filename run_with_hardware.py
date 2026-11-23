"""
hardware/run_with_hardware.py - run any go2-convex-mpc example UNCHANGED and replicate its joint
torques on real motors through the Orange Pi 5.

From the repository root (like the other examples):

    # 1. no hardware: just print the torque levels the controller produces
    python -m hardware.run_with_hardware examples.ex02_trot_forward --dry-run \
        --joints FL_hip_joint,FL_thigh_joint,FL_calf_joint

    # 2. list the joint names of the model
    python -m hardware.run_with_hardware examples.ex02_trot_forward --list-joints

    # 3. real motors (3 motors = one leg), torques clipped to 6 N*m
    python -m hardware.run_with_hardware examples.ex02_trot_forward --host 192.168.50.2 \
        --joints FL_hip_joint,FL_thigh_joint,FL_calf_joint --rated-nm 60 --max-nm 6

How it works: before the example is imported, mujoco.mj_step is wrapped so that after every
physics step the torques of the chosen joints are sent to the board. The example code is not modified.
--send pos sends the simulated joint angles instead of torques (the motors' own position loop follows).
"""
import argparse
import atexit
import os
import runpy
import sys
import threading
import time

import mujoco

from .hw_mirror import HardwareMirror, hinge_joint_names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("example", help="module to run, e.g. examples.ex02_trot_forward")
    ap.add_argument("--host", default="192.168.0.15", help="IP of the Orange Pi 5 (EtherCAT master)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--joints", default="FL_hip_joint,FL_thigh_joint,FL_calf_joint",
                    help="MuJoCo hinge joints that have a motor, in motor BUS order")
    ap.add_argument("--send", choices=["torque", "pos"], default="torque")
    ap.add_argument("--rated-nm", default="60", help="rated torque of the motors, N*m (one value or a list)")
    ap.add_argument("--max-nm", default=None, help="clip torques to +/- this many N*m (default 10%% of rated)")
    ap.add_argument("--rate", type=float, default=200.0, help="packets per second to the board")
    ap.add_argument("--dry-run", action="store_true", help="do not send anything, only print torque levels")
    ap.add_argument("--list-joints", action="store_true", help="print the model's hinge joints and exit")
    ap.add_argument("--realtime", action="store_true", help="pace the simulation to wall-clock time")
    ap.add_argument("--trace", action="store_true",
                    help="report which mujoco functions the example calls (and from where), then use --hook")
    ap.add_argument("--hook", default="mj_step",
                    help="mujoco function after which torques are sent (default mj_step; see --trace)")
    ap.add_argument("--hook-warn-s", type=float, default=5.0,
                    help="warn if mujoco.mj_step has not been called after this many seconds")
    a = ap.parse_args()

    names = [n.strip() for n in a.joints.split(",") if n.strip()]
    rated = [float(x) for x in a.rated_nm.split(",")]
    rated = rated * len(names) if len(rated) == 1 else rated
    max_nm = None
    if a.max_nm is not None:
        max_nm = [float(x) for x in a.max_nm.split(",")]
        max_nm = max_nm * len(names) if len(max_nm) == 1 else max_nm

    st = {"hw": None, "wall0": None, "sim0": None, "steps": 0, "failed": False}
    counts, callers = {}, {}
    if a.trace:
        def _counting(name, orig):
            def w(*args, **kwargs):
                counts[name] = counts.get(name, 0) + 1
                if name not in callers:
                    f = sys._getframe(1)
                    callers[name] = f"{f.f_code.co_filename}:{f.f_lineno} in {f.f_code.co_name}()"
                return orig(*args, **kwargs)
            return w
        for _n in dir(mujoco):
            if _n.startswith("mj_") and not _n.startswith("mj__"):
                _o = getattr(mujoco, _n)
                if callable(_o) and not isinstance(_o, type):
                    setattr(mujoco, _n, _counting(_n, _o))

        def _report():
            for delay in (6, 12):
                time.sleep(delay)
                top = sorted(counts.items(), key=lambda kv: -kv[1])[:8]
                print("[hw-trace] mujoco functions called so far (count, first caller):", flush=True)
                for n, c in top:
                    print(f"[hw-trace]   {n:28s} x{c:<8d} {callers.get(n, '')}", flush=True)
                if not top:
                    print("[hw-trace]   (none yet - still starting up, or MuJoCo is used in another process)", flush=True)
                print(f"[hw-trace] mujoco.viewer imported: {'mujoco.viewer' in sys.modules}", flush=True)
        threading.Thread(target=_report, daemon=True).start()

    if not hasattr(mujoco, a.hook):
        raise SystemExit(f"mujoco has no function {a.hook}")
    orig_step = getattr(mujoco, a.hook)

    def wrapped(model, data, *args, **kwargs):
        orig_step(model, data, *args, **kwargs)
        st["steps"] += 1
        if st["failed"]:
            return
        try:
            _hook(model, data)
        except Exception:
            import traceback
            traceback.print_exc()
            print("[hw] hook error, replication disabled (simulation keeps running)", flush=True)
            st["failed"] = True

    def _hook(model, data):
        if st["hw"] is None:
            if a.list_joints:
                print("Hinge joints in this model:\n  " + "\n  ".join(hinge_joint_names(model)), flush=True)
                sys.stdout.flush()
                os._exit(0)                 # also works if the example steps physics in a thread
            st["hw"] = HardwareMirror(model, a.host, names, rated_nm=rated, max_nm=max_nm, port=a.port,
                                      mode=a.send, rate_hz=a.rate, dry_run=a.dry_run)
            atexit.register(st["hw"].close)
            st["wall0"], st["sim0"] = time.perf_counter(), data.time
            print(f"[hw] replicating {a.send} of {names} -> motors in this order "
                  f"({'DRY RUN' if a.dry_run else a.host})", flush=True)
        hw = st["hw"]
        wall = time.perf_counter() - st["wall0"]
        sim = data.time - st["sim0"]
        if a.realtime and sim > wall:
            time.sleep(sim - wall)
        hw.extra = f"rtf={sim / wall:.2f}" if wall > 0.5 else ""
        hw.step(data)

    setattr(mujoco, a.hook, wrapped)              # must happen before the example imports anything
    print(f"[hw] hook installed: {a.send} of {names} -> {'DRY RUN' if a.dry_run else a.host + ':' + str(a.port)}; "
          f"waiting for the first mujoco.{a.hook} call", flush=True)

    def _watch():
        time.sleep(a.hook_warn_s)
        if st["steps"] == 0:
            print(f"[hw] WARNING: mujoco.{a.hook} has not been called after {a.hook_warn_s:.0f} s. "
                  "Either the example is still starting up (MPC warm-up) or it steps the physics some other "
                  "way. Run again with --trace to see which mujoco functions it calls.", flush=True)
    threading.Thread(target=_watch, daemon=True).start()
    sys.argv = [a.example]
    try:
        runpy.run_module(a.example, run_name="__main__", alter_sys=True)
    finally:
        if st["hw"] is not None:
            st["hw"].close()
            atexit.unregister(st["hw"].close)


if __name__ == "__main__":
    main()
