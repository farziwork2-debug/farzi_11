#!/usr/bin/env python3
"""
mujoco_to_motors.py - run a MuJoCo simulation on the laptop and mirror chosen joints onto real
MyActuator motors through the Cerebellum board (myact_ctl --mode csp-udp).

    python3 mujoco_to_motors.py --xml robot.xml --host 192.168.1.50 --joints FL_hip,FL_knee

--joints lists MuJoCo *hinge* joint names in the SAME ORDER as the motors on the EtherCAT bus
(first name -> motor [0], the one nearest the master, and so on).

This file is a template: put your own locomotion controller / policy call in controller().
NOTE: written against the MuJoCo Python bindings (mujoco.viewer.launch_passive); it has not been
run against a real model here, so test it with the motors unloaded first.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from myact_udp import MyActBridge


def controller(model, data):
    """Replace with your locomotion code: set data.ctrl (or whatever your code does)."""
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True, help="MuJoCo model file")
    ap.add_argument("--host", required=True, help="IP address of the Cerebellum board")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--joints", required=True, help="comma list of hinge joint names, motor bus order")
    ap.add_argument("--rate", type=float, default=200.0, help="packets per second to the motors")
    a = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(a.xml)
    data = mujoco.MjData(model)
    names = [n.strip() for n in a.joints.split(",") if n.strip()]
    adr = [model.joint(n).qposadr[0] for n in names]
    bridge = MyActBridge(a.host, len(names), a.port)
    period = 1.0 / a.rate
    last_print = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        next_send = time.perf_counter()
        while viewer.is_running():
            t_start = time.perf_counter()

            controller(model, data)
            mujoco.mj_step(model, data)

            # Send on wall-clock time, so packets keep flowing even if the sim is slow or paused;
            # the board stops the motors if packets stop for --udp-timeout-ms.
            now = time.perf_counter()
            if now >= next_send:
                bridge.send([data.qpos[i] for i in adr])
                next_send = now + period
                st = bridge.poll()
                if st and now - last_print > 1.0:
                    last_print = now
                    print(" | ".join(f"{n}: {st.phase[i]} q={st.q[i]:.3f} rad tq={st.torque_pct[i]:.1f}%"
                                     for i, n in enumerate(names)))

            viewer.sync()

            # run in real time, otherwise the motors cannot keep up with the simulation
            left = model.opt.timestep - (time.perf_counter() - t_start)
            if left > 0:
                time.sleep(left)


if __name__ == "__main__":
    main()
