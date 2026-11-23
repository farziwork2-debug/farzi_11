#!/usr/bin/env python3
"""
mujoco_to_motors.py - run a MuJoCo simulation on the laptop and drive real MyActuator motors
through the Orange Pi 5 (myact_ctl --mode udp).

Two ways to use the motors:

  --send torque : forward the joint torques your locomotion code produces (N*m) to the motors
  --send pos    : forward the simulated joint angles; the motors' own position loop does the rest

    python3 mujoco_to_motors.py --xml robot.xml --host 192.168.50.2 \
        --joints FL_hip,FL_knee,FL_ankle --send torque --rated-nm 60

--joints lists MuJoCo *hinge* joint names in the SAME ORDER as the motors on the EtherCAT bus
(first name -> motor [0], the one nearest the master).
--rated-nm is the motor's rated torque in N*m (X10-200: 60, X8-120: 43; one value or a list per motor).
Check your own datasheet - the conversion N*m -> % of rated torque depends on it.

Put your locomotion controller / policy call in controller(). By default the torque that is
sent is what the simulator applies at each joint (data.qfrc_actuator); change get_torques() if your
code produces the torque values some other way (for example from your own packets).

NOTE: written against the MuJoCo Python bindings (mujoco.viewer.launch_passive); it has not been run
against a real model here. Test with the motors unloaded and the robot on a stand first.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from myact_udp import MyActBridge, POS, TORQUE


def controller(model, data):
    """Replace with your locomotion code: set data.ctrl (or whatever your code does)."""
    pass


def get_torques(model, data, dof_adr):
    """Joint torques in N*m, one per motor. Default: what the actuators apply at these joints."""
    return [float(data.qfrc_actuator[d]) for d in dof_adr]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True, help="MuJoCo model file")
    ap.add_argument("--host", required=True, help="IP address of the Orange Pi 5 (EtherCAT master)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--joints", required=True, help="comma list of hinge joint names, motor bus order")
    ap.add_argument("--send", choices=["torque", "pos"], default="torque")
    ap.add_argument("--rated-nm", default="60", help="rated torque of the motors in N*m (one value or a list)")
    ap.add_argument("--rate", type=float, default=500.0, help="packets per second to the board")
    a = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(a.xml)
    data = mujoco.MjData(model)
    names = [n.strip() for n in a.joints.split(",") if n.strip()]
    qadr = [model.joint(n).qposadr[0] for n in names]
    dadr = [model.joint(n).dofadr[0] for n in names]
    rated = [float(x) for x in a.rated_nm.split(",")]
    if len(rated) == 1:
        rated = rated * len(names)
    if len(rated) != len(names):
        raise SystemExit("--rated-nm needs 1 value or one per joint")

    bridge = MyActBridge(a.host, len(names), a.port)
    period = 1.0 / a.rate
    last_print = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        next_send = time.perf_counter()
        while viewer.is_running():
            t_start = time.perf_counter()

            controller(model, data)
            mujoco.mj_step(model, data)

            # Send on wall-clock time so packets keep flowing even when the sim is slow or paused;
            # the board stops all motors if packets stop for --udp-timeout-ms.
            now = time.perf_counter()
            if now >= next_send:
                if a.send == "torque":
                    bridge.send(mode=TORQUE, tau_nm=get_torques(model, data, dadr), rated_nm=rated)
                else:
                    bridge.send(mode=POS, q=[data.qpos[i] for i in qadr])
                next_send = now + period
                st = bridge.poll()
                if st and now - last_print > 1.0:
                    last_print = now
                    print(" | ".join(f"{n}: {st.phase[i]}/{st.mode[i]} q={st.q[i]:.3f} rad "
                                     f"tq={st.torque_pct[i]:.1f}%" for i, n in enumerate(names)))

            viewer.sync()

            # run in real time, otherwise the motors cannot keep up with the simulation
            left = model.opt.timestep - (time.perf_counter() - t_start)
            if left > 0:
                time.sleep(left)


if __name__ == "__main__":
    main()
