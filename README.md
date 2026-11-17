# ASU-FR3

CRISP Cartesian impedance controllers for the Franka FR3 (ROS 2 Jazzy, ros2_control) in **one Docker image**, with MuJoCo models calibrated against the real arm, a controller interface for Isaac Sim, and a real-arm mode on franka_hardware (libfranka).

```bash
./fr3 build           # once
./fr3 up sim          # start the simulated arm
./fr3 run 01          # run script 01 (hold a pose) in the running stack
./fr3 down            # stop
```

Full reference (every command, every script, every argument): [docs/GUIDE.md](docs/GUIDE.md). `./fr3 --help` lists the commands.

## Why this exists

Running the CRISP controllers used to mean several repositories, a pixi environment, a pile of launch files and a lot of tribal knowledge. This project replaces that with one image and one command:

- **One image, three modes.** `sim` (MuJoCo), `isaac` (controllers only, Isaac Sim exchanges joint states and efforts over ROS topics) and `real` (the physical FR3). The same controllers, gains files and scripts run in all three.
- **Built for people who just want the controller.** `./fr3 up`, `./fr3 run 01`, `./fr3 down`. No ROS workspace to build, no environment to activate, no launch files to read.
- **Numbered scripts.** Eighteen ready-made scripts (`00` status check to `16` gripper, plus `08.1`): hold a pose, follow waypoints, trace circles and figure-8s, measure friction dead bands, sweep a joint, drag the hand in RViz, home the arm. Each documents every argument and prints its own `--help`.
- **Safety by default.** A safety box and speed caps in every script, a checklist and a confirmation before anything touches the real arm, a stack that always leaves the arm holding its pose when a script ends or you press Ctrl+C, and a launch-time guard that refuses a double-counted gravity setup.

## What was built

- **A reproducible image** pinning CRISP v2.4.1, franka_ros2 v3.0.0 and ROS 2 Jazzy, with a CLI (`./fr3`) over docker compose: start, stop, run, monitor, record, compare, live stiffness changes, URDF export.
- **MuJoCo models calibrated against the real arm.** Measured on the physical FR3: joint friction, joint inertia, the friction dead band, and how the arm holds poses and follows sine sweeps. Findings along the way:
  - MuJoCo's friction behaved like a soft constraint and let the arm creep; fixed with `noslip_iterations`, so the sim holds still the way the real arm does.
  - Per-joint friction and armature fitted to the real measurements.
  - Hold errors at the side poses: sim 17.5 to 18.3 mm against 17.6 and 18.0 mm on the real arm.
- **Start-up lock.** The simulated arm no longer falls at launch or sags when controllers switch: gravity is on and the simulator compensates it, and the reported torques include gravity like the real arm's.
- **Gravity guard.** Exactly one party may compensate gravity (the simulator, the controllers or libfranka). The launch refuses any other combination, and refuses a gravity-compensating controllers file on the real arm outright.
- **Isaac Sim interface.** Joint states in, effort commands out, over named topics, with Cyclone DDS (unicast file included) or Fast DDS, plus a MuJoCo stand-in that plays the part of Isaac Sim for testing our side.
- **Real-arm tooling.** A safety checklist, a hand-guiding waypoint recorder, a gripper command, a live monitor, a 1 kHz recorder, and sim-versus-real log comparison.
- **VR teleoperation** with a Quest 2 headset (the headset app comes from the official store; the receiving side is ours).
- **Documented behaviour.** The sim-versus-real differences, the controller's limits and the open problems were measured and written down instead of hidden (the longer write-ups are internal and not part of this repository).

## Status

| Mode | State |
|---|---|
| `sim` | Works, calibrated. Holds, start-up and gravity match the real arm closely: side holds within 0.7 mm. Not an exact copy: the centre hold ends 4 to 5 mm from the target against 9.6 mm on the real arm, settling is slower (0.14 s against 0.05 s), and the phase of a 9 Hz sine sweep differs by up to 32 degrees. Moving paths (circles, figure-8s) have no real-arm data to compare against yet. |
| `isaac` | Our side is complete (topics, DDS options, gravity handling, stand-in). It has not been run against a real Isaac Sim session yet. Until the Isaac scene stops applying gravity to the robot's bodies, the arm can still fall at launch and sag when controllers switch. |
| `real` | Tested on a physical FR3 in the lab (holds, dead bands, sweeps, gains). One known fault: a `communication_constraints_violation` right after a homing move finishes, cause not found yet. Whether libfranka already compensates motor friction is also open. Use it with the user stop in your hand, as with any robot code. |

## What is in this folder

| Folder | Contents |
|---|---|
| `scripts/` | the numbered scripts and their helper modules |
| `tools/` | the live monitor and the VR tools |
| `bin/` | the helpers behind the `./fr3` commands |
| `config/` | controller gains files, DDS files, RViz profiles, safety limits (`lab_config.py`) |
| `models/` | MuJoCo models and scenes, URDF, meshes |
| `packages/` | the ROS 2 packages (launch files, simulator plugin) |
| `examples/` | sample waypoint files |

## Built on

[CRISP controllers](https://github.com/utiasDSL/crisp_controllers) and crisp_py (TUM LSY), franka_ros2 and libfranka (Franka Robotics), MuJoCo and the MuJoCo Menagerie FR3 model (Google DeepMind), ros_tcp_endpoint (Unity) and quest2ros. Licences and notices: `LICENSE` and `NOTICE`.
