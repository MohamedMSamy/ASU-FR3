# ASU-FR3

The CRISP Cartesian impedance controllers (ROS 2 Jazzy, ros2_control) for the Franka FR3, in one Docker image, with three modes: `sim` (MuJoCo), `isaac` (controllers for Isaac Sim) and `real` (franka_hardware / libfranka).

```bash
./fr3 build           # once
./fr3 up sim          # start
./fr3 run 01          # run script 01_hold_test.py in the running stack
./fr3 down            # stop
```

Everything you need is in [docs/GUIDE.md](docs/GUIDE.md): build, every `./fr3` command, every script with its arguments. `./fr3 --help` lists the commands.

Folders: `scripts/` numbered lab scripts, `tools/` monitor and VR teleop, `bin/` helpers behind the `./fr3` verbs, `config/` controller gains and RViz profiles, `models/` MuJoCo models and URDF, `packages/` the ROS 2 packages, `examples/` sample waypoint files.

Licence: see `LICENSE` and `NOTICE`.
