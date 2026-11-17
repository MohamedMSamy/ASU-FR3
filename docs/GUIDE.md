# ASU-FR3: guide

A Franka FR3 with the CRISP Cartesian impedance controllers, in one Docker image. You start it, you send targets, you stop it.

Contents: 1 build, 2 `./fr3` commands, 3 the numbered scripts (00 to 16), 4 the other tools, 5 settings you can change, 6 where things end up, 7 if something goes wrong.

## 1. Build it (once, about an hour the first time)

You need Docker with the compose plugin. From this folder:

```bash
./fr3 build
```

That is `docker build --target final -t asu-fr3:0.1.0-dev .` (the `Dockerfile` in this folder). `compose.yaml` starts the image: `./fr3` calls `docker compose` for you, so you never type it yourself. To use another image name or tag: `ASU_FR3_IMAGE=name:tag ./fr3 build` and the same variable on every later command (or put it in `.env`, see section 5).

If someone gave you the image as a file instead of the folder: `docker load < asu-fr3-0.1.0-dev.tar`, then skip the build.

## 2. Every `./fr3` command

Notation: `[mode]` is optional and is one of `sim` (the simulated arm in MuJoCo, same as `mujoco`), `isaac` or `real`. Leave it out and you get `sim`, except where noted. `./fr3 --help` prints the list.

| Command | What it does |
|---|---|
| `./fr3 build` | builds the image from the `Dockerfile` |
| `./fr3 up sim` | starts the simulated arm. It keeps running in this terminal (Ctrl+C stops it): use a second terminal for everything else |
| `./fr3 up sim --rviz` | the same with an RViz window |
| `./fr3 up real` | the real arm. Prints a safety checklist and waits for you to type `go`. Needs the robot's IP (section 5). Never start it without the user stop in your hand |
| `./fr3 up real --rviz` | the real arm with RViz |
| `./fr3 up isaac` | the controllers only, for Isaac Sim (Isaac exchanges joint states and efforts over ROS topics) |
| `./fr3 up isaac --standin` | the same plus a MuJoCo stand-in that plays the part of Isaac Sim |
| `./fr3 down [mode]` | stops and removes the stack of that mode |
| `./fr3 logs [mode]` | follows the stack's output (Ctrl+C to leave) |
| `./fr3 shell [mode]` | opens a shell inside the running container with ROS ready |
| `./fr3 run [mode] <number or name> [arguments]` | runs a script from `scripts/` (or `tools/`) inside the running stack: section 3. `./fr3 run 01` runs `01_hold_test.py` in the sim. `./fr3 run real 01` runs it on the real arm (asks for confirmation unless you add `--yes`) |
| `./fr3 run fake <number or name> [arguments]` | rehearsal: runs the script against a fake robot, no stack, no ROS. Good for `--help` and `--dry-run` |
| `./fr3 monitor [mode] [--once]` | live dashboard in a second terminal: forces, joint torques, errors, who publishes the targets. `--once` prints one frame as text and exits |
| `./fr3 setk [mode] <k_pos> <k_rot>` | changes the Cartesian stiffness live, for example `./fr3 setk 300 15` (k_pos in N/m, k_rot in N m/rad). Refuses more than 3000 and 300. Lost when the stack restarts |
| `./fr3 gripper [mode] [open\|close\|toggle\|move W\|status] [options]` | Franka Hand. No argument = toggle. The default mode is **real**. Options: section 3, script 16 |
| `./fr3 vr [mode] [--pos S] [--rot S] [--ip IP] [--port P]` | Quest 2 teleop, section 4 |
| `./fr3 scale [mode] --pos S --rot S` | changes the VR scales while `./fr3 vr` runs |
| `./fr3 rec [mode] NAME SECONDS` | records the controller's 1 kHz stream (torques, errors, joint states, missed cycles) to `data/logs/NAME_<mode>_<time>_intro.csv`. Start it a second before the test, record a few seconds longer |
| `./fr3 cmp NAME [--tag T] [--steps]` | compares the latest sim log and the latest real log named NAME (offline). `--tag` limits it to one phase, `--steps` is for the dead-band `_steps.csv` files. The plot goes to `data/logs/cmp/NAME.png` |
| `./fr3 cmp-all` | the same for the whole standard set of tests (`GAINS=k600_20 ./fr3 cmp-all` for another gain tag) |
| `./fr3 ctl [mode] [verb ...]` | `ros2 control` inside the container: no verb lists the controllers; `./fr3 ctl switch_controllers --deactivate A --activate B` switches |
| `./fr3 ros2 [mode] <arguments>` | any `ros2` command inside the container, for example `./fr3 ros2 topic info /target_pose -v` |
| `./fr3 exec [mode] <command ...>` | any command inside the container, for example `./fr3 exec bash -c 'ros2 node list'` |
| `./fr3 urdf [--keep-ros2-control]` | exports a plain URDF with meshes to `data/urdf/` (for Isaac Sim, MoveIt and so on) |
| `./fr3 bundle` | packs the image, `compose.yaml` and `fr3` into `dist/` to copy to another machine |

Only one stack of each mode runs at a time. The running stack has its own ROS domain (100 for sim and isaac, 101 for real), so a sim and a real stack do not see each other.

## 3. The numbered scripts

`./fr3 run [mode] <number> [arguments]`. The number is the two digits at the start of the file name (`1` works as `01`; `08.1` is its own script). You can also type the whole name (`01_hold_test.py`). Everything after the number goes to the script. `./fr3 run 01 --help` prints the script's own help.

**Arguments every script shares.** Where a script has them, they are repeated in its table below, so you never have to look back here:

| Argument | Default | Meaning |
|---|---|---|
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

`--no-switch` exists only on the scripts that switch to the Cartesian controller by themselves (01, 02, 03, 04, 09, 10). Scripts 00, 06, 11, 12, 14 and 16 take none of these (they do not move the arm, or they are offline).

Units everywhere: metres, radians for joints, degrees where the argument name says so, positions in the robot's base frame (x forward, y left, z up). The scripts refuse any target outside the safety box defined in `config/lab_config.py` (sim: x 0.20 to 0.65, y −0.40 to 0.40, z 0.25 to 0.75; real: x 0.30 to 0.54, y −0.38 to 0.38, z 0.10 to 0.55).

Overview:

| Number | Name | One line |
|---|---|---|
| 00 | `00_status.py` | Status check |
| 01 | `01_hold_test.py` | Hold a pose |
| 02 | `02_waypoints.py` | Waypoints from a file |
| 03 | `03_geometry.py` | Trace a shape |
| 04 | `04_deadband_cartesian.py` | Dead-band test, Cartesian |
| 05 | `05_deadband_joint.py` | Dead-band test, one joint |
| 06 | `06_compare_logs.py` | Compare two logs |
| 07 | `07_record_waypoints.py` | Teach waypoints by hand |
| 08 | `08_home.py` | Go home (joint trajectory) |
| 08.1 | `08.1_home.py` | Go home (joint impedance) |
| 09 | `09_rviz_goal.py` | Drag the hand in RViz |
| 10 | `10_template.py` | Template for your own script |
| 11 | `11_axis_errors.py` | Per-axis errors of a log |
| 12 | `12_sweep_figure.py` | Stiffness-sweep figures |
| 13 | `13_sine_sweep.py` | Sine sweep of one joint |
| 14 | `14_sweep_fit.py` | Fit a sine sweep |
| 15 | `15_set_target.py` | Set the target by hand |
| 16 | `16_gripper.py` | Franka Hand |

### 00: Status check (`00_status.py`)

Read only: nothing moves. Prints which controllers exist and which is active, where the hand is and how far it is from pointing straight down, the joint positions and efforts, and the main controller settings. Run it first, in the sim and on the real arm.

Commands:

```bash
./fr3 run 00
./fr3 run real 00
```

Arguments: none.

### 01: Hold a pose (`01_hold_test.py`)

Moves to a pose, holds it for N seconds and prints the error once per second, then the final error and how fast it is still drifting. The basic "is the controller good" test.

Commands:

```bash
./fr3 run 01
./fr3 run 01 --duration 10 --pos 0.45 0.2 0.45 --log hold
./fr3 run real 01 --duration 20 --lin-speed 0.05 --ang-speed 0.25
```

| Argument | Default | Meaning |
|---|---|---|
| `--pos X Y Z` | 0.40 0 0.45 | where to hold (metres, robot base frame) |
| `--yaw-deg D` | 0 | turn the hand about the vertical axis by D degrees |
| `--duration S` | 20 | seconds to hold and watch |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 02: Waypoints from a file (`02_waypoints.py`)

Visits the points of a CSV file one after the other, hand pointing down. Every point is checked against the safety box before anything moves. One point per line: `x, y, z, [yaw_deg], [settle_seconds]` (`#` starts a comment). A relative file name is looked up in `data/` first, then in `examples/`.

Commands:

```bash
./fr3 run 02 --dry-run
./fr3 run 02 --file waypoints_example.csv --log wp
./fr3 run real 02 --file my_points.csv --lin-speed 0.03
```

| Argument | Default | Meaning |
|---|---|---|
| `--file FILE` | waypoints_example.csv | the CSV with the points |
| `--repeat N` | 1 | run the whole list N times |
| `--no-return` | off | do not go back to the first point at the end |
| `--dry-run` | off | only check the file and print the plan; no robot, no ROS |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 03: Trace a shape (`03_geometry.py`)

Moves the hand along a shape at constant speed (it speeds up, cruises and brakes smoothly). At the end it prints the tracking error. Preview first with `--dry-run`: it prints the length, the time and the accelerations and needs no robot.

Commands:

```bash
./fr3 run fake 03 --shape circle --center 0.45 0 0.45 --width 0.10 --speed 0.05 --dry-run
./fr3 run 03 --shape circle --center 0.45 0 0.45 --width 0.10 --speed 0.05 --loops 3 --log circle
./fr3 run 03 --shape circle --plane xz --center 0.45 0 0.5 --width 0.12 --yaw-deg 30
```

| Argument | Default | Meaning |
|---|---|---|
| `--shape NAME` | required | one of: line, square, rectangle, triangle, circle, ellipse, figure8, spiral |
| `--plane P` | xy | xy (flat), xz or yz (upright planes); the hand keeps pointing down |
| `--center X Y Z` | 0.40 0 0.45 | centre of the shape (m) |
| `--width W` | 0.10 | width in metres (for a circle this is the diameter) |
| `--height H` | = width (figure8: width/2) | height in metres |
| `--turns N` | 2 | spiral only: number of turns |
| `--r-inner R` | 0.01 | spiral only: inner radius (m) |
| `--speed V` | 0.05 | cruise speed along the path (m/s) |
| `--accel A` | 0.10 | speed-up and braking acceleration (m/s²) |
| `--loops N` | 1 | how many times the shape is traced |
| `--yaw-deg D` | 0 | turn the hand about the vertical axis by D degrees |
| `--dry-run` | off | preview only; no robot, no ROS |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 04: Dead-band test, Cartesian (`04_deadband_cartesian.py`)

Measures the friction dead band along one axis: the target moves in tiny steps 0 → +A → −A → 0 and the script records where the hand really ends up. Writes a `_steps.csv` file next to the log. Run the same command on the sim and the real arm and compare with 06.

Commands:

```bash
./fr3 run 04 --axis ry --amplitude 3 --step 0.25 --log db_ry
./fr3 run real 04 --axis x --amplitude 5 --step 0.5 --log db_x
```

| Argument | Default | Meaning |
|---|---|---|
| `--axis A` | ry | x, y, z (millimetres) or rx, ry, rz (degrees, about the base axes) |
| `--amplitude A` | 5 mm / 3 deg | largest offset |
| `--step S` | 0.5 mm / 0.25 deg | size of one step |
| `--dwell S` | 2.0 | seconds to wait at each step |
| `--pos X Y Z` | 0.40 0 0.45 | where to do the test |
| `--allow-large` | off | allow amplitudes above 10 mm / 8 deg |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 05: Dead-band test, one joint (`05_deadband_joint.py`)

The same staircase test on one joint, through the joint impedance controller. Units are radians.

Commands:

```bash
./fr3 run 05 --joint 2 --amplitude 0.01 --step 0.0005 --log dbj2
./fr3 run real 05 --joint 4 --log dbj4
```

| Argument | Default | Meaning |
|---|---|---|
| `--joint N` | required | joint number 1 to 7 |
| `--amplitude A` | 0.01 | largest offset (rad) |
| `--step S` | 0.0005 | size of one step (rad) |
| `--dwell S` | 1.5 | seconds to wait at each step |
| `--allow-large` | off | allow amplitudes above 0.05 rad |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 06: Compare two logs (`06_compare_logs.py`)

Offline (no robot, no stack needed): puts a sim log and a real log side by side, prints tracking error, joint positions and torques, and can draw a plot. For dead-band files (`*_steps.csv`) it overlays the hysteresis loops. `./fr3 cmp NAME` does the file-finding for you.

Commands:

```bash
./fr3 cmp hold_k300_15 --tag hold
./fr3 run fake 06 /home/ros/data/logs/a_sim_....csv /home/ros/data/logs/a_real_....csv --png /home/ros/data/logs/out.png
```

| Argument | Default | Meaning |
|---|---|---|
| `files` | required | the log CSV files to compare (paths inside the container: `/home/ros/data/logs/...`) |
| `--tag T` | all rows | only rows whose tag starts with T (for example `hold` or `trace`) |
| `--png FILE` | no plot | write a comparison plot to FILE |

### 07: Teach waypoints by hand (`07_record_waypoints.py`)

Real arm only in practice: switches to the gravity-compensation controller so you can push the arm by hand. Move the hand, press Enter to record the pose, repeat, type `q` to finish. The CSV plays back with 02. The user stop stays in your hand.

Commands:

```bash
./fr3 run real 07 --out my_points.csv
```

| Argument | Default | Meaning |
|---|---|---|
| `--out FILE` | my_points.csv | CSV to write (a relative name goes into `data/`) |
| `--append` | off | add to the file instead of overwriting it |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 08: Go home (joint trajectory) (`08_home.py`)

Switches to the joint trajectory controller and moves all joints to the "ready" pose. The duration is picked so no joint is faster than the limit (at least 5 s). Afterwards the Cartesian target is set to the home pose so switching back to the Cartesian controller does not jump. Clear the workspace first: the joint move is not checked against obstacles.

Commands:

```bash
./fr3 run 08
./fr3 run real 08 --max-joint-speed 0.15
./fr3 run 08 --dry-run
```

| Argument | Default | Meaning |
|---|---|---|
| `--max-joint-speed V` | 0.2 | largest joint speed on the way (rad/s) |
| `--dry-run` | off | print the plan, do not move |
| `--no-settle` | off | do not set the Cartesian target to the home pose afterwards |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 08.1: Go home (joint impedance) (`08.1_home.py`)

The same destination as 08, but through the joint impedance controller: the target is moved smoothly, and the controller carries the gravity term. Same options as 08. Use 08 or 08.1, not both at once.

Commands:

```bash
./fr3 run 08.1
./fr3 run 08.1 --max-joint-speed 0.1
./fr3 run 08.1 --dry-run
```

| Argument | Default | Meaning |
|---|---|---|
| `--max-joint-speed V` | 0.2 | largest joint speed on the way (rad/s) |
| `--dry-run` | off | print the plan, do not move |
| `--no-settle` | off | do not set the Cartesian target to the home pose afterwards |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 09: Drag the hand in RViz (`09_rviz_goal.py`)

A marker in RViz: drag it and the hand follows (arrows = x/y/z, square = horizontal plane, ring = turn about the vertical; right-click for tilt). The goal is clamped to the safety box and the target moves only at the speed limits. Needs `./fr3 up sim --rviz`. Do not run another script at the same time: the controller ignores a target with two publishers.

Commands:

```bash
./fr3 up sim --rviz   (one terminal)  then  ./fr3 run 09
./fr3 run 09 --tilt --max-tilt 10
./fr3 run real 09 --lin-speed 0.03 --max-tilt 8
```

| Argument | Default | Meaning |
|---|---|---|
| `--duration S` | 0 | seconds to run (0 = until Ctrl+C) |
| `--ns NAME` | lab_goal | name of the interactive marker (topics `/NAME/update`, `/NAME/feedback`) |
| `--tilt` | off | start with the two tilt rings visible |
| `--max-tilt D` | 15 | largest tilt away from straight down (degrees) |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 10: Template for your own script (`10_template.py`)

Start here for your own experiments: copy it (`cp scripts/10_template.py scripts/20_mine.py`) and edit `my_trajectory()`. The file explains the helper functions in its header.

Commands:

```bash
./fr3 run fake 10
./fr3 run 10 --duration 10 --log tmpl
```

| Argument | Default | Meaning |
|---|---|---|
| `--duration S` | 10 | seconds the template runs |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |
| `--no-switch` | off | do not change controllers: use whichever one is active now |

### 11: Per-axis errors of a log (`11_axis_errors.py`)

Offline: the error on each axis for one or more log files (works on old logs too).

Commands:

```bash
./fr3 run fake 11 "/home/ros/data/logs/hold_*_sim_*.csv" --tag hold
```

| Argument | Default | Meaning |
|---|---|---|
| `logs` | required | one or more log CSV files (wildcards allowed) |
| `--tag T` | all rows | only rows with this tag, for example `hold` |
| `--forces` | off | also print the spring force and torque K·e (needs the `.json` next to each log) |

### 12: Stiffness-sweep figures (`12_sweep_figure.py`)

Offline: figures and a table of sim versus real over a stiffness sweep. For the logs of a gain sweep named `<prefix>...`.

Commands:

```bash
./fr3 run fake 12 kswp_p
```

| Argument | Default | Meaning |
|---|---|---|
| `prefix` | required | the log-name prefix, for example `kswp_p` |
| `--out DIR` | data/logs/figures | where the figures go |
| `--ts-skip-k K ...` | 300 | gains left out of the time-series figure |

### 13: Sine sweep of one joint (`13_sine_sweep.py`)

Moves one joint's target on a small sine wave at several frequencies and records the response, to find the joint's effective inertia and damping (14 does the fitting). Start from home (08). Caps keep the torque and the speed small. Writes a `_sweep.csv` file.

Commands:

```bash
./fr3 run 13 --joint 4 --log sw_j4
./fr3 run fake 13 --joint 2 --dry-run
./fr3 run real 13 --joint 4 --log sw_j4_k300_15
```

| Argument | Default | Meaning |
|---|---|---|
| `--joint N` | required | joint number 1 to 7 |
| `--freqs F ...` | 0.2 0.4 0.8 1.5 2.5 4.0 6.0 9.0 | frequencies in Hz |
| `--amplitude A` | 0.02 | largest amplitude (rad), reduced per frequency by the caps |
| `--max-torque T` | 3.0 | peak spring torque K·A (N m); the controller clips at 5 |
| `--max-speed V` | 0.4 | peak joint speed (rad/s) |
| `--cycles N` | 6 | cycles per frequency (each at least 4 s) |
| `--rate HZ` | 100 | sampling and target rate |
| `--k-joint K` | read from the controller | joint stiffness used for the plan (N m/rad) |
| `--dry-run` | off | print the plan, do not move |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 14: Fit a sine sweep (`14_sweep_fit.py`)

Offline: fits effective inertia, damping and delay to the `_sweep.csv` files of 13. With a URDF it also gives the armature (make the URDF with `./fr3 urdf`).

Commands:

```bash
./fr3 run fake 14 "/home/ros/data/logs/sw_j4_*_sweep.csv"
```

| Argument | Default | Meaning |
|---|---|---|
| `sweeps` | required | one or more `_sweep.csv` files (wildcards allowed; the newest match of each is used) |
| `--urdf FILE` | /home/ros/data/urdf/fr3.urdf | the URDF for the link inertia |

### 15: Set the target by hand (`15_set_target.py`)

The Cartesian controller remembers its last target, also while it is off, and jumps there when switched on. This script sets the target you want: the current pose (no argument), or a new position the target moves to smoothly. `--activate` also switches to the Cartesian controller first, without a jump.

Commands:

```bash
./fr3 run 15
./fr3 run 15 --xyz 0.45 0 0.35 --activate
./fr3 run 15 --xyz 0.45 0 0.35 --down --yaw 20 --duration 10
./fr3 run real 15 --xyz 0.45 0 0.35 --activate --lin-speed 0.03
```

| Argument | Default | Meaning |
|---|---|---|
| `--xyz X Y Z` | current position | target position (m, base frame) |
| `--down` | off | hand straight down (with `--yaw`); otherwise the current orientation is kept |
| `--yaw D` | 0 | degrees about the vertical, only with `--down` |
| `--instant` | off | no ramp: one jump of the target (the arm snaps there if the Cartesian controller is active) |
| `--duration S` | from the speeds | seconds for the move; overrides the speeds |
| `--activate` | off | switch to the Cartesian controller and ramp to the target |
| `--secs S` | 1.0 | how long the target message is repeated |
| `--dry-run` | off | print only, publish nothing |
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv`, plus a `.json` next to it with the controller settings |
| `--lin-speed V` | 0.10 m/s sim, 0.05 m/s real | speed of the target when it moves (m/s). The scripts refuse more than 0.30 (sim) or 0.10 (real) |
| `--ang-speed W` | 0.50 rad/s sim, 0.25 rad/s real | rotation speed of the target (rad/s) |
| `--yes` | off | skip the "are you sure" question on the real arm |

### 16: Franka Hand (`16_gripper.py`)

Open, close or toggle the hand (the same as `./fr3 gripper`). It reads the width first, so toggle always does the opposite and open on an open hand does nothing. Real stack only: in the sim `status` works and open/close say there is no action server. The hand closes at the given force without checking what is between the fingers.

Commands:

```bash
./fr3 gripper
./fr3 gripper open
./fr3 gripper close --width 0.012 --force 20
./fr3 gripper move 0.04
./fr3 gripper status
./fr3 run fake 16
```

| Argument | Default | Meaning |
|---|---|---|
| `action` | toggle | `toggle`, `open`, `close`, `move` or `status` |
| `value` | none | width in metres, only for `move` |
| `--width W` | 0.0 | grasp width (m): 0 closes fully; for a part give its width |
| `--force F` | 20 | grasp force (N); refused above 70 |
| `--speed V` | 0.05 | closing speed (m/s); refused above 0.1 |
| `--epsilon-inner E` | 0.01 | the grasp succeeds if the fingers stop between width−E and width+epsilon-outer |
| `--epsilon-outer E` | 0.08 | see above |
| `--open-width W` | 0.08 | width `open` goes to (m) |
| `--open-speed V` | 0.1 | opening speed (m/s) |
| `--open-above W` | 0.07 | widths above this count as open (m) |
| `--timeout T` | 15 | seconds to wait for the hand |
| `--namespace NS` | franka_gripper | namespace of the hand's action servers |
| `--dry-run` | off | print what would be sent, send nothing |

## 4. The other tools

**Live monitor** `./fr3 monitor [mode] [--once]`: a terminal dashboard (Cartesian forces and moments, joint torques, position and angle error, the stiffness in use, and which node is publishing the targets). Run it in a second terminal.

**VR teleop** (Quest 2): `./fr3 vr [mode] [--pos S] [--rot S] [--ip IP] [--port P]`. The headset app comes from the official store; this only starts the receiving side inside the stack.

| Argument | Default | Meaning |
|---|---|---|
| `--pos S` | 0.5 (or `$POS`) | position scale: how many centimetres the arm moves per centimetre of your hand |
| `--rot S` | 0.2 (or `$ROT`) | rotation scale (0 keeps the orientation fixed) |
| `--ip IP` | first address of `hostname -I` (or `$VR_IP`) | the computer's address that the headset connects to. On a machine with the robot cable plugged in, pass the Wi-Fi address |
| `--port P` | 10000 | TCP port of the endpoint |

Start the Cartesian controller yourself first: `./fr3 ctl switch_controllers --deactivate joint_trajectory_controller --activate cartesian_impedance_controller`. While it runs, `./fr3 scale --pos 0.5 --rot 0.3` changes the scales (positions up to the limits printed by the tool). `VR_TELEOP=vr_teleop_clamped.py ./fr3 vr` runs the variant that keeps the target inside the safety box.

Other files in `tools/` are used by the verbs above (`monitor.py`, `rec_introspection.py`, `vr_*.py`) or by the lab work (`ksweep_summary.py`, `box_from_points.py`: turns the points of script 07 into safety-box limits). Run any of them with `./fr3 run <name>.py`.

## 5. Settings you can change

Copy `.env.example` to `.env`; compose reads it automatically. Every line is optional.

| Variable | Default | What it is |
|---|---|---|
| `FR3_PROJECT` | asu-fr3 | prefix of the compose project. Another value plus another `ROS_DOMAIN_ID` runs a second, separate stack |
| `ASU_FR3_IMAGE` | asu-fr3:0.1.0-dev | image name and tag |
| `ROS_DOMAIN_ID` | 100 (sim, isaac), 101 (real) | ROS domain |
| `RMW` | cyclone | `cyclone` or `fastdds` |
| `ROS_NETWORK_INTERFACE` | lo | network interface for Cyclone |
| `FR3_SCENE` | set by the mode | the MuJoCo scene in `models/` |
| `CONTROLLERS_FILE` | set by the mode | the gains file in `config/` |
| `RVIZ_CONFIG` | asu_fr3.rviz | RViz profile in `config/` (used with `--rviz`) |
| `ROBOT_IP` | 172.16.0.2 | the real arm's address |
| `CYCLONE_CONFIG` | cyclone_config.xml | Cyclone file in `config/` (isaac mode uses `cyclone_isaac.xml`) |

Isaac mode only: `ISAAC_JOINT_STATES_TOPIC` (default `/isaac_joint_states`, Isaac to controllers), `ISAAC_JOINT_COMMANDS_TOPIC` (default `/isaac_joint_commands`, controllers to Isaac, effort), `ISAAC_SUM_WRAPPED_JOINT_STATES` (true if Isaac reports angles wrapped to ±2π), `ISAAC_EFFORT_INITIAL_VALUE` (effort sent before a controller writes; `none` = NaN), `ISAAC_COMMAND_THRESHOLD` (empty = plugin default; −1 = publish every cycle), `BRIDGE_PHYSICS_HZ` (stand-in physics rate, 1000), `BRIDGE_REALTIME_FACTOR` (stand-in speed, 1.0; 0.2 emulates a slow Isaac).

Which gravity set-up each mode uses is chosen for you: the sim holds the arm up by itself, the real arm is held by the robot, Isaac mode lets the controllers do it. The launch refuses a wrong combination.

## 6. Where things end up

- Logs of your runs: `data/logs/` (created on your machine, never shared).
- Waypoint files you make: `data/` (`./fr3 run 02 --file my_points.csv` finds them there). Samples: `examples/`.
- Exported URDF: `data/urdf/`.
- Comparison plots: `data/logs/cmp/`; figures from script 12: `data/logs/figures/`.
- Your own changes to `scripts/`, `tools/`, `config/`, `models/`, `bin/` are used immediately: the stack reads these folders from your copy, no rebuild needed.

## 7. If something goes wrong

- `./fr3 logs` shows the stack's output; `./fr3 run 00` shows what is running.
- "no script numbered N": the message lists the numbered scripts.
- A script says another controller is active: `./fr3 ctl` lists them.
- The arm jumps when you switch controllers: set the target first with `./fr3 run 15` (script 15).
- Real arm: the user stop stays in your hand, always.
