# ASU-FR3: guide

A Franka FR3 with the CRISP Cartesian impedance controllers, in one Docker image. You start it, you send targets, you stop it.

Contents: 1 build, 2 `./fr3` commands, 3 the numbered scripts (00 to 16), 4 the other tools, 5 settings you can change, 6 defaults (scene, controllers, gains, model, limits), 7 how to change a default, 8 recording a terminal session, 9 where things end up, 10 if something goes wrong.

## 1. Build it (once, about an hour the first time)

You need Docker with the compose plugin. From this folder:

```bash
./fr3 build
```

That is `docker build --target final -t asu-fr3:0.1.0-dev .` (the `Dockerfile` in this folder). `compose.yaml` starts the image: `./fr3` calls `docker compose` for you, so you never type it yourself. To use another image name or tag: `ASU_FR3_IMAGE=name:tag ./fr3 build` and the same variable on every later command (or put it in `.env`, see section 5).

If someone gave you the image as a file instead of the folder: `docker load < asu-fr3-0.1.0-dev.tar`, then skip the build.

## 2. Every `./fr3` command

Notation: `[mode]` is optional and is one of `sim` (the simulated arm in MuJoCo, same as `mujoco`), `isaac` or `real`. Leave it out and you get `sim`, except where noted. `./fr3 --help` prints the list.

| Command                                                                | What it does                                                                                                                                                                                                                  |
| ---------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `./fr3 build`                                                          | builds the image from the `Dockerfile`                                                                                                                                                                                        |
| `./fr3 up sim`                                                         | starts the simulated arm. It keeps running in this terminal (Ctrl+C stops it): use a second terminal for everything else                                                                                                      |
| `./fr3 up sim --rviz`                                                  | the same with an RViz window                                                                                                                                                                                                  |
| `./fr3 up real`                                                        | the real arm. Prints a safety checklist and waits for you to type `go`. Needs the robot's IP (section 5). Never start it without the user stop in your hand                                                                   |
| `./fr3 up real --rviz`                                                 | the real arm with RViz                                                                                                                                                                                                        |
| `./fr3 up isaac`                                                       | the controllers only, for Isaac Sim (Isaac exchanges joint states and efforts over ROS topics)                                                                                                                                |
| `./fr3 up isaac --standin`                                             | the same plus a MuJoCo stand-in that plays the part of Isaac Sim                                                                                                                                                              |
| `./fr3 down [mode]`                                                    | stops and removes the stack of that mode                                                                                                                                                                                      |
| `./fr3 logs [mode]`                                                    | follows the stack's output (Ctrl+C to leave)                                                                                                                                                                                  |
| `./fr3 shell [mode]`                                                   | opens a shell inside the running container with ROS ready                                                                                                                                                                     |
| `./fr3 run [mode] <number or name> [arguments]`                        | runs a script from `scripts/` (or `tools/`) inside the running stack: section 3. `./fr3 run 01` runs `01_hold_test.py` in the sim. `./fr3 run real 01` runs it on the real arm (asks for confirmation unless you add `--yes`) |
| `./fr3 run fake <number or name> [arguments]`                          | rehearsal: runs the script against a fake robot, no stack, no ROS. Good for `--help` and `--dry-run`                                                                                                                          |
| `./fr3 monitor [mode] [--once]`                                        | live dashboard in a second terminal: forces, joint torques, errors, who publishes the targets. `--once` prints one frame as text and exits                                                                                    |
| `./fr3 setk [mode] <k_pos> <k_rot>`                                    | changes the Cartesian stiffness live, for example `./fr3 setk 300 15` (k_pos in N/m, k_rot in N m/rad). Refuses more than 3000 and 300. Lost when the stack restarts                                                          |
| `./fr3 gripper [mode] [open\|close\|toggle\|move W\|status] [options]` | Franka Hand. No argument = toggle. The default mode is **real**. Options: section 3, script 16                                                                                                                                |
| `./fr3 vr [mode] [--pos S] [--rot S] [--ip IP] [--port P] [--unclamped]` | Quest 2 teleop, section 4                                                                                                                                                                                                     |
| `./fr3 scale [mode] --pos S --rot S`                                   | changes the VR scales while `./fr3 vr` runs                                                                                                                                                                                   |
| `./fr3 rec [mode] NAME SECONDS`                                        | records the controller's 1 kHz stream (torques, errors, joint states, missed cycles) to `data/logs/NAME_<mode>_<time>_intro.csv`. The file is written while recording, so SECONDS can be as long as the session (a 1-hour recording is about 1.8 GB on disk, no memory), and a killed recording keeps what was written. Start it a second before the test; press Ctrl+C when the test is done (SECONDS is the upper limit). The same columns are in every `--log` file, so a `--log` run and a `rec` run of the same test line up by `stamp`. `rec` is the long capture (1 kHz, press Ctrl+C when done); `--log` is one test. Columns: `t` (seconds), `stamp`, `qN` / `dqN` (measured position and speed of joint N), `tau_measN` (reported torque), `tau_cmdN` (commanded torque), `cmd_limitedN` (1 = the command was clipped), `cycles_<controller>` / `missed_<controller>` (control cycles run and missed); `pos_x..z`, `quat_x..w` (the hand/TCP pose), `tgt_pos_x..z`, `tgt_quat_x..w` (the target pose), and `vr_pos_x..z`, `vr_quat_x..w`, `vr_clutch`, `vr_trigger` (the raw VR hand pose and buttons, only when a headset sent data: use them to see whether the VR input needs smoothing)                              |
| `./fr3 cmp NAME [--tag T] [--steps]`                                   | compares the latest sim log and the latest real log named NAME (offline). `--tag` limits it to one phase, `--steps` is for the dead-band `_steps.csv` files. The plot goes to `data/logs/cmp/NAME.png`                        |
| `./fr3 cmp-all`                                                        | the same for the whole standard set of tests (`GAINS=k600_20 ./fr3 cmp-all` for another gain tag)                                                                                                                             |
| `./fr3 ctl [mode] [verb ...]`                                          | `ros2 control` inside the container: no verb lists the controllers; `./fr3 ctl switch_controllers --deactivate A --activate B` switches                                                                                       |
| `./fr3 ros2 [mode] <arguments>`                                        | any `ros2` command inside the container, for example `./fr3 ros2 topic info /target_pose -v`                                                                                                                                  |
| `./fr3 exec [mode] <command ...>`                                      | any command inside the container, for example `./fr3 exec bash -c 'ros2 node list'`                                                                                                                                           |
| `./fr3 urdf [--keep-ros2-control]`                                     | exports a plain URDF with meshes to `data/urdf/` (for Isaac Sim, MoveIt and so on)                                                                                                                                            |
| `./fr3 bundle`                                                         | packs the image, `compose.yaml` and `fr3` into `dist/` to copy to another machine                                                                                                                                             |

Only one stack of each mode runs at a time. The running stack has its own ROS domain (100 for sim and isaac, 101 for real), so a sim and a real stack do not see each other.

## 3. The numbered scripts

`./fr3 run [mode] <number> [arguments]`. The number is the two digits at the start of the file name (`1` works as `01`; `08.1` is its own script). You can also type the whole name (`01_hold_test.py`). Everything after the number goes to the script. `./fr3 run 01 --help` prints the script's own help.

**Arguments every script shares.** Where a script has them, they are repeated in its table below, so you never have to look back here:

| Argument | Default | Meaning |
|---|---|---|
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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
| `--log NAME` | none (no log) | save the run to `data/logs/NAME_<mode>_<time>.csv` (100 Hz) plus a `.json` next to it with the controller settings. The CSV has the same readable columns as `./fr3 rec` (`qN`, `dqN`, `tau_measN`, `tau_cmdN`, `cmd_limitedN`, `cycles_/missed_<controller>`, `stamp`) plus pose, target and error columns (and the `vr_*` columns when a headset is connected) |
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

**VR teleop** (Quest 2): `./fr3 vr [mode] [--pos S] [--rot S] [--ip IP] [--port P] [--unclamped]`. The headset app comes from the official store; this only starts the receiving side inside the stack. **By default every target is clamped to the safety box of the mode** (`config/lab_config.py`, sim and real alike), so the hand cannot be led out of it. The start line prints `handler=vr_teleop_clamped.py` and that the target is clamped.

| Argument | Default | Meaning |
|---|---|---|
| `--pos S` | 0.5 (or `$POS`) | position scale: how many centimetres the arm moves per centimetre of your hand |
| `--rot S` | 0.2 (or `$ROT`) | rotation scale (0 keeps the orientation fixed) |
| `--ip IP` | first address of `hostname -I` (or `$VR_IP`) | the computer's address that the headset connects to. On a machine with the robot cable plugged in, pass the Wi-Fi address |
| `--port P` | 10000 | TCP port of the endpoint |
| `--unclamped` | off | use the plain handler (`vr_teleop.py`) that does **not** limit the target to the safety box. It prints a warning at start, and a second one on the real arm. Only for tests where you need to leave the box on purpose |

Switch to the Cartesian controller first, with a target at the current pose so nothing jumps: `./fr3 run <mode> 15 --activate` (for example `./fr3 run real 15 --activate`). While it runs, `./fr3 scale --pos 0.5 --rot 0.3` changes the scales (positions up to the limits printed by the tool). `VR_TELEOP=<file in tools/> ./fr3 vr` picks any other handler.

Other files in `tools/` are used by the verbs above (`monitor.py`, `rec_introspection.py`, `vr_*.py`) or by the lab work (`ksweep_summary.py`, `box_from_points.py`: turns the points of script 07 into safety-box limits). Run any of them with `./fr3 run <name>.py`.

## 5. Settings you can change

Copy `.env.example` to `.env`; compose reads it automatically. Every line is optional.

| Variable                | Default                      | What it is                                                                                              |
| ----------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------------- |
| `FR3_PROJECT`           | asu-fr3                      | prefix of the compose project. Another value plus another `ROS_DOMAIN_ID` runs a second, separate stack |
| `ASU_FR3_IMAGE`         | asu-fr3:0.1.0-dev            | image name and tag                                                                                      |
| `ROS_DOMAIN_ID`         | 100 (sim, isaac), 101 (real) | ROS domain                                                                                              |
| `RMW`                   | cyclone                      | `cyclone` or `fastdds`                                                                                  |
| `ROS_NETWORK_INTERFACE` | lo                           | network interface for Cyclone                                                                           |
| `FR3_SCENE`             | set by the mode              | the MuJoCo scene in `models/`                                                                           |
| `CONTROLLERS_FILE`      | set by the mode              | the gains file in `config/`                                                                             |
| `RVIZ_CONFIG`           | asu_fr3.rviz                 | RViz profile in `config/` (used with `--rviz`)                                                          |
| `ROBOT_IP`              | 172.16.0.2                   | the real arm's address                                                                                  |
| `CYCLONE_CONFIG`        | cyclone_config.xml           | Cyclone file in `config/` (isaac mode uses `cyclone_isaac.xml`)                                         |

Isaac mode only: `ISAAC_JOINT_STATES_TOPIC` (default `/isaac_joint_states`, Isaac to controllers), `ISAAC_JOINT_COMMANDS_TOPIC` (default `/isaac_joint_commands`, controllers to Isaac, effort), `ISAAC_SUM_WRAPPED_JOINT_STATES` (true if Isaac reports angles wrapped to ±2π), `ISAAC_EFFORT_INITIAL_VALUE` (effort sent before a controller writes; `none` = NaN), `ISAAC_COMMAND_THRESHOLD` (empty = plugin default; −1 = publish every cycle), `BRIDGE_PHYSICS_HZ` (stand-in physics rate, 1000), `BRIDGE_REALTIME_FACTOR` (stand-in speed, 1.0; 0.2 emulates a slow Isaac).

Which gravity set-up each mode uses is chosen for you: the sim holds the arm up by itself, the real arm is held by the robot, Isaac mode lets the controllers do it. The launch refuses a wrong combination.

## 6. Defaults: scene, controllers, gains, model

**What each mode starts with** (when you set nothing):

| Mode | Scene (`models/`) | Controllers file (`config/`) | Who holds the arm against gravity | `enable_overrun` |
|---|---|---|---|---|
| `sim` | `scene_fric_gravcomp_v1.xml` | `controllers.yaml` | the simulator (gravity on, `gravcomp` on every body) | true |
| `isaac` | `scene_fric_gravity_v1.xml` (only for the `--standin`) | `controllers_gravity.yaml` | the controllers (`use_gravity_compensation: true`) | true |
| `real` | none (the real arm loads no scene) | `controllers_real.yaml` | libfranka, in the robot | false |

`controllers.yaml` and `controllers_real.yaml` differ only in `enable_overrun`. `controllers_gravity.yaml` is `controllers.yaml` plus `use_gravity_compensation: true` in the three CRISP controller blocks. In every mode the **joint trajectory controller is the one active at start**; the other motion controllers are loaded but inactive. The lab protocol lowers the Cartesian stiffness first (`./fr3 setk 300 15`).

**Controller values** (the same in the three files unless noted; units N/m, N m/rad, N m):

| Controller | Value |
|---|---|
| controller manager | `update_rate` 1000 Hz |
| `cartesian_impedance_controller` | tool frame `fr3_hand_tcp`, base frame `base`. Stiffness `k_pos` 900 N/m on x, y, z; `k_rot` 30 N m/rad on x, y, z. Nullspace stiffness 2.0. Filters `q` 0.5, `dq` 0.5, `output_torque` 0.5. `use_friction` true, `use_coriolis_compensation` true, `use_operational_space` false, `use_local_jacobian` true |
| `joint_impedance_controller` | task stiffness 0 (joint space only). Nullspace stiffness 5.0, damping 0.5, `max_tau` 5.0, weights 150, 150, 150, 150, 50, 30, 10, so the spring is 750, 750, 750, 750, 250, 150, 50 N m/rad per joint and the damper 75, 75, 75, 75, 25, 15, 5. `limit_error` and `limit_torques` true, `max_delta_tau` 0.5, `use_friction` true |
| `gravity_compensation` | `k_pos` 0, `k_rot` 30, nullspace stiffness 0 (a hand-guiding mode) |
| `joint_trajectory_controller` | effort commands. P 600, 600, 600, 600, 250, 150, 50; D 30, 30, 30, 30, 10, 10, 5; I 0 (clamp 1) |

Not set in the files, so the controller's built-in values apply (from reading the CRISP source): the Cartesian stiffness is clamped at 900 N/m and 60 N m/rad whatever you set, damping is 2 times the square root of the stiffness, `filter.target_pose` is 0.1, and the target error is clipped at 0.1 m and 0.5 rad.

**The MuJoCo model** (`fr3_fric_v1.xml`, also inside the gravcomp one): the arm with the Franka Hand welded open.

| Joint | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|
| `frictionloss` [N m] | 0.83 | 0.61 | 1.58 | 0.91 | 0.77 | 0.54 | 0.43 |
| `armature` [kg m²] | 0.45 | 0.45 | 0.35 | 0.35 | 0.06 | 0.06 | 0.06 |
| torque limit [N m] | 87 | 87 | 87 | 87 | 12 | 12 | 12 |

Joint `damping` 0.003 on all joints. The solver setting `noslip_iterations="5"` is what keeps the arm from creeping under friction, and the integrator is `implicitfast`. Gravity is -9.81 m/s² in both scenes; the gravcomp scene adds `gravcomp="1"` on every body and `actuatorgravcomp="true"` on the joints. The home pose (`READY_Q`) is 0, -45°, 0, -135°, 0, 90°, 45°; the test position `READY_POS` is `0.40 0 0.45` m. The friction numbers are tuned so the simulated arm holds and moves like the measured real arm; they are about 2.3 times the real arm's measured sliding friction, because the simulator's friction is a soft constraint. Do not copy them to another simulator as if they were measured values.

**Safety limits per mode** (`config/lab_config.py`, `LIMITS`; the scripts refuse a target outside the box or a speed above the cap):

| Limit | `sim` | `real` |
|---|---|---|
| box, lowest corner x, y, z [m] | 0.20, -0.40, 0.25 | 0.30, -0.38, 0.10 |
| box, highest corner x, y, z [m] | 0.65, 0.40, 0.75 | 0.54, 0.38, 0.55 |
| default target speed `lin_speed` | 0.10 m/s | 0.05 m/s |
| default rotation speed `ang_speed` | 0.50 rad/s | 0.25 rad/s |
| speed cap `max_lin_speed` | 0.30 m/s | 0.10 m/s |
| acceleration cap `max_accel` (scripts refuse a path that needs more) | 1.0 m/s² | 0.4 m/s² |
| corner velocity jump `max_corner_jump` | 0.15 m/s | 0.05 m/s |

**Constants every script shares** (`config/lab_config.py`):

| Constant | Value |
|---|---|
| test position `READY_POS` | 0.40, 0.00, 0.45 m, hand pointing down |
| home pose `READY_Q` | 0, -45°, 0, -135°, 0, 90°, 45° |
| frames and topics | base frame `base`, tool frame `fr3_hand_tcp`, joint states `franka/joint_states`, joint names `fr3_joint1` to `fr3_joint7` |
| `PUBLISH_HZ` (how fast scripts stream targets) | 50 Hz |
| `LOG_HZ` (the `--log` rate) | 100 Hz |
| `rec` rate | 1000 Hz, flushed to disk every 2 s |
| `SETTLE_S` (wait at each waypoint before measuring) | 3.0 s |
| folders | logs `data/logs`, data `data`, samples `examples` (change with `FR3_LOG_DIR`, `FR3_DATA_DIR`) |

**Timings built into the scripts:** connecting waits up to 10 s for the robot. Switching to a controller does: hold the pose, wait 0.3 s, switch, wait 0.5 s, check with `ros2 control` (up to 30 s), wait 0.5 s, hold again. On Ctrl+C or an error the hold target is published for 0.3 s, then the script exits. A second Ctrl+C during that is ignored.

**VR defaults** (`./fr3 vr`, the files in `tools/`):

| Setting | Default |
|---|---|
| position scale, rotation scale (`--pos`, `--rot`) | 0.5, 0.2 (the handler itself defaults to 0.5 and 0.5, the launcher sets 0.2) |
| handler | `vr_teleop_clamped.py` (`--unclamped`: `vr_teleop.py`) |
| clutch: hold the right grip (`press_middle`) above | 0.5 |
| headset pose older than this drops the clutch (`stale_after`) | 0.3 s |
| target rate | 50 Hz |
| target may lead the real hand pose by (`max_lead`, `max_rot_lead`) | 0.03 m and 0.10 rad (set by the launcher; the handler's own defaults are 0.15 m and 0.5 rad) |
| largest tilt of the hand from straight down (`max_tilt_deg`) | 20°; the clamped handler shrinks it to 8° within 0.05 m of the box faces (`edge_tilt_deg`, `edge_margin`) |
| home button | the upper button (B or Y), goes to `READY_POS` at the mode's default speeds |
| Franka Hand: toggle with the right index trigger (`press_index`) above | 0.7, at most one press per 0.6 s; grasp width 0.0 m, force 20 N, speed 0.05 m/s, tolerance 0.01 / 0.08 m; open to 0.08 m at 0.1 m/s. Start with `GRIPPER=0` to skip the hand handler |
| endpoint port | 10000 |
| `./fr3 scale` limits | at most 1.0 for the position scale and for the rotation scale |

**Command defaults.** Most commands take an optional mode first and default to `sim` (`down`, `logs`, `shell`, `run`, `monitor`, `setk`, `rec`, `scale`, `vr`, `ros2`, `exec`, `ctl`); `up` needs a mode, and `gripper` defaults to **real**. `./fr3 ctl` with no verb lists the controllers (`CTL_TIMEOUT` 10 s for `ros2` calls). `./fr3 rec` needs a name and the seconds. `./fr3 cmp` compares all rows unless you give `--tag`; `./fr3 cmp-all` uses the gains tag `k300_15` (`GAINS=...` to change it). `./fr3 setk` refuses more than 3000 N/m or 300 N m/rad. `./fr3 urdf` writes to `data/urdf/`.

**Environment variables the tools read** (besides the ones in section 5):

| Variable | Default | What it does |
|---|---|---|
| `FR3_LOG_DIR`, `FR3_DATA_DIR` | `data/logs`, `data` | where logs and data files go |
| `GAINS` | `k300_15` | gains tag for `./fr3 cmp-all` |
| `CMP_A`, `CMP_B` | `sim`, `real` | which two modes `./fr3 cmp` compares (`CMP_B=sim` compares sim with sim) |
| `CTL_TIMEOUT` | 10 | seconds before a `ros2` call through `./fr3 ctl` is stopped |
| `VR_IP`, `POS`, `ROT`, `VR_TELEOP`, `GRIPPER` | first address of `hostname -I`, 0.5, 0.2, `vr_teleop_clamped.py`, 1 | the VR launcher's settings (the `--ip`, `--pos`, `--rot`, `--unclamped` options override them) |
| `CRISP_LAB_NO_VERIFY` | 0 | 1 skips the check that the right controller is active after a switch |
| `CRISP_LAB_READ_LIST` | 0 | 1 uses crisp_py's own controller switch when the controller list can be read |
| `CRISP_LAB_FAKE_DEADBAND_MM`, `CRISP_LAB_FAKE_DEADBAND_DEG` | 1.0, 1.0 | the dead band of the fake robot in `./fr3 run fake` |
| `ASU_FR3_ALLOW_GRAVITY_MISMATCH` | unset | 1 turns the launch guard's refusal into a warning, in the simulator modes only, never on the real arm |

## 7. How to change a default

Which folders your edits reach without a rebuild: `config/`, `models/`, `scripts/`, `tools/`, `bin/` and `data/` are mounted into the container from your copy, so an edit is used at the next start (scripts and tools: at the next run). `packages/`, `examples/` and the `Dockerfile` are baked into the image: after editing those run `./fr3 build`.

| You want to change | How | When it takes effect |
|---|---|---|
| a script's arguments (speed, position, duration, joint, ...) | add the flag: `./fr3 run 01 --pos 0.45 0.2 0.45 --duration 10`. `./fr3 run fake 01 --help` prints every flag of script 01 without a stack | that run |
| the default speeds, the home pose, the logging rate, the safety box | edit `config/lab_config.py` (`LIMITS` for the speeds, caps and the box, `READY_POS`, `READY_Q`, `LOG_HZ`, ...) | the next script run |
| the Cartesian stiffness, live | `./fr3 setk [mode] 300 15` (k_pos in N/m, k_rot in N m/rad) | at once; lost when the stack restarts |
| any other controller parameter, live | `./fr3 ros2 [mode] param set /cartesian_impedance_controller <name> <value>`, for example `task.k_pos_x 300.0`, `use_friction false`, `filter.target_pose 0.5`. Numbers with a decimal point (`300.0`, not `300`). List them with `./fr3 ros2 [mode] param list /cartesian_impedance_controller`, read one with `param get`. The joint controller is `/joint_impedance_controller` (for example `nullspace.stiffness 5.0`) | at once; lost when the stack restarts |
| a controller parameter for good | edit the file of the mode in `config/` (`controllers.yaml` for sim, `controllers_gravity.yaml` for isaac, `controllers_real.yaml` for real), then `./fr3 down` and `./fr3 up <mode>`. Another file for one start: `CONTROLLERS_FILE=name.yaml ./fr3 up sim` | the next start |
| the simulated arm (friction, armature, mass) | edit the model in `models/` (`fr3_fric_gravcomp_v1.xml` for sim), then restart. Another scene for one start: `FR3_SCENE=name.xml ./fr3 up sim`. The launch refuses a scene and controllers file that would compensate gravity twice or not at all | the next start |
| the real safety box | record the corner points by hand: `./fr3 run real 07 --out box_points.csv` (move the hand, Enter to record each corner, `q` to finish), then `python3 tools/box_from_points.py data/box_points.csv --margin 0.02 --z-low-margin 0.005` prints two lines; paste them into `LIMITS["real"]` in `config/lab_config.py` | the next script run |
| ports, domain, image name, RViz profile, robot address | a line in `.env` (copy `.env.example`), see section 5 | the next start |
| the VR scales | `./fr3 scale [mode] --pos 0.5 --rot 0.3` while it runs, or `--pos` and `--rot` when starting it | at once |

## 8. Recording a terminal session

`./fr3 rec` and `--log` record **data**. To keep what the terminal printed (errors, the checklist, the script's own summary), record the text:

| What | Command |
|---|---|
| Whole session: everything typed and printed, in one file | `script -q -f data/logs/terminal_$(date +%Y%m%d_%H%M%S).txt` — everything after this is recorded; type `exit` to stop |
| The stack's own log (it is gone after `./fr3 down`) | `ROBOT_IP=<ip> ./fr3 up real 2>&1 \| tee -i data/logs/stack_real_$(date +%Y%m%d_%H%M%S).log` |
| A running stack's log, from now on | `./fr3 logs real 2>&1 \| tee -i data/logs/stack_real_$(date +%Y%m%d_%H%M%S).log` |
| Catch the moment of a communication error live | `./fr3 logs real \| grep --line-buffered -iE "communication\|overrun\|reflex"` |
| Take the colour codes out of a saved file | `sed 's/\x1b\[[0-9;]*m//g' data/logs/stack_real_<time>.log > clean.log` |

Use `tee -i`, not plain `tee`: Ctrl+C goes to the whole pipe, and without `-i` `tee` can quit before the stack's last lines (the shutdown messages, often the interesting ones) are written. The file fills as the stack runs; follow it from another terminal with `tail -f <file>`. This file holds only the stack's output; what a script prints in another terminal goes into the `script` file, or add `2>&1 \| tee -i data/logs/run_01.txt` to that run.

Start `script` first when you sit down (one terminal for the whole lab day), start the stack with `tee` in the next terminal, and run `./fr3 rec` for the data. Name the files with the date: nobody remembers which run is which a week later. All of these go into `data/logs/`, which is never shared.

## 9. Where things end up

- Logs of your runs: `data/logs/` (created on your machine, never shared).
- Waypoint files you make: `data/` (`./fr3 run 02 --file my_points.csv` finds them there). Samples: `examples/`.
- Exported URDF: `data/urdf/`.
- Comparison plots: `data/logs/cmp/`; figures from script 12: `data/logs/figures/`.
- Your own changes to `scripts/`, `tools/`, `config/`, `models/`, `bin/` are used immediately: the stack reads these folders from your copy, no rebuild needed.

## 10. If something goes wrong

- `./fr3 logs` shows the stack's output; `./fr3 run 00` shows what is running.
- "no script numbered N": the message lists the numbered scripts.
- `./fr3 setk` or `./fr3 ros2 ... param ...` says "Node not found" or "FAILED to set", although the stack is running (the monitor and `./fr3 ctl` work): the ROS daemon inside the container went stale. Reset it and try again: `./fr3 exec <mode> ros2 daemon stop` (for example `./fr3 exec isaac ros2 daemon stop`).
- A script says another controller is active: `./fr3 ctl` lists them.
- The arm jumps when you switch controllers: set the target first with `./fr3 run 15` (script 15).
- Real arm: the user stop stays in your hand, always.
