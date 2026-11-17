# models/: the MuJoCo models and scenes

The model is calibrated against the real arm (per-joint friction, armature, no creep): arm plus the Franka Hand welded open.

| File | What it is | Used by |
|---|---|---|
| `scene_fric_gravcomp_v1.xml` + `fr3_fric_gravcomp_v1.xml` | gravity ON, compensated by the simulator itself (`gravcomp` on every body, `actuatorgravcomp` on the joints): the arm does not fall at launch and does not sag when controllers switch; the effort it reports includes gravity, like the real arm | `./fr3 up sim` (with `config/controllers.yaml`) |
| `scene_fric_gravity_v1.xml` + `fr3_fric_v1.xml` | gravity ON, nothing compensates it in the model: the controllers do (`config/controllers_gravity.yaml`) | `./fr3 up isaac --standin` (Isaac Sim has gravity on) |
| `scene_fric_v1.xml` | gravity OFF | nothing by default (the real arm does not use a scene) |
| `fr3_single.urdf.xacro`, `fr3.urdf.xacro`, `fr3.ros2_control.urdf.xacro` | the robot description and the hardware plugin for each mode | all modes |
| `assets/` | the meshes | all modes |
| `LICENSE` | MuJoCo Menagerie licence | |

Who compensates gravity must be exactly one party: the launch refuses a wrong pair (a controllers file with `use_gravity_compensation: true` together with a scene that already compensates, a scene with gravity off, or the real arm). The real arm never loads a scene: libfranka compensates gravity in the robot.

The original models (no hand, friction 0.2, gravity off) are in `archive/models_old_variants/`.
