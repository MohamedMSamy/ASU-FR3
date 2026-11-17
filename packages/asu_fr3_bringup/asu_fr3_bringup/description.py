"""Render the robot_description (URDF) from the effective models folder.

Environment (isaac backend only): ISAAC_JOINT_STATES_TOPIC, ISAAC_JOINT_COMMANDS_TOPIC, ISAAC_SUM_WRAPPED_JOINT_STATES,
ISAAC_EFFORT_INITIAL_VALUE ("none" leaves the effort command unset), ISAAC_COMMAND_THRESHOLD (empty = plugin default, -1 = publish every cycle).

The launch file and the tests both call `render_robot_description`, so what the tests check is what
the launch executes. The mappings are the ones the original demo launch passed. `ros2_control`, `arm_id`
and `hand` are not xacro arguments of fr3_single.urdf.xacro and are ignored by xacro (as in the demo);
`mujoco_model` is a real argument since M3.3.

  python3 -m asu_fr3_bringup.description --use-fake-hardware true > robot.urdf
"""
import argparse
import os

import xacro

from asu_fr3_bringup import overlay


# What each mode starts with when FR3_SCENE / CONTROLLERS_FILE are not set (the `fr3` script has the same table: scene_for, controllers_for).
#   mujoco: the calibrated model (friction, armature, noslip) with gravity ON and the PLANT compensating it (gravcomp + actuatorgravcomp: the start-up lock, like the real arm's brakes/libfranka): no fall at launch, no sag at a controller switch. Controllers do NOT compensate (controllers.yaml).
#   isaac:  the same as mujoco: Isaac Sim has gravity ON and nothing there compensates it, so the controllers compensate it, and the MuJoCo stand-in has gravity on too.
#           If Isaac's gravity is ever switched off, start with CONTROLLERS_FILE=controllers.yaml and FR3_SCENE=scene_fric_v1.xml (compensating a gravity that is off makes the arm rise).
#   real:   libfranka compensates gravity in the hardware: never use controllers_gravity.yaml there (double compensation).
# The OLD variants (scene.xml, scene_hand_v1.xml) are in archive/models_old_variants/: copy them back into models/ to use them.
DEFAULTS = {
    "mujoco": ("scene_fric_gravcomp_v1.xml", "controllers.yaml"),
    "isaac": ("scene_fric_gravity_v1.xml", "controllers_gravity.yaml"),
    "real": ("scene_fric_v1.xml", "controllers_real.yaml"),
}


def scene_name(mode="mujoco"):
    return os.environ.get("FR3_SCENE") or DEFAULTS.get(mode, DEFAULTS["mujoco"])[0]


def controllers_name(mode="mujoco"):
    return os.environ.get("CONTROLLERS_FILE") or DEFAULTS.get(mode, DEFAULTS["mujoco"])[1]


def render_robot_description(
    eff,
    robot_ip="172.16.0.2",
    arm_id="fr3",
    arm_prefix="",
    load_gripper="true",
    use_fake_hardware="true",
    fake_sensor_commands="false",
    hardware="",
    mode="mujoco",
):
    """hardware: "" (decided by use_fake_hardware: real or mujoco) or "isaac" (topic-based plugin)."""
    mujoco_model = str(eff.file("models", scene_name(mode)))
    mappings = {
        "ros2_control": "true",
        "arm_id": arm_id,
        "arm_prefix": arm_prefix,
        "robot_ip": robot_ip,
        "hand": load_gripper,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "mujoco_model": mujoco_model,
        "hardware": hardware,
        "isaac_joint_states_topic": os.environ.get("ISAAC_JOINT_STATES_TOPIC", "/isaac_joint_states"),
        "isaac_joint_commands_topic": os.environ.get("ISAAC_JOINT_COMMANDS_TOPIC", "/isaac_joint_commands"),
        "isaac_sum_wrapped_joint_states": os.environ.get("ISAAC_SUM_WRAPPED_JOINT_STATES", "false"),
        "isaac_effort_initial_value": os.environ.get("ISAAC_EFFORT_INITIAL_VALUE", "0.0"),
        "isaac_command_threshold": os.environ.get("ISAAC_COMMAND_THRESHOLD", ""),
    }
    return xacro.process_file(str(eff.file("models", "fr3_single.urdf.xacro")), mappings=mappings).toprettyxml(
        indent="  "
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--use-fake-hardware", default="true")
    ap.add_argument("--robot-ip", default="172.16.0.2")
    ap.add_argument("--hardware", default="", help='"isaac" renders the topic-based plugin')
    ap.add_argument("--mode", default="mujoco", help="mujoco, isaac or real: picks the default scene")
    args = ap.parse_args()
    eff = overlay.get_effective()
    print(eff.report({"scene": ("models", scene_name(args.mode)), "urdf": ("models", "fr3_single.urdf.xacro")}), file=__import__("sys").stderr)
    print(render_robot_description(eff, robot_ip=args.robot_ip, use_fake_hardware=args.use_fake_hardware, hardware=args.hardware, mode=args.mode))


if __name__ == "__main__":
    main()
