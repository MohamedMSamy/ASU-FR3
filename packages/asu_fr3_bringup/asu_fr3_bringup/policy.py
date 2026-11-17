"""Gravity guard: who compensates gravity must be exactly one party, and never twice.

Called by the launch before anything starts. It reads the controllers file that will be loaded and the MuJoCo scene (and the models it includes):

  controllers file: `use_gravity_compensation: true` anywhere   -> the CONTROLLERS compensate gravity
  scene:            `gravcomp` on any body                      -> the SIMULATOR compensates gravity
  scene:            the gravity vector of `<option gravity=...>` (MuJoCo's default is -9.81 when no option sets it)
  real arm:         libfranka compensates gravity in the robot's hardware, so the controllers must NOT

Refused (RuntimeError, nothing is started):
  real      with the flag on                          (double compensation: the arm is pushed up by the full gravity torque at activation)
  mujoco / isaac stand-in:
            flag on  and a scene with gravcomp        (both compensate)
            flag on  and a scene with gravity off     (the arm rises)
            flag off and gravity on and no gravcomp   (nothing compensates: the arm falls)
ASU_FR3_ALLOW_GRAVITY_MISMATCH=1 turns the refusals of the simulator modes into a warning (for experiments on purpose). It never applies to `real`.
"""
import os
import xml.etree.ElementTree as ET

import yaml


def controllers_flag(path):
    """True if `use_gravity_compensation` is true anywhere in the controllers file."""
    with open(path) as fh:
        data = yaml.safe_load(fh)

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == "use_gravity_compensation" and bool(v):
                    return True
                if walk(v):
                    return True
        elif isinstance(node, list):
            return any(walk(v) for v in node)
        return False

    return walk(data)


def scene_gravity(eff, scene_name, _depth=0):
    """(gravity vector, has_gravcomp) of a scene: the last `<option gravity>` in document order wins; MuJoCo's default is (0, 0, -9.81)."""
    gravity, gravcomp = None, False
    root = ET.parse(str(eff.file("models", scene_name))).getroot()
    for el in root.iter():
        if el.tag == "option" and "gravity" in el.attrib:
            gravity = [float(x) for x in el.attrib["gravity"].split()]
        elif el.tag == "body" and float(el.attrib.get("gravcomp", "0") or 0) != 0.0:
            gravcomp = True
        elif el.tag == "include" and _depth < 4:
            g, c = scene_gravity(eff, el.attrib["file"], _depth + 1)
            gravity = g if g is not None else gravity
            gravcomp = gravcomp or c
    return gravity, gravcomp


def check(mode, controllers_path, scene_name, eff):
    """Return a one-line summary of who owns gravity, or raise RuntimeError."""
    flag = controllers_flag(controllers_path)
    name = os.path.basename(str(controllers_path))
    if mode == "real":
        if flag:
            raise RuntimeError(
                f"{name} has use_gravity_compensation: true and mode is real. libfranka already compensates gravity in the robot: the arm would be pushed up "
                "by the full gravity torque at activation. Use controllers_real.yaml (or any file without that flag).")
        return f"[asu_fr3] gravity owner: libfranka (hardware); {name} does not compensate"

    gravity, gravcomp = scene_gravity(eff, scene_name)
    g_on = gravity is None or sum(x * x for x in gravity) > 1e-12
    problem = None
    if flag and gravcomp:
        problem = f"{name} compensates gravity (use_gravity_compensation: true) AND {scene_name} compensates it too (gravcomp): double compensation"
    elif flag and not g_on:
        problem = f"{name} compensates gravity but {scene_name} has gravity off: the arm would rise"
    elif not flag and g_on and not gravcomp:
        problem = f"gravity is on in {scene_name} but nothing compensates it ({name} has no use_gravity_compensation, the scene has no gravcomp): the arm would fall"
    if problem:
        if os.environ.get("ASU_FR3_ALLOW_GRAVITY_MISMATCH") == "1":
            return f"[asu_fr3] WARNING (ASU_FR3_ALLOW_GRAVITY_MISMATCH=1): {problem}"
        raise RuntimeError(problem + ". Pick a consistent pair (see models/README.md), or set ASU_FR3_ALLOW_GRAVITY_MISMATCH=1 for an experiment on purpose.")
    owner = "the controllers (use_gravity_compensation)" if flag else ("the simulator (gravcomp)" if gravcomp else "nobody needed: gravity is off")
    return f"[asu_fr3] gravity owner: {owner}; scene {scene_name}, controllers {name}"
