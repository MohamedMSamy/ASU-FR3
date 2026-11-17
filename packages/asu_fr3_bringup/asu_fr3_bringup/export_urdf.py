"""Export a plain URDF of the robot for other tools (Isaac Sim, MoveIt, ...), rendered from OUR xacro.

Why: the controller and the simulator must share one robot model. This renders models/fr3_single.urdf.xacro through the
config overlay (so your own models/ files are used), drops the <ros2_control> section (not needed outside ros2_control),
and makes the file self-contained: the meshes it references are copied next to it and the paths made relative.

  python3 -m asu_fr3_bringup.export_urdf [--out DIR] [--keep-ros2-control] [--no-meshes]
Default output: /home/ros/data/urdf (the mounted ./data/urdf), file fr3.urdf, meshes in franka_description/.
"""
import argparse
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from asu_fr3_bringup import description, overlay

PREFIX = "package://franka_description/"


def export(out_dir, keep_ros2_control=False, copy_meshes=True):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    eff = overlay.get_effective()
    root = ET.fromstring(description.render_robot_description(eff, use_fake_hardware="true"))
    if not keep_ros2_control:
        for tag in root.findall("ros2_control"):
            root.remove(tag)
    n_meshes = 0
    if copy_meshes:
        share = Path(get_package_share_directory("franka_description"))
        for mesh in root.iter("mesh"):
            name = mesh.get("filename", "")
            if name.startswith(PREFIX):
                rel = name[len(PREFIX):]
                dest = out / "franka_description" / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                if not dest.exists():
                    shutil.copy2(share / rel, dest)
                mesh.set("filename", f"franka_description/{rel}")
                n_meshes += 1
    ET.indent(root, space="  ")
    path = out / "fr3.urdf"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path, n_meshes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(os.environ.get("FR3_LOG_DIR", "/home/ros/data/logs"), "..", "urdf"))
    ap.add_argument("--keep-ros2-control", action="store_true", help="keep the <ros2_control> section")
    ap.add_argument("--no-meshes", action="store_true", help="leave package:// mesh paths as they are")
    a = ap.parse_args()
    path, n = export(os.path.normpath(a.out), a.keep_ros2_control, not a.no_meshes)
    print(f"[export_urdf] wrote {path} ({n} mesh references resolved)")


if __name__ == "__main__":
    main()
