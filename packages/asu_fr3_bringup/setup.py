from setuptools import find_packages, setup
import os
from glob import glob

package_name = "asu_fr3_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("config/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed M Samy",
    maintainer_email="mohamednessim15@gmail.com",
    description="ASU-FR3 bringup for the CRISP controllers on the Franka FR3",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_publisher = asu_fr3_bringup.target_publisher:main",
            "crisp_py_franka_hand_adapter = asu_fr3_bringup.crisp_py_franka_hand_adapter:main",
            "mujoco_isaac_bridge = asu_fr3_bringup.mujoco_isaac_bridge:main",
            "target_viz = asu_fr3_bringup.target_viz:main",
        ],
    },
)
