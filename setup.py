import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ai_msc_robotics_assignments_ros"

install_requires = []

setup(
    name=package_name,
    version="1.0.1",
    url="https://github.com/gstavrinos/ai_msc_robotics_assignments_ros",
    author="George Stavrinos",
    author_email="gstavrinos@protonmail.com",
    description="Doom can run anywhere. Now it runs on ROS too.",
    license="GPL-2.0",
    packages=find_packages(),
    install_requires=install_requires,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    entry_points={
        "console_scripts": [
            "robot_sim.py = robot_sim.robot_sim:main",
            "mp4_to_image_msg.py = mp4_to_image_msg.mp4_to_image_msg:main",
        ],
    },
)
