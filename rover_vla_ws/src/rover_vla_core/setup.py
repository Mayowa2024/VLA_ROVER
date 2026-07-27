from setuptools import find_packages, setup
from glob import glob
import os

package_name = "rover_vla_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Mayowa",
    maintainer_email="maintainer@example.com",
    description=(
        "ROS 2 scaffolding for rover VLA data collection, safety, "
        "teleoperation and motor integration."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "instruction_publisher = rover_vla_core.instruction_publisher:main",
            "safety_filter = rover_vla_core.safety_filter:main",
            "fake_odom = rover_vla_core.fake_odom:main",
            "encoder_odom = rover_vla_core.encoder_odom:main",
            "mock_motor_driver = rover_vla_core.mock_motor_driver:main",
            "frame_recorder = rover_vla_core.frame_recorder:main",
            "teleop_adapter = rover_vla_core.teleop_adapter:main",
            "g29_udp_sender = rover_vla_core.g29_udp_sender:main",
            "serial_motor_driver = rover_vla_core.serial_motor_driver:main",
        ],
    },
)
