from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "px4_swarm_control"
config_files = [path for path in glob("config/*") if Path(path).is_file()]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", config_files),
        (
            f"share/{package_name}/config/px4_speed_profiles",
            glob("config/px4_speed_profiles/*"),
        ),
        (f"share/{package_name}/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuanyifan",
    maintainer_email="yuanyifan@example.com",
    description="Python ROS 2 control package for the PX4 swarm control system.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "check_live_px4_gz_bridge = px4_swarm_control.live_bridge_smoke:main",
            "ground_station_node = px4_swarm_control.ground_station_node:main",
            "operator_console = px4_swarm_control.operator_console:main",
            "px4_speed_profile = px4_swarm_control.px4_speed_profile:main",
            "vehicle_node = px4_swarm_control.vehicle_node:main",
        ],
    },
)
