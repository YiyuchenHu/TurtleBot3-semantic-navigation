from setuptools import setup, find_packages
import os
from glob import glob

package_name = "tb3_coordinator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yiyuchen Hu",
    maintainer_email="yiyuchenhu@gmail.com",
    description="Lightweight coordinator for TB3 semantic navigation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coordinator_node = tb3_coordinator.coordinator_node:main",
            "semantic_memory_marker_node = tb3_coordinator.semantic_memory_marker_node:main",
            "semantic_map_memory_node = tb3_coordinator.semantic_map_memory_node:main",
            "semantic_runtime_debug_node = tb3_coordinator.semantic_runtime_debug_node:main",
        ],
    },
)
