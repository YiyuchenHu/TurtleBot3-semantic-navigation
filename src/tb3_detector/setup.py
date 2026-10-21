from setuptools import setup, find_packages
import os
from glob import glob

package_name = "tb3_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"),
         glob("models/*.md") + glob("models/*.pt")),
        # ROS 2 expects node executables in lib/<package_name>/  (libexec dir).
        # console_scripts go to bin/ so we also install a plain script wrapper here.
        (os.path.join("lib", package_name), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yiyuchen Hu",
    maintainer_email="yiyuchenhu@gmail.com",
    description="Stage-1 perception: YOLOv8 detector node for TB3 semantic navigation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = tb3_detector.detector_node:main",
            "la_detector_node = tb3_detector.la_detector_node:main",
        ],
    },
)
