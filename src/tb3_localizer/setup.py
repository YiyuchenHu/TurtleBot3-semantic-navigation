from setuptools import setup, find_packages
import os
from glob import glob

package_name = "tb3_localizer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yiyuchen Hu",
    maintainer_email="yiyuchenhu@gmail.com",
    description="Stage-2 perception: LiDAR + camera fusion planar localizer for TB3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localizer_node = tb3_localizer.localizer_node:main",
        ],
    },
)
