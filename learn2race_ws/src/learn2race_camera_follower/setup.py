from glob import glob
import os

from setuptools import find_packages, setup

package_name = "learn2race_camera_follower"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='Ruben Pereira',
    maintainer_email='1181915@isep.ipp.pt',
    description="Robust camera-based line follower for Learn2Race.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_follower_node = learn2race_camera_follower.line_follower_node:main",
            "line_detection_preview_node = learn2race_camera_follower.line_detection_preview_node:main",
        ],
    },
)
