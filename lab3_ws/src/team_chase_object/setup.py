# Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "team_chase_object"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student team — replace with both student names",
    maintainer_email="student@example.com",
    description="ROS 2 Lab 3: detect, locate, and follow a colored object.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detect_object = team_chase_object.detect_object:main",
            "get_object_range = team_chase_object.get_object_range:main",
            "chase_object = team_chase_object.chase_object:main",
        ],
    },
)
