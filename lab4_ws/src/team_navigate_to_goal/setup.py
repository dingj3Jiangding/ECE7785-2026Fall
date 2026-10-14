# Lab partners: Ding Jiang, Tongning Zhang.
from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "team_navigate_to_goal"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*.yaml") + glob("config/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ding Jiang, Tongning Zhang",
    maintainer_email="student@example.com",
    url="https://github.com/dingj3Jiangding/ECE7785-2026Fall",
    description="ROS 2 Lab 4: sequential waypoint navigation with LiDAR obstacle avoidance.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "navigate_to_goal = team_navigate_to_goal.navigate_to_goal:main",
    ]},
)
