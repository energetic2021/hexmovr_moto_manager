from glob import glob

from setuptools import find_packages, setup

package_name = "hexmovr_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools", "python-can", "PyYAML"],
    zip_safe=True,
    maintainer="hexmovr02",
    maintainer_email="energetic2021@gmail.com",
    description="Pure Python ROS2 bridge for Hexmovr motors over SocketCAN",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "hexmovr_bridge = hexmovr_bridge.node:main",
        ],
    },
)
