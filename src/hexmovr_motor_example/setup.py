from glob import glob

from setuptools import find_packages, setup

package_name = "hexmovr_motor_example"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hexmovr02",
    maintainer_email="energetic2021@gmail.com",
    description="Example ROS2 package for controlling Hexmovr motor ID 1",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motor_demo = hexmovr_motor_example.motor_demo:main",
            "motor_direct_library = hexmovr_motor_example.motor_direct_library:main",
        ],
    },
)
