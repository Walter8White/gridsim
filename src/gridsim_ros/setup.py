from glob import glob

from setuptools import find_packages, setup

package_name = "gridsim_ros"

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
    maintainer="gridsim maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS 2 adapters and launch files for the gridsim MVP.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_state_publisher = gridsim_ros.sim_state_publisher:main",
            "sensor_noise_node = gridsim_ros.sensor_noise_node:main",
            "calibration_node = gridsim_ros.calibration_node:main",
        ],
    },
)
