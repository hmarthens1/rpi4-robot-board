from setuptools import setup

package_name = "robot_status"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Publishes battery, ultrasonic range and system status of a fleet robot",
    license="TODO: License declaration",
    entry_points={"console_scripts": [
        "status_node = robot_status.status_node:main",
        "command_node = robot_status.command_node:main",
    ]},
)
