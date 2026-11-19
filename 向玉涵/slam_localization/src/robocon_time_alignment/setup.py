from setuptools import find_packages, setup

package_name = "robocon_time_alignment"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robocon Team",
    maintainer_email="team@example.invalid",
    description="Sensor time alignment adapters for the Robocon BR stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "d435i_time_align_node = robocon_time_alignment.d435i_time_align_node:main",
            "mid360_stamp_monitor_node = robocon_time_alignment.mid360_stamp_monitor_node:main",
            "mid360_clock_query_node = robocon_time_alignment.mid360_clock_query_node:main",
        ]
    },
)
