from setuptools import find_packages, setup

package_name = "robocon_localization"

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
    description="Localization quality adapters for the Robocon BR stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lio_quality_node = robocon_localization.lio_quality_node:main",
            "lio_session_supervisor = robocon_localization.lio_session_supervisor:main",
            "auto_relocalize_node = robocon_localization.auto_relocalize_node:main",
        ]
    },
)
