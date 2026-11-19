from glob import glob
from setuptools import setup

setup(
    name="robocon_fusion", version="0.1.0", packages=["robocon_fusion"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/robocon_fusion"]),
                ("share/robocon_fusion", ["package.xml"]),
                ("share/robocon_fusion/launch", glob("launch/*.launch.py")),
                ("share/robocon_fusion/config", glob("config/*.yaml"))],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Robocon Team", maintainer_email="team@example.invalid",
    description="Isolated fusion replay", license="Apache-2.0",
    entry_points={"console_scripts": [
        "livo_rgb_rectifier = robocon_fusion.rgb_rectifier:main",
        "slam_bench = robocon_fusion.bench_app:main",
        "slam_bench_session = robocon_fusion.bench_session:main"]},
)
