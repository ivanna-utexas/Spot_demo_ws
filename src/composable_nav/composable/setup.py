import os
from glob import glob

from setuptools import find_packages, setup


package_name = "composable_prefnav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob(os.path.join("launch", "*.launch.py"))),
        (f"share/{package_name}/config", glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools", "numpy", "omegaconf", "hydra-core", "scipy"],
    zip_safe=False,
    maintainer="SocialNavSUB",
    maintainer_email="todo@todo.com",
    description="ROS 2 diffusion waypoint wrapper for PrefNav.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diffusion_waypoint_node = composable_prefnav.diffusion_waypoint_node:main",
        ],
    },
)
