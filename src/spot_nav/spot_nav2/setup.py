from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'spot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('spot_nav/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('spot_nav/config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('spot_nav/maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('spot_nav/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Tsoi',
    maintainer_email='nathan@vertile.com',
    description='Spot Navigation Stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = spot_nav.twist_to_ackermann:main',
            'simple_map_server = spot_nav.simple_map_server:main',
            'soft_stop = spot_nav.soft_stop:main',
        ],
    },
)
