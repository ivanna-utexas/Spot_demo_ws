import os
from glob import glob
from setuptools import setup

package_name = 'spot_mapping_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'best_effort_topic_relay_node = spot_mapping_common.best_effort_topic_relay_node:main',
            'pointcloud_validator_node = spot_mapping_common.pointcloud_validator_node:main',
            'spot_imu_adapter_node = spot_mapping_common.spot_imu_adapter_node:main',
            'imu_normalizer_node = spot_mapping_common.imu_normalizer_node:main',
            'scan_extractor_node = spot_mapping_common.scan_extractor_node:main',
            'timing_monitor_node = spot_mapping_common.timing_monitor_node:main',
        ],
    },
)
