from setuptools import setup
import os
from glob import glob

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'lane_detector_node = lane_detection.lane_detector_node:main',
            'diagnose_camera    = lane_detection.diagnose_camera:main',
            'respawn_robot      = lane_detection.respawn_robot:main',
        ],
    },
)
