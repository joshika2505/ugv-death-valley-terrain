from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sih_ugv_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIH Team',
    maintainer_email='sih@example.com',
    description='Visual SLAM, Sensor Fusion, and Autonomous Navigation Stack for SIH Outdoor UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'autonomous_navigator = sih_ugv_navigation.autonomous_navigator:main',
            'locality_mission_node = sih_ugv_navigation.locality_mission_node:main',
            'point_ab_mission_coordinator = sih_ugv_navigation.point_ab_mission_coordinator:main',
            'dynamic_obstacle_node = sih_ugv_navigation.dynamic_obstacle_node:main',
            'hospital_marker_publisher = sih_ugv_navigation.hospital_marker_publisher:main',
        ],
    },
)
