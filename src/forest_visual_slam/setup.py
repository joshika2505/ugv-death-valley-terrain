from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'forest_visual_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Forest UGV Team',
    maintainer_email='forest_ugv@example.com',
    description='GPS-Free Visual Odometry, SLAM, and Sensor Fusion for Forest UGV Navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visual_odometry_node = forest_visual_slam.visual_odometry_node:main',
        ],
    },
)
