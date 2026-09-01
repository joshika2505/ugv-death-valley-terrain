from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'forest_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
        (os.path.join('share', package_name, 'web', 'css'), glob('web/css/*')),
        (os.path.join('share', package_name, 'web', 'js'), glob('web/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Forest UGV Team',
    maintainer_email='forest_ugv@example.com',
    description='HERCULES-Inspired Mission Control Web Dashboard for Gazebo Forest Autonomous UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard_server = forest_dashboard.dashboard_server:main',
        ],
    },
)
