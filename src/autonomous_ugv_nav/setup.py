import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomous_ugv_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Mithul',
    maintainer_email='mithul@example.com',
    description='Production-Grade Autonomous UGV Navigation Stack for GPS-Denied and EMCON Unstructured Outdoor Terrains',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_costmap_node = autonomous_ugv_nav.costmap.semantic_costmap_node:main',
            'mppi_controller_node = autonomous_ugv_nav.planner.mppi_controller_node:main',
            'global_planner_node = autonomous_ugv_nav.planner.global_planner_node:main',
            'ekf_state_estimator_node = autonomous_ugv_nav.estimator.ekf_state_estimator_node:main',
            'safety_monitor_node = autonomous_ugv_nav.safety.safety_monitor_node:main',
        ],
    },
)
