from setuptools import find_packages, setup

package_name = 'forest_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Forest UGV Team',
    maintainer_email='forest_ugv@example.com',
    description='Global A* and Reactive Local Path Planners for GPS-Denied Forest Navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'global_path_planner = forest_planner.global_path_planner:main',
            'reactive_local_planner = forest_planner.reactive_local_planner:main',
        ],
    },
)
