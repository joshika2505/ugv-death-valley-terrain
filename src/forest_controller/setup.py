from setuptools import find_packages, setup

package_name = 'forest_controller'

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
    description='Motor Velocity Controller & Command Interface for Forest UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_controller_node = forest_controller.motor_controller_node:main',
        ],
    },
)
