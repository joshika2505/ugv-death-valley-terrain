from setuptools import find_packages, setup

package_name = 'forest_perception'

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
    description='Real-Time Deep Learning Perception for Forest UGV Traversability and Hazard Detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = forest_perception.perception_node:main',
            'camera_viewer_gui = forest_perception.camera_viewer_gui:main',
        ],
    },
)
