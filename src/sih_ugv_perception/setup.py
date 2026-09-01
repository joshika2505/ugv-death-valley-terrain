from setuptools import find_packages, setup

package_name = 'sih_ugv_perception'

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
    maintainer='SIH Team',
    maintainer_email='sih@example.com',
    description='Deep Learning and Computer Vision Perception for Outdoor UGV Path Segmentation and Beacon Tracking',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_segmentation_node = sih_ugv_perception.path_segmentation_node:main',
            'visual_beacon_detector = sih_ugv_perception.visual_beacon_detector:main',
        ],
    },
)
