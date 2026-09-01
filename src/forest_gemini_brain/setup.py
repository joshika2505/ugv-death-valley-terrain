from setuptools import find_packages, setup

package_name = 'forest_gemini_brain'

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
    description='Google Gemini Multimodal Vision-Language-Action Brain for GPS-Denied Forest Autonomous UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gemini_brain_node = forest_gemini_brain.gemini_brain_node:main',
        ],
    },
)
