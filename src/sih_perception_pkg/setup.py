from setuptools import setup, find_packages

setup(
    name="sih_perception",
    version="1.0.0",
    description="Tracked UGV Perception, 3D Depth Distance & Traversability Classification Pipeline",
    author="SIH Perception Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "scipy>=1.7.0",
        "opencv-python-headless>=4.5.0",
        "pyyaml>=5.4.0",
        "matplotlib>=3.4.0",
    ],
)
