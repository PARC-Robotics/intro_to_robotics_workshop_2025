import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'intro_to_robotics_workshop_2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*"))
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*rviz"))
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*sdf"))
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinedu Amadi',
    maintainer_email='noobinventor@tutamail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "workshop_basic_node = intro_to_robotics_workshop_2025.workshop_basic_node:main"
        ],
    },
)
