from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocolumbus_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikew',
    maintainer_email='mikew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = robocolumbus_stuff.teleop_node:main',
            'imu_gps_node = robocolumbus_stuff.imu_gps_node:main',
            'cone_node = robocolumbus_stuff.cone_node:main',
            'nav_node = robocolumbus_stuff.nav_node:main',
            'speaker_node = robocolumbus_stuff.speaker_node:main',
            'controller_node = robocolumbus_stuff.controller_node:main',
            'pcd_node = robocolumbus_stuff.pcd_node:main',
        ],
    },
)
