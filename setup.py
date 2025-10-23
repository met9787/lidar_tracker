from setuptools import find_packages, setup
import os
import glob

package_name = 'lidar_tracker'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gjohny',
    maintainer_email='gjohny@tamu.edu',
    description='People detection and tracking from lidar data..',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = lidar_tracker.tracker_node:main',
            'scan_processor = lidar_tracker.scan_processor:main',
        ],
    },
)
