from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slamwalker_explore'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KrisWonka',
    maintainer_email='wl2464649623@gmail.com',
    description='Locomotion-aware frontier exploration for SlamWalker (Phase 1).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer = slamwalker_explore.frontier_explorer_node:main',
            'session_manager = slamwalker_explore.session_manager_node:main',
            'scan_resampler = slamwalker_explore.scan_resampler_node:main',
        ],
    },
)
