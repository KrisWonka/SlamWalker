from setuptools import find_packages, setup

package_name = 'slamwalker_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='kris_nano',
    maintainer_email='kris_nano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge_node = slamwalker_bridge.serial_bridge_node:main',
            'teleop = slamwalker_bridge.teleop_node:main',
        ],
    },
)
