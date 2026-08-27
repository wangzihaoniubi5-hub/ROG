import os
from glob import glob
from setuptools import setup

package_name = 'lidar_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # 只保留 package.xml
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@example.com',
    description='Lidar processing package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'super_node = lidar_processing.super_node:main',
            'fusion_node = lidar_processing.fusion_node:main',
            'custom_to_pointcloud2 = lidar_processing.custom_to_pointcloud2:main',
        ],
    },
)
