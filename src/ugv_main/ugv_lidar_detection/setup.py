from setuptools import setup
import os
from glob import glob

package_name = 'ugv_lidar_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'scikit-learn', 'numpy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='4D LiDAR Object Detection package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_detector_node = ugv_lidar_detection.lidar_detector_node:main'
        ],
    },
)
