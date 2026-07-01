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
    maintainer='Waver project maintainers',
    maintainer_email='whxogus4793@naver.com',
    description='LiDAR object detection utilities for Waver UGV perception experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_detector_node = ugv_lidar_detection.lidar_detector_node:main'
        ],
    },
)
