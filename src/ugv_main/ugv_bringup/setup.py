from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chotaehyun',
    maintainer_email='whxogus4793@naver.com',
    description='Sensor and legacy base bringup launch files for the Waver UGV platform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv_bringup = ugv_bringup.ugv_bringup:main',
            'ugv_driver = ugv_bringup.ugv_driver:main',
        ],
    },
)
