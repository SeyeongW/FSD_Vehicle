from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_tools'


def package_data_files(directory):
    data_files = []
    if not os.path.isdir(directory):
        return data_files
    for root, _, files in os.walk(directory):
        file_paths = [os.path.join(root, filename) for filename in files]
        if file_paths:
            data_files.append((os.path.join('share', package_name, root), file_paths))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py')),
        ),
    ]
    + package_data_files('models')
    + package_data_files('worlds')
    + package_data_files('docs')
    + package_data_files('config')
    + package_data_files('waypoints'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_ctrl = ugv_tools.keyboard_ctrl:main',
            'joy_ctrl = ugv_tools.joy_ctrl:main',
            'behavior_ctrl = ugv_tools.behavior_ctrl:main',
            'waver_gazebo_patrol = ugv_tools.waver_gazebo_patrol:main',
            'waver_cmd_vel_serial_bridge = ugv_tools.waver_cmd_vel_serial_bridge:main',
            'waver_remote_panel = ugv_tools.waver_remote_panel:main',
        ],
    },
)
