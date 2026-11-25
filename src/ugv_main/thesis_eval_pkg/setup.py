import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'thesis_eval_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models', 'bird_0p5'), glob('models/bird_0p5/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seo',
    maintainer_email='seoyu0207@naver.com',
    description='졸업논문 단일/다중 LiDAR 비교 실험 패키지',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ft_fusion_node = thesis_eval_pkg.ft_fusion_node:main',
            'eval_logger_node = thesis_eval_pkg.eval_logger_node:main',
        ],
    },
)
