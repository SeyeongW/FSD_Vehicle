from setuptools import find_packages, setup

package_name = 'lidar_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seo',
    maintainer_email='seoyu0207@naver.com',
    description='Fixed multi-LiDAR point cloud fusion into a common frame',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_fusion_node = lidar_fusion_pkg.lidar_fusion_node:main',
        ],
    },
)
