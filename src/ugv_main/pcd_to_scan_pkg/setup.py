from setuptools import setup

package_name = 'pcd_to_scan_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seo',
    maintainer_email='seo@example.com',
    description='Convert PointCloud2 from Livox MID-360 to LaserScan for 2D SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_to_laserscan_node = pcd_to_scan_pkg.pointcloud_to_laserscan_node:main',
        ],
    },
)