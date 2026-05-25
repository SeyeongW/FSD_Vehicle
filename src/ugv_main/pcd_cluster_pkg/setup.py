from setuptools import find_packages, setup

package_name = 'pcd_cluster_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cluster_node = pcd_cluster_pkg.cluster_node:main',
            'cluster_node_backup = pcd_cluster_pkg.cluster_node_backup:main',
        ],
    },
)
