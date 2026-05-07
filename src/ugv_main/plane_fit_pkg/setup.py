from setuptools import setup

package_name = 'plane_fit_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plane_fit.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/plane_fit_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RANSAC and Least Squares plane fitting for PointCloud2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plane_fit_node = plane_fit_pkg.plane_fit_node:main',
        ],
    },
)
