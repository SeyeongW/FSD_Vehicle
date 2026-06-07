from setuptools import find_packages, setup

package_name = "waver_experiment_logger"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="waver",
    maintainer_email="you@example.com",
    description="Gazebo bird patrol mechanism event logger.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "seo_mechanism_trial_logger_node = waver_experiment_logger.seo_mechanism_trial_logger_node:main",
            "gazebo_bird_dataset_logger_node = waver_experiment_logger.gazebo_bird_dataset_logger_node:main",
        ],
    },
)
