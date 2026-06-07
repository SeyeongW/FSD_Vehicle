from setuptools import find_packages, setup

package_name = "waver_seo_tracking"

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
    description="Gazebo-only SEO tracking adapters for Waver bird patrol.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "seo_cluster_tracker_node = waver_seo_tracking.seo_cluster_tracker_node:main",
            "seo_bird_yolo_node = waver_seo_tracking.seo_bird_yolo_node:main",
            "seo_fake_bird_classification_node = waver_seo_tracking.seo_bird_yolo_node:main",
            "seo_camera_tilt_joint_node = waver_seo_tracking.seo_camera_tilt_joint_node:main",
            "seo_observation_body_tracker_node = waver_seo_tracking.seo_observation_body_tracker_node:main",
            "mission_state_cmd_selector_node = waver_seo_tracking.mission_state_cmd_selector_node:main",
            "remote_ui_patrol_adapter_node = waver_seo_tracking.remote_ui_patrol_adapter_node:main",
            "scan_alias_node = waver_seo_tracking.scan_alias_node:main",
        ],
    },
)
