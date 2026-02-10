from setuptools import find_packages, setup

package_name = 'perception_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jin Wei Lim',
    maintainer_email='jin@example.com',
    description='YOLO-based object detection for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = perception_demo.nodes.camera:main',
            'depth_node = perception_demo.nodes.depth:main',
            'detector_node = perception_demo.nodes.detector:main',
            'tracker_node = perception_demo.nodes.tracker:main',
            'teleop_node = perception_demo.nodes.teleop:main',
            'object_estimator_node = perception_demo.nodes.object_estimator:main',
            'grasp_node = perception_demo.nodes.grasp:main',
            'viz_node = perception_demo.nodes.viz:main',
        ],
    },
)
