from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gesture_controlled_robot_py'

files = glob('models/*/*.urdf.xacro')
for file in files:
    cmd = 'ros2 run xacro xacro '+file+' > ' + file.replace('.xacro', '.urdf')
    os.system(cmd)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*/*.urdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    package_data={
        # '': ['package.xml'],
        package_name: [
            'hand_gesture_recognition_mediapipe/*',
            'hand_gesture_recognition_mediapipe/*/*.ipynb',
            'hand_gesture_recognition_mediapipe/model/keypoint_classifier/*',
            'hand_gesture_recognition_mediapipe/model/point_history_classifier/*',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xetho',
    maintainer_email='tediberhanu777@gmail.com',
    description='Simple wheeled robot with gesture recognition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = gesture_controlled_robot_py.movt_pub_node:main',
            'listener = gesture_controlled_robot_py.gesture_listener_node:main',
            'object_avoidance = gesture_controlled_robot_py.object_avoidance:main',
            'trajectory_subscriber = gesture_controlled_robot_py.trajectory_subscriber:main',
            'trajectory_publisher = gesture_controlled_robot_py.trajectory_publisher:main'
        ],
    },
)
