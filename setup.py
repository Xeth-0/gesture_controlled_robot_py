from setuptools import find_packages, setup

package_name = 'gesture_controlled_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/' + package_name + '/hand_gesture_recognition_mediapipe',
         ['./gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/keypoint_classification.ipynb',
          './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/keypoint_classification_EN.ipynb',
          './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/point_history_classification.ipynb',
          ]),
        ('lib/python3.10/site-packages/' + package_name + '/hand_gesture_recognition_mediapipe/model/keypoint_classifier', [
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/keypoint_classifier/keypoint.csv',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/keypoint_classifier/keypoint_classifier.hdf5',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/keypoint_classifier/keypoint_classifier.tflite',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/keypoint_classifier/keypoint_classifier_label.csv'
        ]),
        ('lib/python3.10/site-packages/' + package_name + '/hand_gesture_recognition_mediapipe/model/point_history_classifier', [
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/point_history_classifier/point_history.csv',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/point_history_classifier/point_history_classifier.hdf5',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/point_history_classifier/point_history_classifier.tflite',
            './gesture_controlled_robot_py/hand_gesture_recognition_mediapipe/model/point_history_classifier/point_history_classifier_label.csv'
        ]),
    ],
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
            'listener = gesture_controlled_robot_py.movt_sub_node:main',
        ],
    },
)
