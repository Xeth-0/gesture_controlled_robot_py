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
