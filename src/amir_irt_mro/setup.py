from setuptools import setup
import os

package_name = 'amir_irt_mro'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/amir_irt_mro.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iwata',
    maintainer_email='your_email@example.com',
    description='Amir IRT-MRO multi-user MR robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amir_trajectory_node = amir_irt_mro.amir_trajectory_node:main',
            'amir_gripper = amir_irt_mro.amir_gripper:main',
            'conflict_detector = amir_irt_mro.conflict_detector:main',
            'info_filter = amir_irt_mro.info_filter:main',
            'amir_base_move = amir_irt_mro.amir_base_move:main',
            'amir_path_planner = amir_irt_mro.amir_path_planner:main',
            'amir_affine_transform = amir_irt_mro.amir_affine_transform:main',
            'amir_initial_position = amir_irt_mro.amir_initial_position:main',
            'amir_palm_ik_node = amir_irt_mro.amir_palm_ik_node:main',
            'amir_absolute_palm_ik_node = amir_irt_mro.amir_absolute_palm_ik_node:main',
            'amir_gripper_joint4_level_node = amir_irt_mro.amir_gripper_joint4_level_node:main',
            'realsense_bottle_pose_node = amir_irt_mro.realsense_bottle_pose_node:main',
            'amir_right_hand_mecanum_node = amir_irt_mro.amir_right_hand_mecanum_node:main',
        ],
    },
)
