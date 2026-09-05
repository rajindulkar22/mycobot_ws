import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mycobot_sim_projects'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    "console_scripts": [
        "pose_sequence = mycobot_sim_projects.pose_sequence:main",
        "keyboard_control = mycobot_sim_projects.keyboard_control:main",
        "joint_monitor = mycobot_sim_projects.joint_monitor:main",
        "tf_explorer = mycobot_sim_projects.tf_explorer:main",
        "gazebo_pose_commander = mycobot_sim_projects.gazebo_pose_commander:main",
        "gripper_commander = mycobot_sim_projects.gripper_commander:main",
        "manipulation_state_machine = mycobot_sim_projects.manipulation_state_machine:main",
        "manipulation_state_machine_ui = mycobot_sim_projects.manipulation_state_machine_ui:main",
        "sim_workbench_ui = mycobot_sim_projects.sim_workbench_ui:main",
    ],
},
)
