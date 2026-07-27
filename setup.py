from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karburettor',
    maintainer_email='whykarthikwhy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ["robot_status_publisher = slam_bot.robot_status_publisher:main",
                            "robot_status_subscriber = slam_bot.robot_status_subscriber:main",
                            "self_mapper = slam_bot.self_mapping:main",
                            "self_mapper_pose = slam_bot.self_mapping_with_pose:main",
                            "self_mapper_icp = slam_bot.slam_bot_icp:main",
                            "self_mapper_pgo = slam_bot.slam_bot_pgo:main",
        ],
    },
)
