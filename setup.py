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
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
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
        'console_scripts': ["mapper_using_raw_odom = slam_bot.mapper_using_raw_odom:main",
                            "mapper_using_ground_truth = slam_bot.mapper_using_ground_truth:main",
                            "mapper_using_odom_with_icp = slam_bot.mapper_using_odom_with_icp:main",
                            "mapper_with_pgo = slam_bot.mapper_with_pgo:main",
                            "tf_merge_relay = slam_bot.tf_merge_relay:main",
        ],
    },
)
