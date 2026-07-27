from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    basic_publisher = Node(
        package='slam_bot',
        executable='robot_status_publisher',
        name='robot_status_publisher'
    )
    basic_subscriber = Node(
        package='slam_bot',
        executable='robot_status_subscriber',
        name='robot_status_subscriber'
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(get_package_share_directory('slam_bot'), 'worlds', 'slam_bot_world.sdf')}.items(),
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'slam_bot',
            '-topic', 'robot_description',
            '-z', '0.05' # Spawning slightly above ground to avoid clipping
        ],
        output='screen'
    )  

    urdf_path = os.path.join(get_package_share_directory('slam_bot'), 'urdf', 'slam_bot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    joystick_config_path = os.path.join(get_package_share_directory('slam_bot'), 'config', 'joystick.yaml')
    joystick_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[joystick_config_path],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joystick_config_path]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_main',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(get_package_share_directory('slam_bot'), 'rviz', 'slam_bot.rviz')]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
    )

    slam_params_path = os.path.join(get_package_share_directory('slam_bot'), 'config', 'slam_params.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_path,
                          'use_sim_time': 'true'
                          }.items()
    )



    return LaunchDescription([
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD_PROVIDER', 'NVIDIA-G0'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES',
            '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
        ),
        #basic_publisher,
        #basic_subscriber,
        robot_state_publisher,
        rviz2,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        joy_node,
        joystick_node,
        slam_toolbox,
    ])