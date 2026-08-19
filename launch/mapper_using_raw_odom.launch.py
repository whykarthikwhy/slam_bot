from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def make_slam_and_rviz(ns, executable, rviz_config):
    slam_node = Node(
        package='slam_bot',
        executable=executable,
        name=f'mapper_{ns}',
        namespace=ns,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/map', f'/{ns}/map'),
            ('/odom', '/odom'),   
            ('/scan', '/scan'),   
        ],
    )
    tf_merge = Node(
        package='slam_bot',
        executable='tf_merge_relay',  
        name=f'tf_merge_{ns}',
        parameters=[{
            'use_sim_time': True,
            'input_topics': ['/tf', f'/{ns}/tf'],
            'output_topic': f'/{ns}/tf_merged',
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=f'rviz2_{ns}',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d', os.path.join(get_package_share_directory('slam_bot'), 'rviz', rviz_config),
            '--ros-args',
            '-r', f'tf:=/{ns}/tf_merged',
            '-r', 'tf_static:=/tf_static',
        ],
    )
    return GroupAction([slam_node, tf_merge, rviz_node])

def generate_launch_description():

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(
            get_package_share_directory('slam_bot'), 'worlds', 'slam_bot_world.sdf')}.items(),
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'slam_bot', '-topic', 'robot_description',
                   '-x', '1.6', '-y', '-1.2', '-z', '1.0'],
        output='screen'
    )
    urdf_path = os.path.join(get_package_share_directory('slam_bot'), 'urdf', 'slam_bot_mapping.urdf')
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
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/slam_bot/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )
    raw_odom = make_slam_and_rviz('raw_odom', 'mapper_using_raw_odom', 'slam_bot_raw_odom.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD_PROVIDER', 'NVIDIA-G0'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                                '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        joy_node,
        joystick_node,
        raw_odom,
    ])