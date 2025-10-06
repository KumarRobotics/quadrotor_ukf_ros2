from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments (with default values)
    launch_prefix = LaunchConfiguration('launch_prefix', default='')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    imu_rotated_frame_id = LaunchConfiguration('imu_rotated_frame_id', default='zed_camera_link')
    base_link = LaunchConfiguration('base_link', default='zed_camera_link')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='zed_imu_link')

    return LaunchDescription([

        DeclareLaunchArgument('launch_prefix', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('odom_frame_id', default_value=TextSubstitution(text='odom')),
        DeclareLaunchArgument('imu_rotated_frame_id', default_value=TextSubstitution(text='zed_camera_link')),
        DeclareLaunchArgument('base_link', default_value=TextSubstitution(text='zed_camera_link')),
        DeclareLaunchArgument('imu_frame_id', default_value=TextSubstitution(text='zed_imu_link')),

        Node(
            package='quadrotor_ukf_ros2',
            executable='quadrotor_ukf_ros2',
            name='quadrotor_ukf_ros2',
            output='screen',
            prefix=launch_prefix,  # this is equivalent to launch-prefix in ROS1
            parameters=[{
                'odom_frame_id': odom_frame_id,
                'imu_rotated_frame_id': imu_rotated_frame_id,
                'imu_frame_id': imu_frame_id,
                'base_link': base_link
            }],
            remappings=[
                ('odom', '/zed/zed_node/odom'),
                ('imu', '/zed/zed_node/imu/data'),
            ],
        ),
    ])

