#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")

    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")

    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("stm32_udp"), "rviz", "stm32_udp.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for RPLIDAR.",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value="115200",
                description="Serial baudrate for RPLIDAR.",
            ),
            DeclareLaunchArgument(
                "lidar_frame_id",
                default_value="laser_frame",
                description="Frame id for RPLIDAR scans.",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_link",
                description="Base frame id for the robot.",
            ),
            DeclareLaunchArgument(
                "laser_x",
                default_value="0.0",
                description="Laser X offset (meters) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "laser_y",
                default_value="0.0",
                description="Laser Y offset (meters) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.15",
                description="Laser Z offset (meters) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "laser_roll",
                default_value="0.0",
                description="Laser roll (radians) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "laser_pitch",
                default_value="0.0",
                description="Laser pitch (radians) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "laser_yaw",
                default_value="0.0",
                description="Laser yaw (radians) relative to base_link.",
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="true",
                description="Whether to launch slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config file.",
            ),
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": lidar_frame_id,
                    }
                ],
                output="screen",
            ),
            Node(
                package="stm32_udp",
                executable="stm32_udp_odom",
                name="stm32_udp_odom",
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    laser_x,
                    laser_y,
                    laser_z,
                    laser_roll,
                    laser_pitch,
                    laser_yaw,
                    base_frame_id,
                    lidar_frame_id,
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(use_slam),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
