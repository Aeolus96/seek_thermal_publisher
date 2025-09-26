from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_raw = LaunchConfiguration("topic_raw", default="/seek/radiometric_raw")
    topic_view = LaunchConfiguration("topic_view", default="/seek/auto_scaled")
    frame_id = LaunchConfiguration("frame_id", default="seek_camera_optical")
    percentile_clip = LaunchConfiguration("percentile_clip", default="2.0")

    return LaunchDescription(
        [
            DeclareLaunchArgument("topic_raw", default_value="/seek/radiometric_raw"),
            DeclareLaunchArgument("topic_view", default_value="/seek/auto_scaled"),
            DeclareLaunchArgument("frame_id", default_value="seek_camera_optical"),
            DeclareLaunchArgument("percentile_clip", default_value="2.0"),
            Node(
                package="seek_thermal_publisher",
                executable="thermal_publisher",
                name="seek_thermal_publisher",
                output="screen",
                parameters=[
                    {"topic_raw": topic_raw},
                    {"topic_view": topic_view},
                    {"frame_id": frame_id},
                    {"percentile_clip": percentile_clip},
                ],
            ),
        ]
    )
