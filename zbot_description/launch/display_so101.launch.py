"""Show the SO-101 in RViz.

  ros2 launch zbot_description display_so101.launch.py            # sliders drive the model
  ros2 launch zbot_description display_so101.launch.py gui:=false # something else publishes /joint_states
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("zbot_description"))
    urdf = (share / "urdf" / "so101.urdf").read_text()
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="joint sliders (true) or external /joint_states (false)"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(share / "rviz" / "so101.rviz")]),
    ])
