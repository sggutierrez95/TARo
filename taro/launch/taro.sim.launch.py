import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    my_robot_bringup_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_bringup"),
                "launch/my_robot_gazebo.launch.xml",
            )
        )
    )

    urdf_file = os.path.join(
                get_package_share_directory("my_robot_description"),
                "urdf/my_robot.urdf.xacro",
            )
    return LaunchDescription([
        my_robot_bringup_launch,
        Node(
            package="taro",
            executable="taro_node",
            output="log",
            arguments=["-id", "-1", "--imshow", "-urdf", urdf_file]
        )
    ])
