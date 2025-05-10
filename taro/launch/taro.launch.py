import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    robot_description = PathJoinSubstitution([
                            FindPackageShare("my_robot_description"),
                            "urdf",
                            "my_robot.urdf.xacro"])

    robot_state_publisher_params = {
        'robot_description': 
            ParameterValue(Command(['xacro ', robot_description]), value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_state_publisher_params],
    )

    gazebo_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ])
            ]),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([
                    FindPackageShare("my_robot_bringup"),
                    "worlds",
                    "test_world.sdf"
                ]), ' -r'],
            }.items()
        )

    ros_gz_sim_node_params = {
        "-topic":"robot_description"
    }

    ros_gz_sim_node = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[ros_gz_sim_node_params],
    )

    ros_gz_bridge_params = {
        "config_file":
            PathJoinSubstitution([
                    FindPackageShare("my_robot_bringup"),
                    "config",
                    "gazebo_bridge.yaml"
            ])
    }
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[ros_gz_bridge_params],
    )

    ################ urdf_config_rviz ################
    urdf_config_rviz = PathJoinSubstitution([
                    FindPackageShare("my_robot_description"),
                    "rviz",
                    "urdf_config.rviz"
            ])

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot", package_name="my_robot_description"
        )
        .robot_description(file_path='urdf/my_robot.urdf.xacro')
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .planning_scene_monitor(
        #     publish_robot_description=True, publish_robot_description_semantic=True
        # )
        # .planning_pipelines(
        #     pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        # )
        .to_moveit_configs()
    )


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d' , urdf_config_rviz],
        parameters=[moveit_config.robot_description]
    )


    taro_node = Node(
                package="taro",
                executable="taro_node",
                output="log",
                arguments=["-id", "0", "--imshow"]
    )


    return LaunchDescription(
        [
            taro_node
        ]
    )
