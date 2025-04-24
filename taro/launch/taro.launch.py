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


    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #     ],
    # )

    # # Define xacro mappings for the robot description file
    # launch_arguments = {
    #     "robot_ip": "xxx.yyy.zzz.www",
    #     "use_fake_hardware": "true",
    #     "gripper": "robotiq_2f_85",
    #     "dof": "7",
    # }

    # # Load the robot configuration
    # moveit_config = (
    #     MoveItConfigsBuilder(
    #         "my_robot", package_name="my_robot_description"
    #     )
    #     .robot_description(mappings=launch_arguments)
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .planning_pipelines(
    #         pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
    #     )
    #     .to_moveit_configs()
    # )

    # # Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config.to_dict()],
    # )

    # # RViz for visualization
    # # Get the path to the RViz configuration file
    # rviz_config_arg = DeclareLaunchArgument(
    #     "rviz_config",
    #     default_value="kinova_moveit_config_demo.rviz",
    #     description="RViz configuration file",
    # )
    # rviz_base = LaunchConfiguration("rviz_config")
    # rviz_config = PathJoinSubstitution(
    #     [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    # )

    # # Launch RViz
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #     ],
    # )

    # # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    # )

    # # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )

    # # ros2_control using mock hardware for trajectory execution
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="both",
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

    # hand_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    # )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            gazebo_sim_launch,
            ros_gz_sim_node,
            ros_gz_bridge_node,
            rviz2_node
            # rviz_config_arg,
            # rviz_node,
            # static_tf,
            # robot_state_publisher,
            # run_move_group_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # arm_controller_spawner,
            # hand_controller_spawner,
        ]
    )
