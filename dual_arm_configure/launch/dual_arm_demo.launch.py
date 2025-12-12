import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "xacro_file",
            default_value="dual_arm_demo.urdf.xacro",
            description=""
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "left_sim",
            default_value="false",
            description=""
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can0",
            description=""
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "right_sim",
            default_value="false",
            description=""
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description=""
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="dual_arm_configure")
        .robot_description(file_path="config/dual_arm_demo.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    ld.add_action(move_group_node)

    rviz_config = os.path.join(get_package_share_directory("dual_arm_configure"), "config", "moveit.rviz")
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    ld.add_action(rviz_node)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(robot_state_publisher)

    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_configure"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )
    ld.add_action(ros2_control_node)
    
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager", "/controller_manager"
                ]
            )
        )

    return ld