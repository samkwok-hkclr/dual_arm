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
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="dual_arm_configure")
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # moveit_config.robot_description = ""

    namespace = "dual_arm"

    ld = LaunchDescription()
    
    # Declare launch arguments
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value='dual_arm_robot.urdf.xacro',
        description='Path to the XACRO file relative to package share/urdf'
    )
    ld.add_action(xacro_file_arg)

    allow_traj_exec = DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    pub_mon_planning_scene = DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    cap = DeclareLaunchArgument("capabilities", default_value=moveit_config.move_group_capabilities["capabilities"])
    dis_cap= DeclareLaunchArgument("disable_capabilities", default_value=moveit_config.move_group_capabilities["disable_capabilities"])
    mon_dynamics = DeclareBooleanLaunchArg("monitor_dynamics", default_value=False)
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            get_package_share_directory("dual_arm_robot_description"),
            'urdf',
            LaunchConfiguration('xacro_file')
        ])
    ])
    
    move_group_configuration = {
        'robot_description': robot_description_content,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
    


    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=move_group_params,
        # Set the display variable, in case OpenGL code is used internally
        # additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("~/robot_description_semantic", "/robot_description_semantic"),
        ],
    )
    
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            # moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    controller,
                    "-c", f"/{namespace}/controller_manager"
                ],
                output="screen",
                remappings=[
                    ('~/joint_states', '/joint_states'),
                    ("~/robot_description", "/robot_description"),
                #     ('~/tf', 'tf'),
                #     ('~/tf_static', 'tf_static')
                ]
            )
        )
        
    ld.add_action(allow_traj_exec)
    ld.add_action(pub_mon_planning_scene)
    ld.add_action(cap)
    ld.add_action(dis_cap)
    ld.add_action(mon_dynamics)
    
    ld.add_action(move_group)
    ld.add_action(ros2_control)
    
    return ld