import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_arm", package_name="dual_arm_configure")
        .robot_description(file_path="config/dual_arm_demo.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    namespace = "dual_arm"

    ld = LaunchDescription()

    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    pub_freq = DeclareLaunchArgument("publish_frequency", default_value="15.0")

    allow_traj_exec = DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    pub_mon_planning_scene = DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    cap = DeclareLaunchArgument("capabilities", default_value=moveit_config.move_group_capabilities["capabilities"])
    dis_cap= DeclareLaunchArgument("disable_capabilities", default_value=moveit_config.move_group_capabilities["disable_capabilities"])
    mon_dynamics = DeclareBooleanLaunchArg("monitor_dynamics", default_value=False)

    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_configure"), "config", "moveit.rviz"
    )
    
    name_counter = 0
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    namespace=namespace,
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
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
            ("~/robot_description", f"/robot_description"),
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
        remappings=[
            ('~/tf', '/tf'),
            ('~/tf_static', '/tf_static')
        ]
    )   

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # namespace=namespace,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="log",
        respawn=False,
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
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
            ("~/robot_description", f"/{namespace}/robot_description"),
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
                    # ('~/tf', 'tf'),
                    # ('~/tf_static', 'tf_static')
                ]
            )
        )
        
    ld.add_action(use_rviz)
    ld.add_action(pub_freq)

    ld.add_action(allow_traj_exec)
    ld.add_action(pub_mon_planning_scene)
    ld.add_action(cap)
    ld.add_action(dis_cap)
    ld.add_action(mon_dynamics)
    
    ld.add_action(move_group)
    ld.add_action(rsp)
    ld.add_action(rviz)
    ld.add_action(ros2_control)
    
    return ld