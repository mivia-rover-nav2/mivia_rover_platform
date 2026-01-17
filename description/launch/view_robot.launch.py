from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mivia_rover_platform",
            description="Package containing the top-level URDF/xacro (instantiates the robot).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="mivia_rover.urdf.xacro",
            description="Top-level URDF/XACRO file describing the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and Joint State Publisher GUI automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint/link names (multi-robot).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="If true, select mock hardware in ros2_control blocks (if present in the xacro).",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "description",
                    "urdf",
                    description_file,
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # RViz config in mivia_rover_description (as per your setup)
    #rviz_config_file = PathJoinSubstitution(
    #    [
    #        FindPackageShare("mivia_rover_description"),
    #        "rviz",
    #        "mivia_rover.rviz",
    #    ]
    #)

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    #rviz_node = Node(
    #    package="rviz2",
    #    executable="rviz2",
    #    name="rviz2",
    #    output="log",
    #    arguments=["-d", rviz_config_file],
    #    condition=IfCondition(gui),
    #)

    return LaunchDescription(declared_arguments + [
        joint_state_publisher_node,
        robot_state_publisher_node,
        #rviz_node,
    ])