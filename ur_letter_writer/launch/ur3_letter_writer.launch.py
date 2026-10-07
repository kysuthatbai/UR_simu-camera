"""
Launch file: UR3/UR3e Gazebo simulation + MoveIt2 + node ve chu cai.

Vi du chay:
  ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3e name:=Bao
  ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3 letter:=M
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def pv_double(name: str):
    """Ep kieu tham so tu launch argument (string) sang double cho ROS param."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type", default_value="ur3e",
            description="Loai robot UR: ur3 hoac ur3e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "startup_delay", default_value="12.0",
            description="So giay cho Gazebo/MoveIt san sang truoc khi chay control node"),

        DeclareLaunchArgument(
            "name", default_value="Bao",
            description="Ten sinh vien - chu cai dau tien se duoc ve"),
        DeclareLaunchArgument(
            "letter", default_value="",
            description="Ghi de truc tiep chu can ve (A-Z); neu dat thi bo qua 'name'"),
        DeclareLaunchArgument("planning_group", default_value="ur_manipulator"),

        # Tam Cartesian cua chu (trong he planning frame, thuong la base_link)
        DeclareLaunchArgument("plane_center_x", default_value="0.35"),
        DeclareLaunchArgument("plane_center_y", default_value="0.0"),
        DeclareLaunchArgument("plane_center_z", default_value="0.30"),

        # Truc "ngang" (u) va "doc" (v) cua chu trong khong gian - chon mat phang tuy y
        DeclareLaunchArgument("plane_u_axis_x", default_value="0.0"),
        DeclareLaunchArgument("plane_u_axis_y", default_value="1.0"),
        DeclareLaunchArgument("plane_u_axis_z", default_value="0.0"),
        DeclareLaunchArgument("plane_v_axis_x", default_value="0.0"),
        DeclareLaunchArgument("plane_v_axis_y", default_value="0.0"),
        DeclareLaunchArgument("plane_v_axis_z", default_value="1.0"),

        DeclareLaunchArgument("letter_width", default_value="0.18"),
        DeclareLaunchArgument("letter_height", default_value="0.22"),
        DeclareLaunchArgument("retract_distance", default_value="0.05"),

        DeclareLaunchArgument("eef_step", default_value="0.005"),
        DeclareLaunchArgument("min_fraction", default_value="0.95"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.2"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.2"),
    ]

    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    startup_delay = LaunchConfiguration("startup_delay")

    # 1) UR3/UR3e simulation (Gazebo) + MoveIt2 + RViz
    #    Dung lai launch file co san trong Universal_Robots_ROS2_GZ_Simulation,
    #    khong tu viet lai simulator / controller / moveit config.
    ur_sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # 2) Node dieu khien ve chu, tham so co the ghi de tu dong lenh
    letter_writer_node = Node(
        package="ur_letter_writer",
        executable="letter_writer_node",
        name="letter_writer_node",
        output="screen",
        parameters=[{
            "student_name": LaunchConfiguration("name"),
            "letter": LaunchConfiguration("letter"),
            "planning_group": LaunchConfiguration("planning_group"),

            "plane_center_x": pv_double("plane_center_x"),
            "plane_center_y": pv_double("plane_center_y"),
            "plane_center_z": pv_double("plane_center_z"),

            "plane_u_axis_x": pv_double("plane_u_axis_x"),
            "plane_u_axis_y": pv_double("plane_u_axis_y"),
            "plane_u_axis_z": pv_double("plane_u_axis_z"),
            "plane_v_axis_x": pv_double("plane_v_axis_x"),
            "plane_v_axis_y": pv_double("plane_v_axis_y"),
            "plane_v_axis_z": pv_double("plane_v_axis_z"),

            "letter_width": pv_double("letter_width"),
            "letter_height": pv_double("letter_height"),
            "retract_distance": pv_double("retract_distance"),

            "eef_step": pv_double("eef_step"),
            "min_fraction": pv_double("min_fraction"),
            "velocity_scaling": pv_double("velocity_scaling"),
            "acceleration_scaling": pv_double("acceleration_scaling"),
        }],
    )

    # Cho Gazebo + controller_manager + MoveIt san sang truoc khi goi MoveGroupInterface
    delayed_letter_writer_node = TimerAction(
        period=startup_delay,
        actions=[letter_writer_node],
    )

    return LaunchDescription(
        declared_arguments + [
            ur_sim_moveit_launch,
            delayed_letter_writer_node,
        ]
    )
