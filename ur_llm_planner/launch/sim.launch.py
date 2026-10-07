"""UR3/UR3e + Robotiq 2F-85 + Gazebo (ban, camera, 5 khoi, 3 vung) + MoveIt 2 + perception +
skill_server.

Vi du:
  ros2 launch ur_llm_planner sim.launch.py                 # UR3e
  ros2 launch ur_llm_planner sim.launch.py ur_type:=ur3
  ros2 launch ur_llm_planner sim.launch.py gazebo_gui:=false launch_rviz:=false

Sau do chay LLM planner o terminal khac:
  ros2 run ur_llm_planner llm_planner_node.py
"""

import math
import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def load_yaml(package, *path):
    with open(os.path.join(get_package_share_directory(package), *path)) as f:
        return yaml.safe_load(f)


def camera_pose_from_world(world_path, model_name="overhead_camera"):
    """Doc pose [x, y, z, roll, pitch, yaw] cua camera tu file world (tranh khai bao 2 noi)."""
    model = ET.parse(world_path).getroot().find(f".//model[@name='{model_name}']")
    if model is None or model.find("pose") is None:
        raise RuntimeError(f"Khong tim thay model '{model_name}' (co <pose>) trong {world_path}")
    pose = [float(v) for v in model.find("pose").text.split()]
    if len(pose) != 6:
        raise RuntimeError(f"<pose> cua '{model_name}' phai co 6 gia tri: {pose}")
    return pose


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    startup_delay = LaunchConfiguration("startup_delay")

    pkg_share = FindPackageShare("ur_llm_planner")
    world_file = PathJoinSubstitution([pkg_share, "worlds", "pick_place.sdf"])
    scene_config = PathJoinSubstitution([pkg_share, "config", "scene.yaml"])
    perception_config = PathJoinSubstitution([pkg_share, "config", "perception.yaml"])
    gripper_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "gripper_controller.yaml"])
    # Can kinematics.yaml de skill_server tu giai IK (chon cau hinh khop hop ly)
    kinematics_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"])

    object_names = load_yaml("ur_llm_planner", "config", "scene.yaml")[
        "/**"]["ros__parameters"]["object_names"]

    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3", "ur3e"],
                              description="Loai robot: ur3 hoac ur3e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("startup_delay", default_value="8.0",
                              description="Giay cho Gazebo/controller truoc khi chay skill_server"),
    ]

    # 1) Gazebo + ros2_control (dung lai launch cua ur_simulation_gz, thay world va URDF co gripper)
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch",
                                  "ur_sim_control.launch.py"])),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur_llm_planner",
            "description_file": "ur_robotiq.urdf.xacro",
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager",
                   "--param-file", gripper_controller_config,
                   "--controller-manager-timeout", "120"],
        output="screen",
    )

    # 2) MoveIt 2: move_group + RViz voi URDF/SRDF co gripper (cac cau hinh con lai cua
    #    ur_moveit_config giu nguyen)
    robot_description = {"robot_description": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "urdf", "ur_robotiq.urdf.xacro"]),
        " name:=ur ur_type:=", ur_type, " safety_limits:=true use_fake_hardware:=true",
    ]), value_type=str)}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "srdf", "ur_robotiq.srdf.xacro"]), " name:=ur",
    ]), value_type=str)}

    ompl = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl.update(load_yaml("ur_moveit_config", "config", "ompl_planning.yaml"))

    # scaled_joint_trajectory_controller khong chay trong Gazebo
    controllers = load_yaml("ur_moveit_config", "config", "controllers.yaml")
    controllers["scaled_joint_trajectory_controller"]["default"] = False
    controllers["joint_trajectory_controller"]["default"] = True

    moveit_params = [
        robot_description,
        robot_description_semantic,
        kinematics_config,
        {"robot_description_planning":
            load_yaml("ur_moveit_config", "config", "joint_limits.yaml")},
        {"move_group": ompl},
        {"use_sim_time": True},
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params + [
            {"publish_robot_description_semantic": True},
            {"moveit_simple_controller_manager": controllers,
             "moveit_controller_manager":
                 "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"moveit_manage_controllers": False,
             "trajectory_execution.allowed_execution_duration_scaling": 1.2,
             "trajectory_execution.allowed_goal_duration_margin": 0.5,
             "trajectory_execution.allowed_start_tolerance": 0.01,
             "trajectory_execution.execution_duration_monitoring": False},
            {"publish_planning_scene": True,
             "publish_geometry_updates": True,
             "publish_state_updates": True,
             "publish_transforms_updates": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"])],
        parameters=moveit_params,
        condition=IfCondition(launch_rviz),
    )

    # 3) Bridge ROS 2 <-> Gazebo: anh camera (Gazebo -> ROS) va gan/nha vat vao gripper
    #    (DetachableJoint, ROS -> Gazebo). Khong co bridge set_pose: vat chi di chuyen do vat ly.
    gripper_topics = [f"/gripper/{name}/{action}@std_msgs/msg/Empty]ignition.msgs.Empty"
                      for name in object_names for action in ("attach", "detach")]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_gripper_bridge",
        arguments=gripper_topics,
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        arguments=["/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                   "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 4) TF cua camera: world -> camera_link (= pose cua model trong world) -> camera_optical_frame
    #    (z nhin ra truoc, x phai, y xuong; camera Gazebo nhin theo +X cua link).
    cx, cy, cz, croll, cpitch, cyaw = camera_pose_from_world(
        os.path.join(get_package_share_directory("ur_llm_planner"), "worlds", "pick_place.sdf"))
    camera_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_tf",
        arguments=["--x", str(cx), "--y", str(cy), "--z", str(cz), "--roll", str(croll),
                   "--pitch", str(cpitch), "--yaw", str(cyaw),
                   "--frame-id", "world", "--child-frame-id", "camera_link"],
    )
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_tf",
        arguments=["--roll", str(-math.pi / 2), "--pitch", "0", "--yaw", str(-math.pi / 2),
                   "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame"],
    )

    # 5) Perception: camera -> vi tri cac khoi + trang thai cac vung (/perception/scene)
    perception = Node(
        package="ur_llm_planner",
        executable="perception_node.py",
        name="perception_node",
        output="screen",
        parameters=[scene_config, perception_config, {"use_sim_time": True}],
    )

    # 6) Skill server: thuc thi robot skill bang MoveIt 2, vi tri vat lay tu perception
    skill_server = Node(
        package="ur_llm_planner",
        executable="skill_server",
        name="skill_server",
        output="screen",
        parameters=[scene_config, kinematics_config, {"use_sim_time": True}],
    )

    return LaunchDescription(declared_arguments + [
        ur_control,
        gripper_controller_spawner,
        move_group,
        rviz,
        gz_bridge,
        camera_bridge,
        camera_link_tf,
        camera_optical_tf,
        perception,
        TimerAction(period=startup_delay, actions=[skill_server]),
    ])
