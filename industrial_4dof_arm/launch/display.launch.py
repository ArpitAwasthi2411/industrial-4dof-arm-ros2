"""Display the 4-DOF industrial arm in RViz2.

Starts:
  * robot_state_publisher      - publishes /robot_description and the TF tree
  * joint_state_publisher_gui  - slider GUI publishing /joint_states
                                 (joint_state_publisher when gui:=false)
  * rviz2                      - preconfigured with Grid, RobotModel and TF

Usage:
  ros2 launch industrial_4dof_arm display.launch.py
  ros2 launch industrial_4dof_arm display.launch.py gui:=false
  ros2 launch industrial_4dof_arm display.launch.py wrist_axis:=roll
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('industrial_4dof_arm')

    default_model = PathJoinSubstitution([pkg_share, 'urdf', 'industrial_arm.urdf.xacro'])
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    model = LaunchConfiguration('model')
    wrist_axis = LaunchConfiguration('wrist_axis')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_args = [
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot Xacro/URDF file.'),
        DeclareLaunchArgument(
            'wrist_axis',
            default_value='pitch',
            choices=['pitch', 'roll'],
            description='Joint 4 axis: pitch (about Y) or roll (about X).'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
            description='Start joint_state_publisher_gui (true) or joint_state_publisher (false).'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz2.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to the RViz2 configuration file.'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', model, ' wrist_axis:=', wrist_axis]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui))

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(gui))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz))

    return LaunchDescription(declare_args + [
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
