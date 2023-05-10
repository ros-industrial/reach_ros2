from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


parameters = [
  {'name': 'robot_description_file',                'description': 'Path to the URDF/xacro file',                     'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'model', 'reach_study.xacro'])},
  {'name': 'robot_description_semantic_file',       'description': 'Path to the SRDF file',                           'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'model', 'reach_study.srdf'])},
  {'name': 'robot_description_kinematics_file',     'description': 'Path to the MoveIt kinematics file',              'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'model', 'kinematics.yaml'])},
  {'name': 'robot_description_joint_limits_file',   'description': 'Path to the MoveIt joint limits file',            'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'model', 'joint_limits.yaml'])},
  {'name': 'config_file',                           'description': 'Path to the reach study configuration YAML file', 'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'config', 'reach_study.yaml'])},
  {'name': 'config_name',                           'description': 'Reach study configuration name',                  'default': 'reach_study'},
  {'name': 'results_dir',                           'description': 'Directory in which to save reach study results',  'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'results'])},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    robot_description_file = LaunchConfiguration('robot_description_file')
    robot_description_semantic_file = LaunchConfiguration('robot_description_semantic_file')
    robot_description_kinematics_file = LaunchConfiguration('robot_description_kinematics_file')
    robot_description_joints_limits_file = LaunchConfiguration('robot_description_joint_limits_file')

    robot_description = ParameterValue(Command(['xacro ', robot_description_file]), value_type=str)
    robot_description_semantic = load_file(robot_description_semantic_file.perform(context))
    kinematics_yaml = load_yaml(robot_description_kinematics_file.perform(context))
    joint_limits_yaml = load_yaml(robot_description_joints_limits_file.perform(context))

    os.environ['REACH_PLUGINS'] = 'reach_ros_plugins'

    return [
        Node(
            package='reach_ros',
            executable='reach_ros_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
                'config_file': LaunchConfiguration('config_file'),
                'config_name': LaunchConfiguration('config_name'),
                'results_dir': LaunchConfiguration('results_dir')}]
        )
    ]
