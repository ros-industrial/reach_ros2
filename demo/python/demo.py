#!/usr/bin/env python3
import reach
import reach_ros

from ament_index_python.packages import get_package_share_directory
import os.path
from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.logging import LoggingSeverity
import sys
import xacro
import yaml


def parse_yaml(parameter_file, namespace=''):
    with open(parameter_file, 'r') as f:
        param_dict = yaml.safe_load(f)
        return parse_parameter_dict(namespace=namespace, parameter_dict=param_dict)


def parse_parameter_dict(*, namespace, parameter_dict):
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if type(param_value) == dict:
            parameters += parse_parameter_dict(
                namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                parameter_dict=param_value)
        else:
            parameter = Parameter(name=full_param_name, value=param_value)
            parameters.append(parameter)
    return parameters


def main():
    # initialize ROS with any parameters provided as arguments
    reach_ros.init_ros(sys.argv)

    # Set logger level to reduce MoveIt message spam
    moveit_loggers = ["moveit_ros.robot_model_loader",
                      "moveit_kinematics_base.kinematics_base",
                      "moveit_rdf_loader.rdf_loader",
                      "moveit_robot_model.robot_model"]
    for logger_name in moveit_loggers:
        reach_ros.set_logger_level(
            logger_name, LoggingSeverity.WARN)

    # Manually load the parameters necessary for running MoveIt.
    reach_ros_dir = get_package_share_directory('reach_ros')
    model_dir = os.path.join(reach_ros_dir, 'demo', 'model')

    # Generate the URDF string from the xacro file
    urdf = xacro.process(os.path.join(model_dir, 'reach_study.xacro'))

    # Load the SRDF
    with open(os.path.join(model_dir, 'reach_study.srdf'), 'r') as f:
        srdf = f.read()

    moveit_parameters = [
        *parse_yaml(os.path.join(model_dir, 'kinematics.yaml'), 'robot_description_kinematics.'),
        *parse_yaml(os.path.join(model_dir, 'joint_limits.yaml'), 'robot_description_joint_limits.'),
        Parameter(name="robot_description", value=urdf),
        Parameter(name="robot_description_semantic", value=srdf),
    ]

    for parameter in moveit_parameters:
        # We don't need to declare the parameters as the node allows undeclared parameters
        reach_ros.set_parameter(parameter.name, parameter.value)

    with open(os.path.join(reach_ros_dir, 'demo', 'config', 'reach_study.yaml'), 'r') as f:
        config = yaml.safe_load(f)

    # Set the search libraries environment variable to include the plugin library provided by this package
    os.environ[reach.SEARCH_LIBRARIES_ENV] = 'reach_ros_plugins'

    # Disable the optimization steps to make this demo faster and highlight the performance difference
    config['optimization']['max_steps'] = 0

    print("Starting first study with default kinematic parameters")
    save_dir = "/tmp"
    study_name = "study1"
    reach.runReachStudy(config, study_name, save_dir, False)

    # Loading the study results
    results_1 = reach.load(os.path.join(save_dir, study_name, 'reach.db.xml')).calculateResults()

    # Decrease the IK solver time and see if we still find equally good solutions
    reach_ros.set_parameter(
        "robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.00005)

    print("Starting second study with decreased solver resolution.")
    study_name = 'study2'
    reach.runReachStudy(config, study_name, save_dir, False)

    # Loading the study results
    results_2 = reach.load(os.path.join(save_dir, study_name, 'reach.db.xml')).calculateResults()

    print(f'Both REACH studies finished')
    print(f'\tOriginal parameters: Score {results_1.total_pose_score:.2f}, Reached {results_1.reach_percentage:.2f}%')
    print(f'\tModified parameters: Score {results_2.total_pose_score:.2f}, Reached {results_2.reach_percentage:.2f}%')


if __name__ == '__main__':
    main()
