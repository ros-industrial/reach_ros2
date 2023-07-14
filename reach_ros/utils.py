from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
import yaml


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


def parse_yaml(parameter_file, namespace=''):
    with open(parameter_file, 'r') as f:
        param_dict = yaml.safe_load(f)
        return parse_parameter_dict(namespace=namespace, parameter_dict=param_dict)
